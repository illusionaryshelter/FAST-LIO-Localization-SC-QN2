#include <cinttypes>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/logging.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2/transform_datatypes.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include <kindr/minimal/quat-transformation.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include <gflags/gflags.h>
#include <gflags/gflags_gflags.h>

struct EIGEN_ALIGN16 PointKITTI {
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(PointKITTI,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity,
                                      intensity)(float, time, time)(uint16_t,
                                                                    ring, ring))

namespace kitti {

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Rotation;

struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Intrinsics.
  Eigen::Vector2d image_size = Eigen::Vector2d::Zero(); // S_xx in calibration.
  Eigen::Matrix3d rect_mat =
      Eigen::Matrix3d::Identity(); // R_rect_xx in calibration.
  Eigen::Matrix<double, 3, 4> projection_mat =
      Eigen::Matrix<double, 3, 4>::Identity(); // P_xx in calibration.

  // Unrectified (raw) intrinsics. Should only be used if rectified set to
  // false.
  Eigen::Matrix3d K =
      Eigen::Matrix3d::Zero(); // Camera intrinsics, K_xx in calibration.
  Eigen::Matrix<double, 1, 5> D =
      Eigen::Matrix<double, 1,
                    5>::Zero(); // Distortion parameters, radtan model.

  // Extrinsics.
  Transformation T_cam0_cam;

  bool distorted = false;
};

typedef std::vector<CameraCalibration,
                    Eigen::aligned_allocator<CameraCalibration>>
    CameraCalibrationVector;

// Where t = 0 means 100% left transformation,
// and t = 1 means 100% right transformation.
Transformation interpolateTransformations(const Transformation &left,
                                          const Transformation &right,
                                          double t) {
  Transformation output;
  // Linearly interpolate the position between the two.
  output.getPosition() = left.getPosition() * (1 - t) + right.getPosition() * t;

  // slerp the rotation between the two.
  output.getRotation().toImplementation() =
      left.getRotation().toImplementation().slerp(
          t, right.getRotation().toImplementation());

  return output;
}

std::string getCameraFrameId(int cam_id) {
  char buffer[20];
  sprintf(buffer, "cam%02d", cam_id);
  return std::string(buffer);
}

void calibrationToRos(uint64_t cam_id, const CameraCalibration &cam,
                      sensor_msgs::msg::CameraInfo *cam_msg) {
  cam_msg->header.frame_id = getCameraFrameId(cam_id);

  cam_msg->width = cam.image_size.x();
  cam_msg->height = cam.image_size.y();

  cam_msg->distortion_model = "plumb_bob";

  // D is otherwise empty by default, hopefully this is fine.
  if (cam.distorted) {
    const size_t kNumDistortionParams = 5;
    cam_msg->d.resize(kNumDistortionParams);
    for (size_t i = 0; i < kNumDistortionParams; ++i) {
      cam_msg->d[i] = cam.D(i);
    }
  }

  // Copy over intrinsics.
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->k[3 * i + j] = cam.K(i, j);
    }
  }

  // Rectification/projection matrices.
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->r[3 * i + j] = cam.rect_mat(i, j);
    }
  }

  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 4; ++j) {
      cam_msg->p[4 * i + j] = cam.projection_mat(i, j);
    }
  }

  // Set the translation of the projection matrix.
  // cam_msg->P[3] = cam.T_cam0_cam.getPosition().x();
  // cam_msg->P[7] = cam.T_cam0_cam.getPosition().y();
}

void stereoCalibrationToRos(uint64_t left_cam_id, uint64_t right_cam_id,
                            const CameraCalibration &left_cam,
                            const CameraCalibration &right_cam,
                            sensor_msgs::msg::CameraInfo *left_cam_msg,
                            sensor_msgs::msg::CameraInfo *right_cam_msg) {
  // Fill in the basics for each camera.
  calibrationToRos(left_cam_id, left_cam, left_cam_msg);
  calibrationToRos(right_cam_id, right_cam, right_cam_msg);

  // Since all transforms are given relative to cam0, need to remove the cam0
  // transform from both to get the relative one between the two (cam0 to cam0
  // is ostensibly identity anyway).
  // This is probably not even necessary.
  // Transformation T_left_cam0 = left_cam.T_cam0_cam.inverse();
  // Transformation T_left_right = T_left_cam0 * right_cam.T_cam0_cam;
}

void imageToRos(const cv::Mat &image, sensor_msgs::msg::Image *image_msg) {
  cv_bridge::CvImage image_cv_bridge;
  image_cv_bridge.image = image;

  if (image.type() == CV_8U) {
    image_cv_bridge.encoding = "mono8";
  } else if (image.type() == CV_8UC3) {
    image_cv_bridge.encoding = "bgr8";
  }
  image_cv_bridge.toImageMsg(*image_msg);
}

void poseToRos(const Transformation &transform,
               geometry_msgs::msg::PoseStamped *pose_msg) {
  tf::poseKindrToMsg(transform, &pose_msg->pose);
}

void transformToTf(const Transformation &transform,
                   tf2::Transform *tf_transform) {
  tf::transformKindrToTF(transform, tf_transform);
}

void transformToRos(const Transformation &transform,
                    geometry_msgs::msg::TransformStamped *transform_msg) {
  tf::transformKindrToMsg(transform, &transform_msg->transform);
}

void timestampToRos(uint64_t timestamp_ns, rclcpp::Time *time) {
  *time = rclcpp::Time(timestamp_ns);
}

class KittiParser {
public:
  // Constants for filenames for calibration files.
  static inline const std::string kVelToCamCalibrationFilename =
      "calib_velo_to_cam.txt";
  static inline const std::string kCamToCamCalibrationFilename =
      "calib_cam_to_cam.txt";
  static inline const std::string kImuToVelCalibrationFilename =
      "calib_imu_to_velo.txt";

  static inline const std::string kVelodyneFolder = "velodyne_points";
  static inline const std::string kCameraFolder = "image_";
  static inline const std::string kPoseFolder = "oxts";

  static inline const std::string kTimestampFilename = "timestamps.txt";
  static inline const std::string kDataFolder = "data";

  KittiParser(const std::string &calibration_path,
              const std::string &dataset_path, bool rectified)
      : calibration_path_(calibration_path), dataset_path_(dataset_path),
        rectified_(rectified), initial_pose_set_(false) {}

  // MAIN API: all you should need to use!
  // Loading calibration files.
  bool loadCalibration() {
    loadVelToCamCalibration();
    loadImuToVelCalibration();
    loadCamToCamCalibration();
    return true;
  }

  void loadTimestampMaps() {
    // Load timestamps for poses.
    std::string filename =
        dataset_path_ + "/" + kPoseFolder + "/" + kTimestampFilename;
    loadTimestampsIntoVector(filename, &timestamps_pose_ns_);

    std::cout << "Timestmap map for pose:\n";
    for (size_t i = 0; i < timestamps_pose_ns_.size(); ++i) {
      std::cout << i << " " << timestamps_pose_ns_[i] << std::endl;
    }

    // Velodyne.
    filename = dataset_path_ + "/" + kVelodyneFolder + "/" + kTimestampFilename;
    loadTimestampsIntoVector(filename, &timestamps_vel_ns_);

    // One per camera.
    timestamps_cam_ns_.resize(camera_calibrations_.size());
    for (size_t i = 0; i < camera_calibrations_.size(); ++i) {
      filename = dataset_path_ + "/" + getFolderNameForCamera(i) + "/" +
                 kTimestampFilename;
      loadTimestampsIntoVector(filename, &timestamps_cam_ns_[i]);
    }
  }

  // Load specific entries (indexed by filename).
  bool getPoseAtEntry(uint64_t entry, uint64_t *timestamp,
                      Transformation *pose) {
    std::string filename = dataset_path_ + "/" + kPoseFolder + "/" +
                           kDataFolder + "/" + getFilenameForEntry(entry) +
                           ".txt";

    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
      return false;
    }
    if (timestamps_pose_ns_.size() <= entry) {
      return false;
    }
    *timestamp = timestamps_pose_ns_[entry];

    std::string line;
    std::vector<double> parsed_doubles;
    while (std::getline(import_file, line)) {
      if (parseVectorOfDoubles(line, &parsed_doubles)) {
        if (convertGpsToPose(parsed_doubles, pose)) {
          return true;
        }
      }
    }
    return false;
  }

  uint64_t getPoseTimestampAtEntry(uint64_t entry) {
    if (timestamps_pose_ns_.size() <= entry) {
      return 0;
    }
    return timestamps_pose_ns_[entry];
  }

  bool interpolatePoseAtTimestamp(uint64_t timestamp, Transformation *pose) {
    // Look up the closest 2 timestamps to this.
    size_t left_index = timestamps_pose_ns_.size();
    for (size_t i = 0; i < timestamps_pose_ns_.size(); ++i) {
      if (timestamps_pose_ns_[i] > timestamp) {
        if (i == 0) {
          // Then we can't interpolate the pose since we're outside the range.
          return false;
        }
        left_index = i - 1;
        break;
      }
    }
    if (left_index >= timestamps_pose_ns_.size()) {
      return false;
    }
    // Make sure we don't go over the size
    // if (left_index == timestamps_pose_ns_.size() - 1) {
    //  left_index--;
    //}

    // Figure out what 't' should be, where t = 0 means 100% left boundary,
    // and t = 1 means 100% right boundary.
    double t = (timestamp - timestamps_pose_ns_[left_index]) /
               static_cast<double>(timestamps_pose_ns_[left_index + 1] -
                                   timestamps_pose_ns_[left_index]);

    std::cout << "Timestamp: " << timestamp
              << " timestamp left: " << timestamps_pose_ns_[left_index]
              << " timestamp right: " << timestamps_pose_ns_[left_index + 1]
              << " t: " << t << std::endl;

    // Load the two transformations.
    uint64_t timestamp_left, timestamp_right;
    Transformation transform_left, transform_right;
    if (!getPoseAtEntry(left_index, &timestamp_left, &transform_left) ||
        !getPoseAtEntry(left_index + 1, &timestamp_right, &transform_right)) {
      // For some reason couldn't load the poses.
      return false;
    }

    // Interpolate between them.
    *pose = interpolateTransformations(transform_left, transform_right, t);
    return true;
  }

  bool getGpsAtEntry() { /* TODO! */
    return false;
  }

  bool getImuAtEntry(uint64_t entry, uint64_t *timestamp,
                     sensor_msgs::msg::Imu *imu) {
    std::string filename = dataset_path_ + "/" + kPoseFolder + "/" +
                           kDataFolder + "/" + getFilenameForEntry(entry) +
                           ".txt";

    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
      return false;
    }
    if (timestamps_pose_ns_.size() <= entry) {
      return false;
    }
    *timestamp = timestamps_pose_ns_[entry];

    std::string line;
    std::vector<double> parsed_doubles;
    while (std::getline(import_file, line)) {
      if (parseVectorOfDoubles(line, &parsed_doubles)) {
        if (convertGpsToImu(parsed_doubles, imu)) {
          return true;
        }
      }
    }
    return false;
  }

  bool getPointcloudAtEntry(uint64_t entry, uint64_t *timestamp,
                            pcl::PointCloud<PointKITTI> *ptcloud) {
    // Get the timestamp for this first.
    if (timestamps_vel_ns_.size() <= entry) {
      std::cout << "Warning: no timestamp for this entry!\n";
      return false;
    }

    *timestamp = timestamps_vel_ns_[entry];

    // Load the actual pointcloud.
    const size_t kMaxNumberOfPoints = 1e6; // From Readme for raw files.
    ptcloud->clear();
    ptcloud->reserve(kMaxNumberOfPoints);

    std::string filename = dataset_path_ + "/" + kVelodyneFolder + "/" +
                           kDataFolder + "/" + getFilenameForEntry(entry) +
                           ".bin";

    std::ifstream input(filename, std::ios::in | std::ios::binary);
    if (!input) {
      std::cout << "Could not open pointcloud file.\n";
      return false;
    }

    // From yanii's kitti-pcl toolkit:
    // https://github.com/yanii/kitti-pcl/blob/master/src/kitti2pcd.cpp
    for (size_t i = 0; input.good() && !input.eof(); i++) {
      PointKITTI point;
      input.read((char *)&point.x, 3 * sizeof(float));
      input.read((char *)&point.intensity, sizeof(float));
      point.ring = 64;
      point.time = 0;//((*timestamp)/1000);//ms
      ptcloud->push_back(point);
    }
    input.close();
    return true;
  }

  bool getImageAtEntry(uint64_t entry, uint64_t cam_id, uint64_t *timestamp,
                       cv::Mat *image) {
    // Get the timestamp for this first.
    if (timestamps_cam_ns_.size() <= cam_id ||
        timestamps_cam_ns_[cam_id].size() <= entry) {
      std::cout << "Warning: no timestamp for this entry!\n";
      return false;
    }
    *timestamp = (timestamps_cam_ns_[cam_id])[entry];

    std::string filename = dataset_path_ + "/" +
                           getFolderNameForCamera(cam_id) + "/" + kDataFolder +
                           "/" + getFilenameForEntry(entry) + ".png";

    *image = cv::imread(filename, cv::IMREAD_UNCHANGED);

    if (!image->data) {
      std::cout << "Could not load image data.\n";
      return false;
    }
    return true;
  }

  bool getCameraCalibration(uint64_t cam_id, CameraCalibration *cam) const {
    if (cam_id >= camera_calibrations_.size()) {
      return false;
    }
    *cam = camera_calibrations_[cam_id];
    return true;
  }

  Transformation T_camN_vel(int cam_number) const {
    return camera_calibrations_[cam_number].T_cam0_cam * T_cam0_vel_;
  }

  Transformation T_camN_imu(int cam_number) const {
    return T_camN_vel(cam_number) * T_vel_imu_;
  }

  // Returns the nanosecond timestamp since epoch for a particular entry.
  // Returns -1 if no valid timestamp is found.
  // int64_t getTimestampNsAtEntry(int64_t entry) const;

  // Basic accessors.
  Transformation T_cam0_vel() const { return T_cam0_vel_; }
  Transformation T_vel_imu() const { return T_vel_imu_; }

  size_t getNumCameras() const { return camera_calibrations_.size(); }

private:
  bool loadCamToCamCalibration() {
    std::string filename =
        calibration_path_ + "/" + kCamToCamCalibrationFilename;
    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
      return false;
    }

    std::string line;
    while (std::getline(import_file, line)) {
      std::stringstream line_stream(line);

      // Check what the header is. Each line consists of two parts:
      // a header followed by a ':' followed by space-separated data.
      std::string header;
      std::getline(line_stream, header, ':');
      std::string data;
      std::getline(line_stream, data, ':');

      std::vector<double> parsed_doubles;
      // Compare all header possibilities...

      std::cout << "Header: " << header << "\n";

      // Load image size.
      if (header.compare(0, 6, "S_rect") == 0) {
        std::cout << "S_rect header: " << header
                  << " substring: " << header.substr(7) << std::endl;
        if (rectified_) {
          // Figure out which number this is.
          int index = std::stoi(header.substr(7));
          std::cout << "Index: " << index << std::endl;
          if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
            camera_calibrations_.resize(index + 1);
          }
          if (parseVectorOfDoubles(data, &parsed_doubles)) {
            camera_calibrations_[index].image_size =
                Eigen::Vector2d(parsed_doubles.data());
            std::cout << "Image size: "
                      << camera_calibrations_[index].image_size << std::endl;
          }
        }
      } else if (!rectified_ && header.compare(0, 2, "S_") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].image_size =
              Eigen::Vector2d(parsed_doubles.data());
        }
      }

      // Load rectification matrix.
      if (header.compare(0, 6, "R_rect") == 0) {
        std::cout << "R_rect header: " << header
                  << " substring: " << header.substr(7) << std::endl;
        if (rectified_) {
          // Figure out which number this is.
          int index = std::stoi(header.substr(7));
          if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
            camera_calibrations_.resize(index + 1);
          }
          if (parseVectorOfDoubles(data, &parsed_doubles)) {
            camera_calibrations_[index].rect_mat =
                Eigen::Matrix3d(parsed_doubles.data()).transpose();
          }
        }
        continue;
      }

      // Projection mat.
      if (header.compare(0, 6, "P_rect") == 0) {
        std::cout << "P_rect header: " << header
                  << " substring: " << header.substr(7) << std::endl;
        if (rectified_) {
          // Figure out which number this is.
          int index = std::stoi(header.substr(7));
          if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
            camera_calibrations_.resize(index + 1);
          }
          if (parseVectorOfDoubles(data, &parsed_doubles)) {
            camera_calibrations_[index].projection_mat =
                Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();

            std::cout << "Projection mat:\n"
                      << camera_calibrations_[index].projection_mat
                      << std::endl;
          }
        }
        continue;
      } else if (!rectified_ && header.compare(0, 2, "P_") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].projection_mat =
              Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();
        }
        continue;
      }

      // Try to load unrectified K, and if using raw images, also load D.
      if (header.compare(0, 1, "K") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
          camera_calibrations_.resize(index + 1);
        }
        // Parse the rotation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix3d K(parsed_doubles.data());
          // All matrices are written row-major but Eigen is column-major
          // (can swap this, but I guess it's anyway easier to just transpose
          // for these small matrices).
          camera_calibrations_[index].K = K.transpose();
        }
        continue;
      }
      if (!rectified_) {
        if (header.compare(0, 1, "D") == 0) {
          int index = std::stoi(header.substr(2));
          if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
            camera_calibrations_.resize(index + 1);
          }
          // Parse the distortion vector.
          if (parseVectorOfDoubles(data, &parsed_doubles)) {
            Eigen::Matrix<double, 1, 5> D(parsed_doubles.data());
            camera_calibrations_[index].D = D;
            camera_calibrations_[index].distorted = !rectified_;
          }
          continue;
        }
      }

      if (header.compare(0, 1, "R") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
          camera_calibrations_.resize(index + 1);
        }
        // Parse the rotation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix3d R(parsed_doubles.data());
          // All matrices are written row-major but Eigen is column-major
          // (can swap this, but I guess it's anyway easier to just transpose
          // for these small matrices).
          camera_calibrations_[index].T_cam0_cam.getRotation() =
              Rotation::fromApproximateRotationMatrix(R.transpose());
        }
        continue;
      } else if (header.compare(0, 1, "T") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= static_cast<size_t>(index)) {
          camera_calibrations_.resize(index + 1);
        }
        // Parse the translation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Vector3d T(parsed_doubles.data());
          camera_calibrations_[index].T_cam0_cam.getPosition() = T;
        }
        continue;
      }
    }
    return true;
  }

  bool loadVelToCamCalibration() {
    std::string filename =
        calibration_path_ + "/" + kVelToCamCalibrationFilename;
    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
      return false;
    }

    std::string line;
    while (std::getline(import_file, line)) {
      std::stringstream line_stream(line);

      // Check what the header is. Each line consists of two parts:
      // a header followed by a ':' followed by space-separated data.
      std::string header;
      std::getline(line_stream, header, ':');
      std::string data;
      std::getline(line_stream, data, ':');

      std::vector<double> parsed_doubles;
      // Compare all header possibilities...
      if (header.compare("R") == 0) {
        // Parse the rotation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix3d R(parsed_doubles.data());
          // All matrices are written row-major but Eigen is column-major
          // (can swap this, but I guess it's anyway easier to just transpose
          // for these small matrices).
          T_cam0_vel_.getRotation() =
              Rotation::fromApproximateRotationMatrix(R.transpose());
        }
      } else if (header.compare("T") == 0) {
        // Parse the translation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Vector3d T(parsed_doubles.data());
          T_cam0_vel_.getPosition() = T;
        }
      }
    }
    // How do we return false?
    return true;
  }

  bool loadImuToVelCalibration() {
    std::string filename =
        calibration_path_ + "/" + kImuToVelCalibrationFilename;
    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
      return false;
    }

    std::string line;
    while (std::getline(import_file, line)) {
      std::stringstream line_stream(line);

      // Check what the header is. Each line consists of two parts:
      // a header followed by a ':' followed by space-separated data.
      std::string header;
      std::getline(line_stream, header, ':');
      std::string data;
      std::getline(line_stream, data, ':');

      std::vector<double> parsed_doubles;
      // Compare all header possibilities...
      if (header.compare("R") == 0) {
        // Parse the rotation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix3d R(parsed_doubles.data());
          // All matrices are written row-major but Eigen is column-major
          // (can swap this, but I guess it's anyway easier to just transpose
          // for these small matrices).
          T_vel_imu_.getRotation() =
              Rotation::fromApproximateRotationMatrix(R.transpose());
        }
      } else if (header.compare("T") == 0) {
        // Parse the translation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Vector3d T(parsed_doubles.data());
          T_vel_imu_.getPosition() = T;
        }
      }
    }
    std::cout << "Transform T_vel_imu: " << T_vel_imu_.getTransformationMatrix()
              << std::endl;
    return true;
  }

  bool convertGpsToPose(const std::vector<double> &oxts, Transformation *pose) {
    if (oxts.size() < 6) {
      return false;
    }

    double lat = oxts[0];
    double lon = oxts[1];
    double alt = oxts[2];

    double roll = oxts[3];
    double pitch = oxts[4];
    double yaw = oxts[5];

    // Position.
    if (!initial_pose_set_) {
      mercator_scale_ = latToScale(lat);
    }
    Eigen::Vector2d mercator;
    latlonToMercator(lat, lon, mercator_scale_, &mercator);
    Eigen::Vector3d position(mercator.x(), mercator.y(), alt);

    // Rotation.
    const Eigen::AngleAxisd axis_roll(roll, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd axis_pitch(pitch, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd axis_yaw(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond rotation = axis_yaw * axis_pitch * axis_roll;

    Transformation transform(position, rotation);

    // Undo the initial transformation, if one is set.
    // If not, set it.
    // The transformation only undoes translation and yaw, not roll and pitch
    // (as these are observable from the gravity vector).
    if (!initial_pose_set_) {
      T_initial_pose_.getPosition() = transform.getPosition();
      T_initial_pose_.getRotation() = Rotation(axis_yaw);
      initial_pose_set_ = true;
    }
    // Get back to local coordinates.
    *pose = T_initial_pose_.inverse() * transform;

    return true;
  }

  bool convertGpsToImu(const std::vector<double> &oxts,
                       sensor_msgs::msg::Imu *imu) {
    if (oxts.size() < 23) {
      return false;
    }

    double roll = oxts[3];
    double pitch = oxts[4];
    double yaw = oxts[5];

    // Imu.
    // - roll:  roll angle (rad),  0 = level, positive = left side up, range:
    // -pi   .. +pi
    // - pitch: pitch angle (rad), 0 = level, positive = front down, range:
    // -pi/2 .. +pi/2
    // - yaw:   heading (rad),     0 = east,  positive = counter clockwise,
    // range: -pi   .. +pi
    tf2::Quaternion orientation;
    orientation.setRPY(roll, pitch, yaw);
    imu->orientation.w = orientation.w();
    imu->orientation.x = orientation.x();
    imu->orientation.y = orientation.y();
    imu->orientation.z = orientation.z();

    // - wf:    angular rate around forward axis (rad/s)
    // - wl:    angular rate around leftward axis (rad/s)
    // - wu:    angular rate around upward axis (rad/s)
    imu->angular_velocity.x = oxts[20];
    imu->angular_velocity.y = oxts[21];
    imu->angular_velocity.z = oxts[22];

    // - af:    forward acceleration (m/s^2)
    // - al:    leftward acceleration (m/s^2)
    // - au:    upward acceleration (m/s^2)
    imu->linear_acceleration.x = oxts[14];
    imu->linear_acceleration.y = oxts[15];
    imu->linear_acceleration.z = oxts[16];
    return true;
  }

  double latToScale(double lat) const { return cos(lat * M_PI / 180.0); }

  void latlonToMercator(double lat, double lon, double scale,
                        Eigen::Vector2d *mercator) const {
    double er = 6378137;
    mercator->x() = scale * lon * M_PI * er / 180.0;
    mercator->y() = scale * er * log(tan((90.0 + lat) * M_PI / 360.0));
  }

  bool loadTimestampsIntoVector(const std::string &filename,
                                std::vector<uint64_t> *timestamp_vec) const {
    std::ifstream import_file(filename, std::ios::in);
    if (!import_file) {
      return false;
    }

    timestamp_vec->clear();
    std::string line;
    while (std::getline(import_file, line)) {
      std::stringstream line_stream(line);

      std::string timestamp_string = line_stream.str();
      std::tm t = {};
      t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
      t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
      t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
      t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
      t.tm_min = std::stoi(timestamp_string.substr(14, 2));
      t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
      t.tm_isdst = -1;

      static const uint64_t kSecondsToNanoSeconds = 1e9;
      time_t time_since_epoch = mktime(&t);

      uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
                           std::stoi(timestamp_string.substr(20, 9));
      timestamp_vec->push_back(timestamp);
    }

    std::cout << "Timestamps: " << std::endl
              << timestamp_vec->front() << " " << timestamp_vec->back()
              << std::endl;

    return true;
  }

  bool parseVectorOfDoubles(const std::string &input,
                            std::vector<double> *output) const {
    output->clear();
    // Parse the line as a stringstream for space-delimeted doubles.
    std::stringstream line_stream(input);
    if (line_stream.eof()) {
      return false;
    }

    while (!line_stream.eof()) {
      std::string element;
      std::getline(line_stream, element, ' ');
      if (element.empty()) {
        continue;
      }
      try {
        output->emplace_back(std::stod(element));
      } catch (const std::exception &exception) {
        std::cout << "Could not parse number in import file.\n";
        return false;
      }
    }
    return true;
  }

  std::string getFolderNameForCamera(int cam_number) const {
    char buffer[20];
    sprintf(buffer, "%s%02d", kCameraFolder.c_str(), cam_number);
    return std::string(buffer);
  }

  std::string getFilenameForEntry(uint64_t entry) const {
    char buffer[20];
    sprintf(buffer, "%010lu", entry);
    return std::string(buffer);
  }

  // Base paths.
  std::string calibration_path_;
  std::string dataset_path_;
  // Whether this dataset contains raw or rectified images. This determines
  // which calibration is read.
  bool rectified_;

  // Cached calibration parameters -- std::vector of camera calibrations.
  CameraCalibrationVector camera_calibrations_;

  // Transformation chain (cam-to-cam extrinsics stored above in cam calib
  // struct).
  Transformation T_cam0_vel_;
  Transformation T_vel_imu_;

  // Timestamp map from index to nanoseconds.
  std::vector<uint64_t> timestamps_vel_ns_;
  std::vector<uint64_t> timestamps_pose_ns_;
  // Vector of camera timestamp vectors.
  std::vector<std::vector<uint64_t>> timestamps_cam_ns_;

  // Cached pose information, to correct to odometry frame (instead of absolute
  // world coordinates).
  bool initial_pose_set_;
  Transformation T_initial_pose_;
  double mercator_scale_;
};

class KittiBagConverter {
public:
  KittiBagConverter(const std::string &calibration_path,
                    const std::string &dataset_path,
                    const std::string &output_filename)
      : parser_(calibration_path, dataset_path, true), world_frame_id_("world"),
        imu_frame_id_("imu"), cam_frame_id_prefix_("/kitti/cam"),
        velodyne_frame_id_("velodyne"), pose_topic_("/kitti/pose_imu"),
        imu_topic_("kitti/oxts/imu"), transform_topic_("/kitti/transform_imu"),
        pointcloud_topic_("/kitti/velodyne_points") {
    // Load all the timestamp maps and calibration parameters.
    parser_.loadCalibration();
    parser_.loadTimestampMaps();

    rosbag2_storage::StorageOptions write_storage_options{};
    write_storage_options.uri = output_filename;
    write_storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    writer_.open(write_storage_options, converter_options);
  }

  void convertAll(uint64_t threshold = 0) {
    uint64_t entry = 0;
    while (convertEntry(entry)) {
      entry++;
      if (threshold != 0 && entry >= threshold) {
        break;
      }
    }
    std::cout << "Converted " << entry << " entries into a rosbag.\n";
  }

  bool convertEntry(uint64_t entry) {
    rclcpp::Time timestamp_ros;
    uint64_t timestamp_ns;
    Transformation pose;
    if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
      geometry_msgs::msg::PoseStamped pose_msg;
      geometry_msgs::msg::TransformStamped transform_msg;

      timestampToRos(timestamp_ns, &timestamp_ros);
      pose_msg.header.frame_id = world_frame_id_;
      pose_msg.header.stamp = timestamp_ros;
      transform_msg.header.frame_id = world_frame_id_;
      transform_msg.header.stamp = timestamp_ros;

      poseToRos(pose, &pose_msg);
      transformToRos(pose, &transform_msg);

      writer_.write(pose_msg, pose_topic_, timestamp_ros);
      writer_.write(transform_msg, transform_topic_, timestamp_ros);

      convertTf(timestamp_ns, pose);
    } else {
      return false;
    }

    sensor_msgs::msg::Imu imu;

    if (parser_.getImuAtEntry(entry, &timestamp_ns, &imu)) {
      timestampToRos(timestamp_ns, &timestamp_ros);
      imu.header.stamp = timestamp_ros;
      imu.header.frame_id = imu_frame_id_;

      writer_.write(imu, imu_topic_, timestamp_ros);
    }

    cv::Mat image;

    for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
      if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) {
        timestampToRos(timestamp_ns, &timestamp_ros);

        sensor_msgs::msg::Image image_msg;
        imageToRos(image, &image_msg);
        image_msg.header.stamp = timestamp_ros;
        image_msg.header.frame_id = getCameraFrameId(cam_id);

        // TODO(helenol): cache this.
        // Get the calibration info for this camera.
        CameraCalibration cam_calib;
        parser_.getCameraCalibration(cam_id, &cam_calib);
        sensor_msgs::msg::CameraInfo cam_info;
        calibrationToRos(cam_id, cam_calib, &cam_info);
        cam_info.header = image_msg.header;

        writer_.write(image_msg, getCameraFrameId(cam_id) + "/image_raw",
                      timestamp_ros);
        writer_.write(cam_info, getCameraFrameId(cam_id) + "/camera_info",
                      timestamp_ros);
      }
    }

    pcl::PointCloud<PointKITTI> pointcloud_kitti;
    sensor_msgs::msg::PointCloud2 point_msg;
    if (parser_.getPointcloudAtEntry(entry, &timestamp_ns, &pointcloud_kitti)) {
      timestampToRos(timestamp_ns, &timestamp_ros);

      // This value is in MICROSECONDS, not nanoseconds.
      pointcloud_kitti.header.stamp = timestamp_ns / 1000;
      pointcloud_kitti.header.frame_id = velodyne_frame_id_;
      pcl::toROSMsg(pointcloud_kitti, point_msg);

      writer_.write(point_msg, pointcloud_topic_, timestamp_ros);
    }
    return true;
  }

  void convertTf(uint64_t timestamp_ns, const Transformation &imu_pose) {
    tf2_msgs::msg::TFMessage tf_msg;
    rclcpp::Time timestamp_ros;

    timestampToRos(timestamp_ns, &timestamp_ros);

    Transformation T_imu_world = imu_pose;
    Transformation T_vel_imu = parser_.T_vel_imu();
    Transformation T_cam_imu;

    geometry_msgs::msg::TransformStamped tf_imu_world, tf_vel_imu, tf_cam_imu;
    transformToRos(T_imu_world, &tf_imu_world);
    tf_imu_world.header.frame_id = world_frame_id_;
    tf_imu_world.child_frame_id = imu_frame_id_;
    tf_imu_world.header.stamp = timestamp_ros;
    transformToRos(T_vel_imu.inverse(), &tf_vel_imu);
    tf_vel_imu.header.frame_id = imu_frame_id_;
    tf_vel_imu.child_frame_id = velodyne_frame_id_;
    tf_vel_imu.header.stamp = timestamp_ros;

    // Put them into one tf_msg.
    tf_msg.transforms.push_back(tf_imu_world);
    tf_msg.transforms.push_back(tf_vel_imu);

    for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
      T_cam_imu = parser_.T_camN_imu(cam_id);
      transformToRos(T_cam_imu.inverse(), &tf_cam_imu);
      tf_cam_imu.header.frame_id = imu_frame_id_;
      tf_cam_imu.child_frame_id = getCameraFrameId(cam_id);
      tf_cam_imu.header.stamp = timestamp_ros;
      tf_msg.transforms.push_back(tf_cam_imu);
    }

    writer_.write(tf_msg, "/tf", timestamp_ros);
  }

private:
  kitti::KittiParser parser_;

  rosbag2_cpp::Writer writer_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;

  std::string pose_topic_;
  std::string imu_topic_;
  std::string transform_topic_;
  std::string pointcloud_topic_;
};

} // namespace kitti

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  if (argc < 4) {
    std::cout << "arguments in order: "
                 "calibration_path dataset_path output_path threshold\n";
    std::cout << "Note: no trailing slashes.\n";
    return 0;
  }

  const std::string calibration_path = argv[1];
  const std::string dataset_path = argv[2];
  const std::string output_path = argv[3];
  uint64_t threshold = 0;
  if (argc >= 5)
    threshold = std::atoll(argv[4]);

  kitti::KittiBagConverter converter(calibration_path, dataset_path,
                                     output_path);
  converter.convertAll(threshold);

  std::cout << "convert bag in: " << output_path << '\n';

  return 0;
}