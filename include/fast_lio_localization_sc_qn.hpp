#pragma once
#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <nano_gicp/nano_gicp.hpp>
#include <nano_gicp/point_type_nano_gicp.hpp>

#include <quatro/quatro_module.h>

#include <Eigen/Eigen>

#include "map_matcher.hpp"
#include "pose_pcd.hpp"
#include "utilities.hpp"

#ifdef FASTLIO_LO_SC_QN_PARAM_DEBUG
#define GET_PARAM_DEBUG(name, param)                                           \
  this->get_parameter(name, param);                                            \
  RCLCPP_INFO_STREAM(this->get_logger(), name << ": " << param);
#else
#define GET_PARAM_DEBUG(name, param) this->get_parameter(name, param);
#endif

using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>
    odom_pcd_sync_pol;

class FastLioLocalizationScQn : public rclcpp::Node {
public:
  using PointCloudT = sensor_msgs::msg::PointCloud2;
  using PathT = nav_msgs::msg::Path;
  using MarkerT = visualization_msgs::msg::Marker;
  using PoseStampedT = geometry_msgs::msg::PoseStamped;
  using OdomT = nav_msgs::msg::Odometry;

private:
  ///// basic params
  std::string map_frame_;
  ///// shared data - odom and pcd
  std::mutex keyframes_mutex_, vis_mutex_;
  bool is_initialized_ = false;
  int current_keyframe_idx_ = 0;
  PosePcd last_keyframe_;
  std::vector<PosePcdReduced> saved_map_from_bag_;
  Eigen::Matrix4d last_corrected_TF_ = Eigen::Matrix4d::Identity();
  ///// map match
  double keyframe_dist_thr_;
  double voxel_res_;
  ///// visualize
  bool saved_map_vis_switch_ = true;
  tf2_ros::Buffer tfListener_buffer_;
  tf2_ros::TransformBroadcaster broadcaster_;
  tf2_ros::TransformListener tfListener_;
  nav_msgs::msg::Path raw_odom_path_, corrected_odom_path_;
  std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
      matched_pairs_xyz_; // for vis
  pcl::PointCloud<pcl::PointXYZ> raw_odoms_, corrected_odoms_;
  pcl::PointCloud<PointType> saved_map_pcd_; // for vis

  ///// ros

  rclcpp::Publisher<PointCloudT>::SharedPtr odom_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr corrected_odom_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr corrected_current_pcd_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr saved_map_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_src_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_dst_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_coarse_aligned_pub_;
  rclcpp::Publisher<PointCloudT>::SharedPtr debug_fine_aligned_pub_;
  rclcpp::Publisher<PathT>::SharedPtr path_pub_;
  rclcpp::Publisher<PathT>::SharedPtr corrected_path_pub_;

  rclcpp::Publisher<MarkerT>::SharedPtr map_match_pub_;
  rclcpp::Publisher<PoseStampedT>::SharedPtr realtime_pose_pub_;

  rclcpp::TimerBase::SharedPtr match_timer_;
  // odom, pcd sync subscriber
  std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>>
      sub_odom_pcd_sync_ = nullptr;
  std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry>>
      sub_odom_ = nullptr;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>
      sub_pcd_ = nullptr;
  ///// Map match
  std::shared_ptr<MapMatcher> map_matcher_;

public:
  explicit FastLioLocalizationScQn()
      : Node("FastLioLocalizationScQn"), tfListener_buffer_(this->get_clock()),
        broadcaster_(*this), tfListener_(tfListener_buffer_) {
    std::string saved_map_path;
    std::string pose_topic, pcd_topic;
    double map_match_hz;
    MapMatcherConfig mm_config;
    auto &gc = mm_config.gicp_config_;
    auto &qc = mm_config.quatro_config_;

    this->declare_parameter("basic.map_frame", map_frame_);
    this->declare_parameter("basic.saved_map", saved_map_path);
    this->declare_parameter("basic.saved_map_pose_topic", pose_topic);
    this->declare_parameter("basic.saved_map_pcd_topic", pcd_topic);
    this->declare_parameter("basic.map_match_hz", map_match_hz);
    this->declare_parameter("basic.visualize_voxel_size", voxel_res_);
    /* keyframe */
    this->declare_parameter("keyframe.keyframe_threshold", keyframe_dist_thr_);
    this->declare_parameter("keyframe.num_submap_keyframes",
                            mm_config.num_submap_keyframes_);
    /* match */
    this->declare_parameter("match.scancontext_max_correspondence_distance",
                            mm_config.scancontext_max_correspondence_distance_);
    this->declare_parameter("match.quatro_nano_gicp_voxel_resolution",
                            mm_config.voxel_res_);
    /* nano */
    this->declare_parameter("nano_gicp.thread_number", gc.nano_thread_number_);
    this->declare_parameter("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
    this->declare_parameter("nano_gicp.correspondences_number",
                            gc.nano_correspondences_number_);
    this->declare_parameter("nano_gicp.max_correspondence_distance",
                            gc.max_corr_dist_);
    this->declare_parameter("nano_gicp.max_iter", gc.nano_max_iter_);
    this->declare_parameter("nano_gicp.transformation_epsilon",
                            gc.transformation_epsilon_);
    this->declare_parameter("nano_gicp.euclidean_fitness_epsilon",
                            gc.euclidean_fitness_epsilon_);
    this->declare_parameter("nano_gicp.ransac.max_iter",
                            gc.nano_ransac_max_iter_);
    this->declare_parameter("nano_gicp.ransac.outlier_rejection_threshold",
                            gc.ransac_outlier_rejection_threshold_);
    /* quatro */
    this->declare_parameter("quatro.enable", mm_config.enable_quatro_);
    this->declare_parameter("quatro.optimize_matching",
                            qc.use_optimized_matching_);
    this->declare_parameter("quatro.distance_threshold",
                            qc.quatro_distance_threshold_);
    this->declare_parameter("quatro.max_correspondences",
                            qc.quatro_max_num_corres_);
    this->declare_parameter("quatro.fpfh_normal_radius",
                            qc.fpfh_normal_radius_);
    this->declare_parameter("quatro.fpfh_radius", qc.fpfh_radius_);
    this->declare_parameter("quatro.estimating_scale", qc.estimat_scale_);
    this->declare_parameter("quatro.noise_bound", qc.noise_bound_);
    this->declare_parameter("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
    this->declare_parameter("quatro.rotation.rot_cost_diff_threshold",
                            qc.rot_cost_diff_thr_);
    this->declare_parameter("quatro.rotation.num_max_iter",
                            qc.quatro_max_iter_);

    GET_PARAM_DEBUG("basic.map_frame", map_frame_);
    GET_PARAM_DEBUG("basic.saved_map", saved_map_path);
    GET_PARAM_DEBUG("basic.saved_map_pose_topic", pose_topic);
    GET_PARAM_DEBUG("basic.saved_map_pcd_topic", pcd_topic);
    GET_PARAM_DEBUG("basic.map_match_hz", map_match_hz);
    GET_PARAM_DEBUG("basic.visualize_voxel_size", voxel_res_);
    /* keyframe */
    GET_PARAM_DEBUG("keyframe.keyframe_threshold", keyframe_dist_thr_);
    GET_PARAM_DEBUG("keyframe.num_submap_keyframes",
                    mm_config.num_submap_keyframes_);
    /* match */
    GET_PARAM_DEBUG("match.scancontext_max_correspondence_distance",
                    mm_config.scancontext_max_correspondence_distance_);
    GET_PARAM_DEBUG("match.quatro_nano_gicp_voxel_resolution",
                    mm_config.voxel_res_);
    /* nano */
    GET_PARAM_DEBUG("nano_gicp.thread_number", gc.nano_thread_number_);
    GET_PARAM_DEBUG("nano_gicp.icp_score_threshold", gc.icp_score_thr_);
    GET_PARAM_DEBUG("nano_gicp.correspondences_number",
                    gc.nano_correspondences_number_);
    GET_PARAM_DEBUG("nano_gicp.max_correspondence_distance", gc.max_corr_dist_);
    GET_PARAM_DEBUG("nano_gicp.max_iter", gc.nano_max_iter_);
    GET_PARAM_DEBUG("nano_gicp.transformation_epsilon",
                    gc.transformation_epsilon_);
    GET_PARAM_DEBUG("nano_gicp.euclidean_fitness_epsilon",
                    gc.euclidean_fitness_epsilon_);
    GET_PARAM_DEBUG("nano_gicp.ransac.max_iter", gc.nano_ransac_max_iter_);
    GET_PARAM_DEBUG("nano_gicp.ransac.outlier_rejection_threshold",
                    gc.ransac_outlier_rejection_threshold_);
    /* quatro */
    GET_PARAM_DEBUG("quatro.enable", mm_config.enable_quatro_);
    GET_PARAM_DEBUG("quatro.optimize_matching", qc.use_optimized_matching_);
    GET_PARAM_DEBUG("quatro.distance_threshold", qc.quatro_distance_threshold_);
    GET_PARAM_DEBUG("quatro.max_correspondences", qc.quatro_max_num_corres_);
    GET_PARAM_DEBUG("quatro.fpfh_normal_radius", qc.fpfh_normal_radius_);
    GET_PARAM_DEBUG("quatro.fpfh_radius", qc.fpfh_radius_);
    GET_PARAM_DEBUG("quatro.estimating_scale", qc.estimat_scale_);
    GET_PARAM_DEBUG("quatro.noise_bound", qc.noise_bound_);
    GET_PARAM_DEBUG("quatro.rotation.gnc_factor", qc.rot_gnc_factor_);
    GET_PARAM_DEBUG("quatro.rotation.rot_cost_diff_threshold",
                    qc.rot_cost_diff_thr_);
    GET_PARAM_DEBUG("quatro.rotation.num_max_iter", qc.quatro_max_iter_);

    map_matcher_ = std::make_shared<MapMatcher>(mm_config);

    loadMap(saved_map_path, pose_topic, pcd_topic);

    RCLCPP_INFO(this->get_logger(), "Map Loaded Success.");

    raw_odom_path_.header.frame_id = map_frame_;
    corrected_odom_path_.header.frame_id = map_frame_;

    odom_pub_ = this->create_publisher<PointCloudT>("/ori_odom", 10);
    path_pub_ = this->create_publisher<PathT>("/ori_path", 10);
    corrected_odom_pub_ =
        this->create_publisher<PointCloudT>("/corrected_odom", 10);
    corrected_path_pub_ = this->create_publisher<PathT>("/corrected_path", 10);
    corrected_current_pcd_pub_ =
        this->create_publisher<PointCloudT>("/corrected_current_pcd", 10);
    map_match_pub_ = this->create_publisher<MarkerT>("/map_match", 10);
    realtime_pose_pub_ =
        this->create_publisher<PoseStampedT>("/pose_stamped", 10);
    saved_map_pub_ = this->create_publisher<PointCloudT>("/saved_map", 10);
    debug_src_pub_ = this->create_publisher<PointCloudT>("/src", 10);
    debug_dst_pub_ = this->create_publisher<PointCloudT>("/dst", 10);
    debug_coarse_aligned_pub_ =
        this->create_publisher<PointCloudT>("/coarse_aligned_quatro", 10);
    debug_fine_aligned_pub_ =
        this->create_publisher<PointCloudT>("/fine_aligned_nano_gicp", 10);

    rmw_qos_profile_t profile_ = rclcpp::QoS(10).get_rmw_qos_profile();
    sub_odom_ = std::make_shared<message_filters::Subscriber<OdomT>>(
        this, "/Odometry", profile_);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<PointCloudT>>(
        this, "/cloud_registered", profile_);
    sub_odom_pcd_sync_ =
        std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(
            odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(
        std::bind(&FastLioLocalizationScQn::odomPcdCallback, this,
                  std::placeholders::_1, std::placeholders::_2));

    match_timer_ = rclcpp::create_timer(
        this, this->get_clock(),
        rclcpp::Duration(std::chrono::duration<double>(1 / map_match_hz)),
        std::bind(&FastLioLocalizationScQn::matchingTimerFunc, this));

    RCLCPP_WARN(this->get_logger(), "Main class, starting node...");
  }

  ~FastLioLocalizationScQn(){};

private:
  // methods
  void updateOdomsAndPaths(const PosePcd &pose_pcd_in) {
    raw_odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                                   pose_pcd_in.pose_eig_(1, 3),
                                   pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    raw_odom_path_.poses.emplace_back(
        poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_odom_path_.poses.emplace_back(
        poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
  }

  bool checkIfKeyframe(const PosePcd &pose_pcd_in,
                       const PosePcd &latest_pose_pcd) {
    return keyframe_dist_thr_ <
           (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) -
            pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3))
               .norm();
  }

  visualization_msgs::msg::Marker
  getMatchMarker(const std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>>
                     &match_xyz_pairs) {
    MarkerT edges_;
    edges_.type = 5u;
    edges_.scale.x = 0.2f;
    edges_.header.frame_id = map_frame_;
    edges_.pose.orientation.w = 1.0f;
    edges_.color.r = 1.0f;
    edges_.color.g = 1.0f;
    edges_.color.b = 1.0f;
    edges_.color.a = 1.0f;
    for (size_t i = 0; i < match_xyz_pairs.size(); ++i) {
      geometry_msgs::msg::Point p_, p2_;
      p_.x = match_xyz_pairs[i].first.x;
      p_.y = match_xyz_pairs[i].first.y;
      p_.z = match_xyz_pairs[i].first.z;
      p2_.x = match_xyz_pairs[i].second.x;
      p2_.y = match_xyz_pairs[i].second.y;
      p2_.z = match_xyz_pairs[i].second.z;
      edges_.points.push_back(p_);
      edges_.points.push_back(p2_);
    }
    return edges_;
  }

  void loadMap(const std::string &saved_map_path, const std::string &pose_topic,
               const std::string &pcd_topic) {

    std::vector<PointCloudT> load_pcd_vec;
    PointCloudT pcd_msg;
    std::vector<PoseStampedT> load_pose_vec;
    PoseStampedT pose_msg;

    {
      rosbag2_cpp::readers::SequentialReader reader;

      rosbag2_storage::StorageOptions storage_options{};
      storage_options.uri = saved_map_path;
      storage_options.storage_id = "sqlite3";
      rosbag2_cpp::ConverterOptions converter_options{};
      converter_options.input_serialization_format = "cdr";
      converter_options.output_serialization_format = "cdr";
      reader.open(storage_options, converter_options);
      rosbag2_cpp::SerializationFormatConverterFactory factory_;

      std::unique_ptr<
          rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
          cdr_des_ = factory_.load_deserializer("cdr");

      auto topics = reader.get_all_topics_and_types();

      // for (auto t : topics) {
      //   fmt::print("meta name:{}, meta type:{}\n", t.name, t.type);
      // }

      auto ros_message =
          std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
      ros_message->time_stamp = 0;
      ros_message->message = nullptr;
      ros_message->allocator = rcutils_get_default_allocator();

      std::shared_ptr<rcpputils::SharedLibrary> library;
      const rosidl_message_type_support_t *string_typesupport;

      RCLCPP_WARN(this->get_logger(), "BAG START READING...");

      while (reader.has_next()) {
        auto serialized_message = reader.read_next();

        if (serialized_message->topic_name == pcd_topic) {
          ros_message->message = &pcd_msg;
          library = rosbag2_cpp::get_typesupport_library(
              "sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp");
          string_typesupport = rosbag2_cpp::get_typesupport_handle(
              "sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp", library);
          cdr_des_->deserialize(serialized_message, string_typesupport,
                                ros_message);
          load_pcd_vec.push_back(pcd_msg);
        } else if (serialized_message->topic_name == pose_topic) {
          ros_message->message = &pose_msg;
          library = rosbag2_cpp::get_typesupport_library(
              "geometry_msgs/msg/PoseStamped", "rosidl_typesupport_cpp");
          string_typesupport = rosbag2_cpp::get_typesupport_handle(
              "geometry_msgs/msg/PoseStamped", "rosidl_typesupport_cpp",
              library);
          cdr_des_->deserialize(serialized_message, string_typesupport,
                                ros_message);
          load_pose_vec.push_back(pose_msg);
        }
      }

      if (load_pcd_vec.size() != load_pose_vec.size()) {
        RCLCPP_ERROR(this->get_logger(), "WRONG BAG FILE!!!!!");
      } else {
        RCLCPP_WARN(this->get_logger(), "BAG FILE READED.");
      }

      reader.close();
    }

    for (size_t i = 0; i < load_pose_vec.size(); ++i) {
      saved_map_from_bag_.push_back(
          PosePcdReduced(load_pose_vec[i], load_pcd_vec[i], i));
      saved_map_pcd_ += transformPcd(saved_map_from_bag_[i].pcd_,
                                     saved_map_from_bag_[i].pose_eig_);
      map_matcher_->updateScancontext(
          saved_map_from_bag_[i]
              .pcd_); // note: update scan context for loop candidate detection
    }
    saved_map_pcd_ = *voxelizePcd(saved_map_pcd_, voxel_res_);

    return;
  }

  // cb
  void odomPcdCallback(
      const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pcd_msg) {
    PosePcd current_frame =
        PosePcd(*odom_msg, *pcd_msg,
                current_keyframe_idx_); // to be checked if keyframe or not
    //// 1. realtime pose = last TF * odom
    current_frame.pose_corrected_eig_ =
        last_corrected_TF_ * current_frame.pose_eig_;
    PoseStampedT current_pose_stamped_ =
        poseEigToPoseStamped(current_frame.pose_corrected_eig_, map_frame_);
    realtime_pose_pub_->publish(current_pose_stamped_);

    geometry_msgs::msg::TransformStamped trans_stamped_msg_;
    trans_stamped_msg_.transform =
        tf2::toMsg(poseEigToROSTf(current_frame.pose_corrected_eig_));
    trans_stamped_msg_.header.frame_id = map_frame_;
    trans_stamped_msg_.child_frame_id = "robot";
    trans_stamped_msg_.header.stamp = rclcpp::Clock().now();
    broadcaster_.sendTransform(trans_stamped_msg_);
    // pub current scan in corrected pose frame
    corrected_current_pcd_pub_->publish(pclToPclRos(
        transformPcd(current_frame.pcd_, current_frame.pose_corrected_eig_),
        map_frame_));

    if (!is_initialized_) //// init only once
    {
      // 1. save first keyframe
      {
        std::lock_guard<std::mutex> lock(keyframes_mutex_);
        last_keyframe_ = current_frame;
      }
      current_keyframe_idx_++;
      //// 2. vis
      {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        updateOdomsAndPaths(current_frame);
      }
      is_initialized_ = true;
    } else {
      //// 1. check if keyframe
      if (checkIfKeyframe(current_frame, last_keyframe_)) {
        // 2. if so, save
        {
          std::lock_guard<std::mutex> lock(keyframes_mutex_);
          last_keyframe_ = current_frame;
        }
        current_keyframe_idx_++;
        //// 3. vis
        {
          std::lock_guard<std::mutex> lock(vis_mutex_);
          updateOdomsAndPaths(current_frame);
        }
      }
    }
    return;
  }

  void matchingTimerFunc() {
    if (!is_initialized_) {
      return;
    }

    //// 1. copy not processed keyframes
    high_resolution_clock::time_point t1_ = high_resolution_clock::now();
    PosePcd last_keyframe_copy;
    {
      std::lock_guard<std::mutex> lock(keyframes_mutex_);
      last_keyframe_copy = last_keyframe_;
      last_keyframe_.processed_ = true;
    }
    if (last_keyframe_copy.idx_ == 0 || last_keyframe_copy.processed_) {
      return; // already processed or initial keyframe
    }

    //// 2. detect match and calculate TF
    // from last_keyframe_copy keyframe to map (saved keyframes) in threshold
    // radius, get the closest keyframe
    int closest_keyframe_idx = map_matcher_->fetchClosestKeyframeIdx(
        last_keyframe_copy, saved_map_from_bag_);
    if (closest_keyframe_idx < 0) {
      return; // if no matched candidate
    }
    // Quatro + NANO-GICP to check match (from current_keyframe to closest
    // keyframe in saved map)
    const RegistrationOutput &reg_output = map_matcher_->performMapMatcher(
        last_keyframe_copy, saved_map_from_bag_, closest_keyframe_idx);

    //// 3. handle corrected results
    if (reg_output.is_valid_) // TF the pose with the result of match
    {
      RCLCPP_INFO(this->get_logger(),
                  "\033[1;32mMap matching accepted. Score: %.3f\033[0m",
                  reg_output.score_);
      last_corrected_TF_ =
          reg_output.pose_between_eig_ * last_corrected_TF_; // update TF
      Eigen::Matrix4d TFed_pose =
          reg_output.pose_between_eig_ * last_keyframe_copy.pose_corrected_eig_;
      // correct poses in vis data
      {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        corrected_odoms_.points[last_keyframe_copy.idx_] =
            pcl::PointXYZ(TFed_pose(0, 3), TFed_pose(1, 3), TFed_pose(2, 3));
        corrected_odom_path_.poses[last_keyframe_copy.idx_] =
            poseEigToPoseStamped(TFed_pose, map_frame_);
      }
      // map matches
      matched_pairs_xyz_.push_back(
          {corrected_odoms_.points[last_keyframe_copy.idx_],
           raw_odoms_.points[last_keyframe_copy.idx_]}); // for vis
      map_match_pub_->publish(getMatchMarker(matched_pairs_xyz_));
    }
    high_resolution_clock::time_point t2_ = high_resolution_clock::now();

    debug_src_pub_->publish(
        pclToPclRos(map_matcher_->getSourceCloud(), map_frame_));
    debug_dst_pub_->publish(
        pclToPclRos(map_matcher_->getTargetCloud(), map_frame_));
    debug_coarse_aligned_pub_->publish(
        pclToPclRos(map_matcher_->getCoarseAlignedCloud(), map_frame_));
    debug_fine_aligned_pub_->publish(
        pclToPclRos(map_matcher_->getFinalAlignedCloud(), map_frame_));

    // publish odoms and paths
    {
      std::lock_guard<std::mutex> lock(vis_mutex_);
      corrected_odom_pub_->publish(pclToPclRos(corrected_odoms_, map_frame_));
      corrected_path_pub_->publish(corrected_odom_path_);
    }
    odom_pub_->publish(pclToPclRos(raw_odoms_, map_frame_));
    path_pub_->publish(raw_odom_path_);
    // publish saved map
    if (saved_map_vis_switch_ && saved_map_pub_->get_subscription_count() > 0) {
      saved_map_pub_->publish(pclToPclRos(saved_map_pcd_, map_frame_));
      saved_map_vis_switch_ = false;
    }
    if (!saved_map_vis_switch_ &&
        saved_map_pub_->get_subscription_count() == 0) {
      saved_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point t3_ = high_resolution_clock::now();
    RCLCPP_INFO(this->get_logger(), "Matching: %.1fms, vis: %.1fms",
                duration_cast<microseconds>(t2_ - t1_).count() / 1e3,
                duration_cast<microseconds>(t3_ - t2_).count() / 1e3);
    return;
  } // timer callback
};
