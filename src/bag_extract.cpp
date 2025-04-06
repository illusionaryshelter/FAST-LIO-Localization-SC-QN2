#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/typesupport_helpers.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/logging.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>

#include <fmt/chrono.h>
#include <fmt/core.h>

#include <gflags/gflags.h>
#include <gflags/gflags_gflags.h>

DEFINE_string(src, "", "src bag name.");
DEFINE_string(pose_in, "", "input posestamped topic name.");
DEFINE_string(pcd_in, "", "input pointcloud2 topic name.");
DEFINE_string(imu_in, "", "input imu topic name.");
DEFINE_string(dst, "", "dst bag name.");
DEFINE_string(pose_out, "/keyframe_pose", "output posestamped topic name.");
DEFINE_string(pcd_out, "/keyframe_pcd", "output pointcloud2 topic name.");
DEFINE_uint32(thre, 0, "reserve.");
int main(int argc, char **argv) {

  google::ParseCommandLineFlags(&argc, &argv, true);

  google::SetUsageMessage(
      "Use to extract the PointCloud2 and PoseStamped data from any ros2 "
      "bag(sqlite&cdr) and generate a new bag(see --help). \n The custom type "
      "like livox is developing.");

  if ((FLAGS_src.empty()) || (FLAGS_pcd_in.empty()) || FLAGS_pose_in.empty() ||
      FLAGS_dst.empty()) {
    std::cerr << "Some args is empty, exit.\n";
    exit(-1);
  }

  if (FLAGS_thre == 0) {
    FLAGS_thre = UINT32_MAX;
  }

  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_cpp::writers::SequentialWriter writer;

  rosbag2_storage::StorageOptions read_storage_options{};
  read_storage_options.uri = FLAGS_src;
  read_storage_options.storage_id = "sqlite3";

  rosbag2_storage::StorageOptions write_storage_options{};
  write_storage_options.uri = FLAGS_dst;
  write_storage_options.storage_id = "sqlite3";

  rosbag2_cpp::ConverterOptions converter_options{};
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  writer.open(write_storage_options, converter_options);
  reader.open(read_storage_options, converter_options);

  writer.create_topic(
      {FLAGS_pcd_out, "sensor_msgs/msg/PointCloud2", "cdr", ""});
  writer.create_topic(
      {FLAGS_pose_out, "geometry_msgs/msg/PoseStamped", "cdr", ""});
  writer.create_topic({"/livox/imu", "sensor_msgs/msg/Imu", "cdr", ""});

  rosbag2_cpp::SerializationFormatConverterFactory factory_;

  std::unique_ptr<
      rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
      cdr_des_ = factory_.load_deserializer("cdr");

  std::unique_ptr<
      rosbag2_cpp::converter_interfaces::SerializationFormatSerializer>
      cdr_s_ = factory_.load_serializer("cdr");

  auto topics = reader.get_all_topics_and_types();

  for (auto t : topics) {
    fmt::print("meta name:{}, meta type:{}\n", t.name, t.type);
  }

  sensor_msgs::msg::PointCloud2 pcd_msg;
  geometry_msgs::msg::PoseStamped pose_msg;
  sensor_msgs::msg::Imu imu_msg;

  auto des_message =
      std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
  des_message->time_stamp = 0;
  des_message->message = nullptr;
  des_message->allocator = rcutils_get_default_allocator();

  auto s_message =
      std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
  des_message->time_stamp = 0;
  des_message->message = nullptr;
  des_message->allocator = rcutils_get_default_allocator();

  std::shared_ptr<rcpputils::SharedLibrary> library;
  const rosidl_message_type_support_t *string_typesupport;

  auto reader_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();

  size_t i_pcd = 0, i_pose = 0, i_imu = 0;

  while (reader.has_next()) {
    reader_msg = reader.read_next();

    if (reader_msg->topic_name == FLAGS_pcd_in) {
      des_message->message = &pcd_msg;
      library = rosbag2_cpp::get_typesupport_library(
          "sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp");
      string_typesupport = rosbag2_cpp::get_typesupport_handle(
          "sensor_msgs/msg/PointCloud2", "rosidl_typesupport_cpp", library);
      cdr_des_->deserialize(reader_msg, string_typesupport, des_message);
      reader_msg->topic_name = FLAGS_pcd_out;
      writer.write(reader_msg);
      i_pcd++;

    } else if (reader_msg->topic_name == FLAGS_pose_in) {
      des_message->message = &pose_msg;
      library = rosbag2_cpp::get_typesupport_library(
          "geometry_msgs/msg/PoseStamped", "rosidl_typesupport_cpp");
      string_typesupport = rosbag2_cpp::get_typesupport_handle(
          "geometry_msgs/msg/PoseStamped", "rosidl_typesupport_cpp", library);
      cdr_des_->deserialize(reader_msg, string_typesupport, des_message);
      reader_msg->topic_name = FLAGS_pose_out;
      writer.write(reader_msg);
      i_pose++;
    } else if (reader_msg->topic_name == FLAGS_imu_in) {
      des_message->message = &imu_msg;
      library = rosbag2_cpp::get_typesupport_library("sensor_msgs/msg/Imu",
                                                     "rosidl_typesupport_cpp");
      string_typesupport = rosbag2_cpp::get_typesupport_handle(
          "sensor_msgs/msg/Imu", "rosidl_typesupport_cpp", library);
      cdr_des_->deserialize(reader_msg, string_typesupport, des_message);
      reader_msg->topic_name = "/livox/imu";
      writer.write(reader_msg);
      i_imu++;
    }

    if (i_pose >= FLAGS_thre && i_pose == i_pcd && i_pose == i_imu) {
      break;
    }
  }
  reader.close();
  writer.close();
  return 0;
}
