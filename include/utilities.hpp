#pragma once

#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2/transform_storage.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>

#include <fmt/chrono.h>
#include <fmt/core.h>

using PointType = pcl::PointXYZI;

double inline toSec(const rclcpp::Time &timestamp) {
  return static_cast<double>(timestamp.seconds()) +
         1e-9 * static_cast<double>(timestamp.nanoseconds());
};

double inline toSec(const builtin_interfaces::msg::Time &timestamp) {
  return static_cast<double>(timestamp.sec) +
         1e-9 * static_cast<double>(timestamp.nanosec);
};

rclcpp::Time inline fromSec(const double t) {
  auto sec = (uint32_t)floor(t);
  return rclcpp::Time(sec, (uint32_t)std::round((t - sec) * 1e9));
}

inline void matrixEigenToTF(const Eigen::Matrix3d &e, tf2::Matrix3x3 &t) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      t[i][j] = e(i, j);
}

inline void matrixTFToEigen(const tf2::Matrix3x3 &t, Eigen::Matrix3d &e) {
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      e(i, j) = t[i][j];
}

inline geometry_msgs::msg::PoseStamped
poseEigToPoseStamped(const Eigen::Matrix4d &pose_eig_in,
                     const std::string &frame_id = "map") {
  double r, p, y;
  tf2::Matrix3x3 mat;
  matrixEigenToTF(pose_eig_in.block<3, 3>(0, 0), mat);
  mat.getRPY(r, p, y);
  tf2::Quaternion quat;
  quat.setRPY(r, p, y);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = pose_eig_in(0, 3);
  pose.pose.position.y = pose_eig_in(1, 3);
  pose.pose.position.z = pose_eig_in(2, 3);
  pose.pose.orientation.w = quat.getW();
  pose.pose.orientation.x = quat.getX();
  pose.pose.orientation.y = quat.getY();
  pose.pose.orientation.z = quat.getZ();
  return pose;
}

inline tf2::Transform poseEigToROSTf(const Eigen::Matrix4d &pose) {
  Eigen::Quaterniond quat(pose.block<3, 3>(0, 0));
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
  transform.setRotation(
      tf2::Quaternion(quat.x(), quat.y(), quat.z(), quat.w()));
  return transform;
}

template <typename T>
inline sensor_msgs::msg::PointCloud2
pclToPclRos(const pcl::PointCloud<T> &cloud, std::string frame_id = "map") {
  sensor_msgs::msg::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

template <typename T>
inline pcl::PointCloud<T> transformPcd(const pcl::PointCloud<T> &cloud_in,
                                       const Eigen::Matrix4d &pose_tf) {
  if (cloud_in.size() == 0) {
    return cloud_in;
  }
  pcl::PointCloud<T> pcl_out = cloud_in;
  pcl::transformPointCloud(cloud_in, pcl_out, pose_tf);
  return pcl_out;
}

inline pcl::PointCloud<PointType>::Ptr
voxelizePcd(const pcl::PointCloud<PointType> &pcd_in, const float voxel_res) {
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_in_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  pcd_in_ptr->reserve(pcd_in.size());
  pcd_out->reserve(pcd_in.size());
  *pcd_in_ptr = pcd_in;
  voxelgrid.setInputCloud(pcd_in_ptr);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}

inline pcl::PointCloud<PointType>::Ptr
voxelizePcd(const pcl::PointCloud<PointType>::Ptr &pcd_in,
            const float voxel_res) {
  static pcl::VoxelGrid<PointType> voxelgrid;
  voxelgrid.setLeafSize(voxel_res, voxel_res, voxel_res);
  pcl::PointCloud<PointType>::Ptr pcd_out(new pcl::PointCloud<PointType>);
  pcd_out->reserve(pcd_in->size());
  voxelgrid.setInputCloud(pcd_in);
  voxelgrid.filter(*pcd_out);
  return pcd_out;
}