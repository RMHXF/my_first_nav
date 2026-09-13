// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ODIN_INTERFACE__ODIN_INTERFACE_HPP_
#define ODIN_INTERFACE__ODIN_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace odin_interface
{

class OdinInterfaceNode : public rclcpp::Node
{
public:
  explicit OdinInterfaceNode(const rclcpp::NodeOptions & options);

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void mapOdomTimerCallback();

  // Resolve the imu -> lidar extrinsics: prefer Odin's published TF (stage 1),
  // fall back to the `imu_to_lidar` parameter (stage 2, when Odin TF is off).
  tf2::Transform getImuToLidar(const rclcpp::Time & stamp);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  rclcpp::TimerBase::SharedPtr map_odom_timer_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string input_odom_topic_;
  std::string input_cloud_topic_;
  std::string odom_frame_;
  std::string imu_frame_;
  std::string lidar_frame_;
  std::string map_frame_;
  bool publish_map_odom_tf_;
  double map_odom_tf_rate_;

  tf2::Transform tf_imu_to_lidar_param_;
  bool imu_to_lidar_cached_;
  tf2::Transform tf_imu_to_lidar_;

  bool odom_to_lidar_initialized_;
  tf2::Transform tf_odom_to_lidar_;
};

}  // namespace odin_interface

#endif  // ODIN_INTERFACE__ODIN_INTERFACE_HPP_
