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

#include "odin_interface/odin_interface.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/transforms.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace odin_interface
{

namespace
{

tf2::Transform makeTransformFromParams(const std::vector<double> & xyzrpy)
{
  tf2::Transform t;
  if (xyzrpy.size() != 6) {
    return t;  // identity
  }
  t.setOrigin(tf2::Vector3(xyzrpy[0], xyzrpy[1], xyzrpy[2]));
  tf2::Quaternion q;
  q.setRPY(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
  t.setRotation(q);
  return t;
}

}  // namespace

OdinInterfaceNode::OdinInterfaceNode(const rclcpp::NodeOptions & options)
: Node("odin_interface", options)
{
  this->declare_parameter<std::string>("input_odom_topic", "odin1/odometry");
  this->declare_parameter<std::string>("input_cloud_topic", "odin1/cloud_raw");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("imu_frame", "imu");
  this->declare_parameter<std::string>("lidar_frame", "lidar");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<bool>("publish_map_odom_tf", true);
  this->declare_parameter<double>("map_odom_tf_rate", 20.0);
  this->declare_parameter<std::vector<double>>("imu_to_lidar", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});

  this->get_parameter("input_odom_topic", input_odom_topic_);
  this->get_parameter("input_cloud_topic", input_cloud_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("imu_frame", imu_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("publish_map_odom_tf", publish_map_odom_tf_);
  this->get_parameter("map_odom_tf_rate", map_odom_tf_rate_);

  std::vector<double> imu_to_lidar;
  this->get_parameter("imu_to_lidar", imu_to_lidar);
  tf_imu_to_lidar_param_ = makeTransformFromParams(imu_to_lidar);
  imu_to_lidar_cached_ = false;
  odom_to_lidar_initialized_ = false;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("lidar_odometry", 5);
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("registered_scan", 5);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    input_odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&OdinInterfaceNode::odometryCallback, this, std::placeholders::_1));
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_cloud_topic_, rclcpp::SensorDataQoS(),
    std::bind(&OdinInterfaceNode::pointCloudCallback, this, std::placeholders::_1));

  if (publish_map_odom_tf_) {
    const auto period = std::chrono::duration<double>(1.0 / map_odom_tf_rate_);
    map_odom_timer_ = this->create_wall_timer(
      period, std::bind(&OdinInterfaceNode::mapOdomTimerCallback, this));
  }
}

tf2::Transform OdinInterfaceNode::getImuToLidar(const rclcpp::Time & stamp)
{
  if (imu_to_lidar_cached_) {
    return tf_imu_to_lidar_;
  }
  try {
    auto tf_stamped = tf_buffer_->lookupTransform(
      imu_frame_, lidar_frame_, stamp, rclcpp::Duration::from_seconds(0.1));
    tf2::fromMsg(tf_stamped.transform, tf_imu_to_lidar_);
    imu_to_lidar_cached_ = true;
    return tf_imu_to_lidar_;
  } catch (const tf2::TransformException &) {
    // Odin TF not (yet) available: fall back to the calibrated parameter.
    return tf_imu_to_lidar_param_;
  }
}

void OdinInterfaceNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  // Odin odometry: pose = odom -> imu.
  tf2::Transform tf_odom_to_imu;
  tf2::fromMsg(msg->pose.pose, tf_odom_to_imu);

  // Compose odom -> lidar = (odom -> imu) * (imu -> lidar).
  tf_odom_to_lidar_ = tf_odom_to_imu * getImuToLidar(msg->header.stamp);
  odom_to_lidar_initialized_ = true;

  nav_msgs::msg::Odometry out;
  out.header.stamp = msg->header.stamp;
  out.header.frame_id = odom_frame_;
  out.child_frame_id = lidar_frame_;

  const auto & origin = tf_odom_to_lidar_.getOrigin();
  out.pose.pose.position.x = origin.x();
  out.pose.pose.position.y = origin.y();
  out.pose.pose.position.z = origin.z();
  out.pose.pose.orientation = tf2::toMsg(tf_odom_to_lidar_.getRotation());

  odom_pub_->publish(out);
}

void OdinInterfaceNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!odom_to_lidar_initialized_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for Odin odometry before transforming the point cloud...");
    return;
  }

  // NOTE: Odin 的 cloud_raw 里 intensity 是 UINT8，pcl::fromROSMsg<PointXYZI> 无法把它转成
  // FLOAT32，会报 "Failed to find match for field 'intensity'" 并丢掉该字段，导致下游 terrain
  // 链失败。因此这里用 PointXYZ 读 xyz（不碰 intensity），变换后再手动补一个 FLOAT32 的
  // intensity 字段（值会被 terrain 覆盖，填 0 即可）。
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud_xyz);

  tf2::Transform tf_lidar_to_odom = tf_odom_to_lidar_.inverse();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_ros::transformPointCloud(*cloud_xyz, *cloud_transformed, tf_lidar_to_odom);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_out->resize(cloud_transformed->size());
  for (size_t i = 0; i < cloud_transformed->size(); ++i) {
    cloud_out->points[i].x = cloud_transformed->points[i].x;
    cloud_out->points[i].y = cloud_transformed->points[i].y;
    cloud_out->points[i].z = cloud_transformed->points[i].z;
    cloud_out->points[i].intensity = 0.0f;
  }

  auto out = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*cloud_out, *out);
  out->header.frame_id = odom_frame_;
  out->header.stamp = msg->header.stamp;  // 保留时间戳，否则下游同步器/TF 会失败
  cloud_pub_->publish(*out);
}

void OdinInterfaceNode::mapOdomTimerCallback()
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = this->now();
  tf.header.frame_id = map_frame_;
  tf.child_frame_id = odom_frame_;

  try {
    // 重定位模式（custom_map_mode=2）下 Odin 发布 odom->map，取逆得到 map->odom。
    auto tf_stamped = tf_buffer_->lookupTransform(odom_frame_, map_frame_, tf2::TimePointZero);
    tf2::Transform tf_odom_to_map;
    tf2::fromMsg(tf_stamped.transform, tf_odom_to_map);
    tf.transform = tf2::toMsg(tf_odom_to_map.inverse());
  } catch (const tf2::TransformException &) {
    // SLAM 模式（custom_map_mode=1）下 Odin 不发布 odom->map：其 "odom" 帧本身就是
    // 全局地图帧（README：地图原点 = 启动时 odom 原点）。此时 map 与 odom 重合，发布恒等。
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
  }

  tf_broadcaster_->sendTransform(tf);
}

}  // namespace odin_interface

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(odin_interface::OdinInterfaceNode)
