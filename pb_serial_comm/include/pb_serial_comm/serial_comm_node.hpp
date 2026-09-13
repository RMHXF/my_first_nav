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

#ifndef PB_SERIAL_COMM__SERIAL_COMM_NODE_HPP_
#define PB_SERIAL_COMM__SERIAL_COMM_NODE_HPP_

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <example_interfaces/msg/u_int8.hpp>

#include "pb_serial_comm/serial_port.hpp"

namespace pb_serial_comm
{

// 串口通信节点：把 ROS 话题打包成二进制帧下发到下位机，并接收下位机回传帧发布成话题。
class SerialCommNode : public rclcpp::Node
{
public:
  explicit SerialCommNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~SerialCommNode() override;

private:
  // 订阅回调（上位机 -> 下位机）
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void gimbalCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void shootCallback(const example_interfaces::msg::UInt8::SharedPtr msg);

  // 接收线程：循环读串口帧
  void receiveLoop();
  // 解析一帧并分发（下位机 -> 上位机）
  void handleFrame(uint8_t cmd_id, const std::vector<uint8_t> & data);

  // 串口参数
  std::string device_;
  int baud_rate_;

  // 话题名
  std::string cmd_vel_topic_;
  std::string gimbal_topic_;
  std::string shoot_topic_;
  std::string chassis_odom_topic_;
  std::string gimbal_state_topic_;
  std::string referee_topic_;
  std::string odom_frame_;
  std::string base_frame_;

  SerialPort serial_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr shoot_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr chassis_odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr referee_pub_;

  std::thread recv_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace pb_serial_comm

#endif  // PB_SERIAL_COMM__SERIAL_COMM_NODE_HPP_
