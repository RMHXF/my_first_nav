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

#include "pb_serial_comm/serial_comm_node.hpp"

#include <chrono>
#include <cmath>
#include <cstring>

#include "pb_serial_comm/frame.hpp"

namespace pb_serial_comm
{

namespace
{

// 小端打包 float32 -> 4 字节（x86/ARM/STM32 均为小端）
void putFloat(float v, uint8_t * out)
{
  std::memcpy(out, &v, sizeof(float));
}

// 小端解包 4 字节 -> float32
float getFloat(const uint8_t * in)
{
  float v;
  std::memcpy(&v, in, sizeof(float));
  return v;
}

}  // namespace

SerialCommNode::SerialCommNode(const rclcpp::NodeOptions & options)
: Node("serial_comm", options)
{
  // 串口参数
  this->declare_parameter<std::string>("device", "/dev/ttyACM0");
  this->declare_parameter<int>("baud_rate", 115200);

  // 话题名
  this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
  this->declare_parameter<std::string>("gimbal_topic", "cmd_gimbal_joint");
  this->declare_parameter<std::string>("shoot_topic", "cmd_shoot");
  this->declare_parameter<std::string>("chassis_odom_topic", "chassis_odometry");
  this->declare_parameter<std::string>("gimbal_state_topic", "gimbal_state");
  this->declare_parameter<std::string>("referee_topic", "referee_data");
  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("base_frame", "base_footprint");

  this->get_parameter("device", device_);
  this->get_parameter("baud_rate", baud_rate_);
  this->get_parameter("cmd_vel_topic", cmd_vel_topic_);
  this->get_parameter("gimbal_topic", gimbal_topic_);
  this->get_parameter("shoot_topic", shoot_topic_);
  this->get_parameter("chassis_odom_topic", chassis_odom_topic_);
  this->get_parameter("gimbal_state_topic", gimbal_state_topic_);
  this->get_parameter("referee_topic", referee_topic_);
  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("base_frame", base_frame_);

  // 订阅（上位机 -> 下位机）
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_topic_, 10, std::bind(&SerialCommNode::cmdVelCallback, this, std::placeholders::_1));
  gimbal_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    gimbal_topic_, 10, std::bind(&SerialCommNode::gimbalCallback, this, std::placeholders::_1));
  shoot_sub_ = this->create_subscription<example_interfaces::msg::UInt8>(
    shoot_topic_, 10, std::bind(&SerialCommNode::shootCallback, this, std::placeholders::_1));

  // 发布（下位机 -> 上位机）
  chassis_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(chassis_odom_topic_, 10);
  gimbal_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(gimbal_state_topic_, 10);
  referee_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(referee_topic_, 10);

  // 启动接收线程
  running_ = true;
  recv_thread_ = std::thread(&SerialCommNode::receiveLoop, this);

  RCLCPP_INFO(this->get_logger(), "串口通信节点启动，设备 %s @ %d", device_.c_str(), baud_rate_);
}

SerialCommNode::~SerialCommNode()
{
  running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
  serial_.close();
}

void SerialCommNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  uint8_t data[CHASSIS_VEL_LEN];
  putFloat(static_cast<float>(msg->linear.x), data);
  putFloat(static_cast<float>(msg->linear.y), data + 4);
  putFloat(static_cast<float>(msg->angular.z), data + 8);

  const auto frame = packFrame(CMD_CHASSIS_VEL, data, CHASSIS_VEL_LEN);
  serial_.write(frame.data(), frame.size());
}

void SerialCommNode::gimbalCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 约定 position = [pitch, yaw]；若下位机顺序不同，请调整这里。
  if (msg->position.size() < 2) {
    return;
  }
  uint8_t data[GIMBAL_LEN];
  putFloat(static_cast<float>(msg->position[0]), data);  // pitch
  putFloat(static_cast<float>(msg->position[1]), data + 4);  // yaw

  const auto frame = packFrame(CMD_GIMBAL, data, GIMBAL_LEN);
  serial_.write(frame.data(), frame.size());
}

void SerialCommNode::shootCallback(const example_interfaces::msg::UInt8::SharedPtr msg)
{
  uint8_t data[SHOOT_LEN] = {msg->data};
  const auto frame = packFrame(CMD_SHOOT, data, SHOOT_LEN);
  serial_.write(frame.data(), frame.size());
}

void SerialCommNode::receiveLoop()
{
  // 串口可能晚于节点就绪才被系统枚举（USB 虚拟串口），失败则周期性重试。
  while (running_ && rclcpp::ok()) {
    if (!serial_.isOpen()) {
      if (serial_.open(device_, static_cast<uint32_t>(baud_rate_))) {
        RCLCPP_INFO(this->get_logger(), "串口打开成功：%s", device_.c_str());
      } else {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "串口打开失败：%s，2 秒后重试", device_.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }
    }

    uint8_t b;
    if (!serial_.readByte(b, 100)) {
      continue;  // 超时，继续等
    }
    if (b != FRAME_SOF) {
      continue;  // 非帧头，跳过
    }

    uint8_t cmd_id, len;
    if (!serial_.readByte(cmd_id, 50) || !serial_.readByte(len, 50)) {
      continue;
    }
    if (len > FRAME_MAX_DATA_LEN) {
      continue;
    }

    uint8_t data[FRAME_MAX_DATA_LEN];
    for (uint8_t i = 0; i < len; ++i) {
      if (!serial_.readByte(data[i], 50)) {
        len = 0;
        break;
      }
    }
    if (len == 0) {
      continue;
    }

    uint8_t crc_l, crc_h, eof;
    if (!serial_.readByte(crc_l, 50) || !serial_.readByte(crc_h, 50) ||
      !serial_.readByte(eof, 50))
    {
      continue;
    }
    if (eof != FRAME_EOF) {
      continue;
    }

    // 校验 CRC16（覆盖 cmd_id, len, data）
    std::vector<uint8_t> crc_input;
    crc_input.reserve(2 + len);
    crc_input.push_back(cmd_id);
    crc_input.push_back(len);
    crc_input.insert(crc_input.end(), data, data + len);
    const uint16_t calc_crc = crc16(crc_input.data(), crc_input.size());
    const uint16_t recv_crc = static_cast<uint16_t>(crc_l) | (static_cast<uint16_t>(crc_h) << 8);
    if (calc_crc != recv_crc) {
      continue;
    }

    std::vector<uint8_t> payload(data, data + len);
    handleFrame(cmd_id, payload);
  }
}

void SerialCommNode::handleFrame(uint8_t cmd_id, const std::vector<uint8_t> & data)
{
  switch (cmd_id) {
    case CMD_CHASSIS_STATE: {
      if (data.size() != CHASSIS_STATE_LEN) {
        RCLCPP_WARN(this->get_logger(), "底盘状态帧长度错误：%zu", data.size());
        break;
      }
      const float x = getFloat(data.data());
      const float y = getFloat(data.data() + 4);
      const float yaw = getFloat(data.data() + 8);
      const float vx = getFloat(data.data() + 12);
      const float vy = getFloat(data.data() + 16);
      const float wz = getFloat(data.data() + 20);

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->now();
      odom.header.frame_id = odom_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      const double half_yaw = yaw * 0.5;
      odom.pose.pose.orientation.z = std::sin(half_yaw);
      odom.pose.pose.orientation.w = std::cos(half_yaw);
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = wz;
      chassis_odom_pub_->publish(odom);
      break;
    }
    case CMD_GIMBAL_STATE: {
      if (data.size() != GIMBAL_STATE_LEN) {
        break;
      }
      sensor_msgs::msg::JointState state;
      state.header.stamp = this->now();
      state.name = {"gimbal_pitch", "gimbal_yaw"};
      state.position.resize(2);
      state.position[0] = getFloat(data.data());      // pitch
      state.position[1] = getFloat(data.data() + 4);  // yaw
      gimbal_state_pub_->publish(state);
      break;
    }
    case CMD_REFEREE_DATA: {
      std_msgs::msg::UInt8MultiArray msg;
      msg.data = data;
      referee_pub_->publish(msg);
      break;
    }
    default:
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "未知命令ID：0x%02X", cmd_id);
      break;
  }
}

}  // namespace pb_serial_comm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pb_serial_comm::SerialCommNode>());
  rclcpp::shutdown();
  return 0;
}
