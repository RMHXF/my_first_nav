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

#ifndef PB_SERIAL_COMM__FRAME_HPP_
#define PB_SERIAL_COMM__FRAME_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pb_serial_comm
{

// ============================== 帧格式 ==============================
// | SOF(1B) | CMD_ID(1B) | LEN(1B) | DATA(LEN B) | CRC16(2B, 低字节在前) | EOF(1B) |
//   SOF = 0xA5, EOF = 0x5A
//   LEN = DATA 部分的字节数（0~255）
//   CRC16 = MODBUS 多项式，对 [CMD_ID, LEN, DATA...] 计算
// =====================================================================

constexpr uint8_t FRAME_SOF = 0xA5;
constexpr uint8_t FRAME_EOF = 0x5A;
constexpr uint8_t FRAME_MAX_DATA_LEN = 255;
// 除 DATA 外的固定开销：SOF + CMD_ID + LEN + CRC16(2) + EOF
constexpr size_t FRAME_OVERHEAD = 6;

// ============================== 命令ID ==============================
// 上位机 -> 下位机（发送）
constexpr uint8_t CMD_CHASSIS_VEL = 0x01;  // 底盘速度
constexpr uint8_t CMD_GIMBAL = 0x02;       // 云台
constexpr uint8_t CMD_SHOOT = 0x03;        // 射击

// 下位机 -> 上位机（接收）
constexpr uint8_t CMD_CHASSIS_STATE = 0x10;  // 底盘状态（里程计）
constexpr uint8_t CMD_GIMBAL_STATE = 0x11;   // 云台状态
constexpr uint8_t CMD_REFEREE_DATA = 0x12;   // 裁判系统数据

// 各命令的数据负载长度（字节）
constexpr size_t CHASSIS_VEL_LEN = 12;   // vx, vy, wz (3 * float32)
constexpr size_t GIMBAL_LEN = 8;         // pitch, yaw (2 * float32)
constexpr size_t SHOOT_LEN = 1;          // 1 * uint8
constexpr size_t CHASSIS_STATE_LEN = 24; // x, y, yaw, vx, vy, wz (6 * float32)
constexpr size_t GIMBAL_STATE_LEN = 8;   // pitch, yaw (2 * float32)

// MODBUS CRC16（多项式 0x8005 反射 0xA001，初值 0xFFFF）
uint16_t crc16(const uint8_t * data, size_t len);

// 打包一帧：cmd_id + data -> 完整帧（含 SOF / CRC / EOF）
std::vector<uint8_t> packFrame(uint8_t cmd_id, const uint8_t * data, size_t len);

// 从字节流中解析帧。输入为一段可能含噪声/半帧的字节流，
// 输出完整、校验通过的帧（cmd_id + data）。返回值：成功解析的帧数量。
// 丢弃无法解析的字节（含帧头错位、CRC 错误、长度越界）。
size_t parseFrames(
  const std::vector<uint8_t> & buffer,
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & frames);

}  // namespace pb_serial_comm

#endif  // PB_SERIAL_COMM__FRAME_HPP_
