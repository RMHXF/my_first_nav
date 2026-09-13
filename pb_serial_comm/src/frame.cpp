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

#include "pb_serial_comm/frame.hpp"

namespace pb_serial_comm
{

uint16_t crc16(const uint8_t * data, size_t len)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]);
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x0001) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

std::vector<uint8_t> packFrame(uint8_t cmd_id, const uint8_t * data, size_t len)
{
  std::vector<uint8_t> frame;
  frame.reserve(FRAME_OVERHEAD + len);
  frame.push_back(FRAME_SOF);
  frame.push_back(cmd_id);
  frame.push_back(static_cast<uint8_t>(len));
  if (len > 0 && data != nullptr) {
    frame.insert(frame.end(), data, data + len);
  }

  // CRC16 覆盖 [cmd_id, len, data...]
  std::vector<uint8_t> crc_input;
  crc_input.reserve(2 + len);
  crc_input.push_back(cmd_id);
  crc_input.push_back(static_cast<uint8_t>(len));
  if (len > 0 && data != nullptr) {
    crc_input.insert(crc_input.end(), data, data + len);
  }
  const uint16_t crc = crc16(crc_input.data(), crc_input.size());

  // 低字节在前（小端）
  frame.push_back(static_cast<uint8_t>(crc & 0xFF));
  frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
  frame.push_back(FRAME_EOF);
  return frame;
}

size_t parseFrames(
  const std::vector<uint8_t> & buffer,
  std::vector<std::pair<uint8_t, std::vector<uint8_t>>> & frames)
{
  size_t count = 0;
  size_t i = 0;

  while (i < buffer.size()) {
    // 找帧头 SOF
    if (buffer[i] != FRAME_SOF) {
      ++i;
      continue;
    }
    // 需要至少 FRAME_OVERHEAD 字节
    if (i + FRAME_OVERHEAD > buffer.size()) {
      break;  // 剩余字节不足一帧，等待更多数据
    }

    const uint8_t cmd_id = buffer[i + 1];
    const uint8_t len = buffer[i + 2];
    // 长度越界则丢弃这个 SOF，继续往后找
    if (static_cast<size_t>(len) > FRAME_MAX_DATA_LEN ||
      i + FRAME_OVERHEAD + len > buffer.size())
    {
      ++i;
      continue;
    }

    // 校验 CRC16（覆盖 cmd_id, len, data）
    std::vector<uint8_t> crc_input;
    crc_input.reserve(2 + len);
    crc_input.push_back(cmd_id);
    crc_input.push_back(len);
    crc_input.insert(crc_input.end(), buffer.begin() + i + 3, buffer.begin() + i + 3 + len);
    const uint16_t calc_crc = crc16(crc_input.data(), crc_input.size());
    const uint16_t recv_crc = static_cast<uint16_t>(buffer[i + 3 + len]) |
      (static_cast<uint16_t>(buffer[i + 3 + len + 1]) << 8);

    if (calc_crc != recv_crc || buffer[i + 3 + len + 2] != FRAME_EOF) {
      ++i;  // CRC 或帧尾错误，丢弃这个 SOF
      continue;
    }

    // 解析成功
    std::vector<uint8_t> data(buffer.begin() + i + 3, buffer.begin() + i + 3 + len);
    frames.emplace_back(cmd_id, std::move(data));
    ++count;
    i += FRAME_OVERHEAD + len;
  }

  return count;
}

}  // namespace pb_serial_comm
