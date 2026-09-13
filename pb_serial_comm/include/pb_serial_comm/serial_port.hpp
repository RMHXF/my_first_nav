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

#ifndef PB_SERIAL_COMM__SERIAL_PORT_HPP_
#define PB_SERIAL_COMM__SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <boost/asio/io_service.hpp>
#include <boost/asio/serial_port.hpp>

namespace pb_serial_comm
{

// 基于 boost::asio 的串口封装（Linux 虚拟串口 /dev/ttyACM*、/dev/ttyUSB*）。
// 写操作线程安全；读操作应在单一线程（接收线程）中调用。
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // 打开串口。device 例如 "/dev/ttyACM0"，baud_rate 例如 115200。
  bool open(const std::string & device, uint32_t baud_rate);
  void close();
  bool isOpen() const;

  // 阻塞写。返回实际写入字节数，失败返回 0。
  size_t write(const uint8_t * data, size_t len);

  // 带超时读取 len 字节。返回实际读取字节数；超时返回 0。
  size_t read(uint8_t * data, size_t len, int timeout_ms);

  // 带超时读取单个字节。成功返回 true，超时/出错返回 false。
  bool readByte(uint8_t & byte, int timeout_ms);

private:
  std::unique_ptr<boost::asio::io_service> io_;
  std::unique_ptr<boost::asio::serial_port> port_;
  std::mutex write_mutex_;  // 保护写操作（多个订阅回调可能并发写）
};

}  // namespace pb_serial_comm

#endif  // PB_SERIAL_COMM__SERIAL_PORT_HPP_
