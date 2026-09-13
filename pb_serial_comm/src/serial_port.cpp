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

#include "pb_serial_comm/serial_port.hpp"

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <sys/select.h>
#include <sys/time.h>

namespace pb_serial_comm
{

namespace asio = boost::asio;

SerialPort::SerialPort()
: io_(std::make_unique<asio::io_service>()),
  port_(std::make_unique<asio::serial_port>(*io_))
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, uint32_t baud_rate)
{
  try {
    port_->open(device);
    port_->set_option(asio::serial_port_base::baud_rate(baud_rate));
    port_->set_option(asio::serial_port_base::character_size(8));
    port_->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));
    port_->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    port_->set_option(asio::serial_port_base::flow_control(
        asio::serial_port_base::flow_control::none));
    return true;
  } catch (const boost::system::system_error &) {
    close();
    return false;
  }
}

void SerialPort::close()
{
  boost::system::error_code ec;
  if (port_->is_open()) {
    port_->cancel(ec);
    port_->close(ec);
  }
}

bool SerialPort::isOpen() const
{
  return port_->is_open();
}

size_t SerialPort::write(const uint8_t * data, size_t len)
{
  if (!port_->is_open() || data == nullptr || len == 0) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(write_mutex_);
  boost::system::error_code ec;
  const size_t n = asio::write(*port_, asio::buffer(data, len), ec);
  return ec ? 0 : n;
}

size_t SerialPort::read(uint8_t * data, size_t len, int timeout_ms)
{
  if (!port_->is_open() || data == nullptr || len == 0) {
    return 0;
  }

  // 用 select 判断可读（带超时），再 read_some 读取，避免阻塞。
  const int fd = port_->native_handle();
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(fd, &readfds);
  struct timeval tv;
  tv.tv_sec = timeout_ms / 1000;
  tv.tv_usec = (timeout_ms % 1000) * 1000;

  const int ret = select(fd + 1, &readfds, nullptr, nullptr, &tv);
  if (ret <= 0) {
    return 0;  // 超时或出错
  }

  boost::system::error_code ec;
  const size_t n = port_->read_some(asio::buffer(data, len), ec);
  return ec ? 0 : n;
}

bool SerialPort::readByte(uint8_t & byte, int timeout_ms)
{
  return read(&byte, 1, timeout_ms) == 1;
}

}  // namespace pb_serial_comm
