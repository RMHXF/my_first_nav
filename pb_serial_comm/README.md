# pb_serial_comm 串口通信功能包

上位机（ROS2）与下位机（STM32）通过 **USB 虚拟串口** 收发二进制帧。

## 1. 帧协议

```
| SOF(1B) | CMD_ID(1B) | LEN(1B) | DATA(LEN B) | CRC16(2B) | EOF(1B) |
```

- `SOF` = `0xA5`，`EOF` = `0x5A`
- `LEN` = DATA 部分字节数（0~255）
- `CRC16` = MODBUS 多项式（0xA001，初值 0xFFFF），对 `[CMD_ID, LEN, DATA...]` 计算，**低字节在前**
- 所有多字节数据（float32 等）均为**小端**

## 2. 命令字定义

### 上位机 → 下位机（发送）

| CMD_ID | 名称 | DATA | 长度 |
|---|---|---|---|
| `0x01` | 底盘速度 | `vx, vy, wz`（3 × float32） | 12 B |
| `0x02` | 云台 | `pitch, yaw`（2 × float32，弧度） | 8 B |
| `0x03` | 射击 | `mode`（1 × uint8） | 1 B |

### 下位机 → 上位机（接收）

| CMD_ID | 名称 | DATA | 长度 |
|---|---|---|---|
| `0x10` | 底盘状态 | `x, y, yaw, vx, vy, wz`（6 × float32） | 24 B |
| `0x11` | 云台状态 | `pitch, yaw`（2 × float32） | 8 B |
| `0x12` | 裁判系统 | 原始字节透传 | 任意 |

## 3. 话题映射

| 方向 | ROS 话题 | 帧 |
|---|---|---|
| 订阅 | `cmd_vel` (Twist) | `0x01` 底盘速度 |
| 订阅 | `cmd_gimbal_joint` (JointState) | `0x02` 云台 |
| 订阅 | `cmd_shoot` (UInt8) | `0x03` 射击 |
| 发布 | `chassis_odometry` (Odometry) | `0x10` 底盘状态 |
| 发布 | `gimbal_state` (JointState) | `0x11` 云台状态 |
| 发布 | `referee_data` (UInt8MultiArray) | `0x12` 裁判系统 |

## 4. 编译与运行

```bash
colcon build --packages-select pb_serial_comm
source install/setup.bash

# 运行（STM32 枚举成 /dev/ttyACM0）
ros2 launch pb_serial_comm serial_comm_launch.py

# 指定设备 / 波特率
ros2 launch pb_serial_comm serial_comm_launch.py device:=/dev/ttyUSB0 baud_rate:=115200
```

串口未插/未枚举时节点会每 2 秒自动重试打开，插上即可自动连接。

## 5. 对接 STM32 示例（伪代码）

```c
// STM32 端 CRC16（与上位机一致）
uint16_t crc16(uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
  }
  return crc;
}

// 解析一帧（CDC 接收回调里逐字节喂给状态机）
// 帧：0xA5 | cmd | len | data[len] | crc16_lo | crc16_hi | 0x5A
```

## 6. 自定义协议

改两处即可换成你自己的协议：

- 帧头/帧尾/CRC/命令字：改 [include/pb_serial_comm/frame.hpp](include/pb_serial_comm/frame.hpp) 的常量。
- 话题与帧的映射/数据布局：改 [src/serial_comm_node.cpp](src/serial_comm_node.cpp) 里的回调与 `handleFrame`。
