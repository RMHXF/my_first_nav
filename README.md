# my_first_nav

本项目基于 [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_sentry_nav)（深圳北理莫斯科大学 · 北极熊战队 2025 赛季哨兵导航，ROS2 Humble + Nav2）修改而来。

主要做了两件事：**把感知传感器从 Livox mid360 换成 Odin1**（自带 SLAM 的 dTOF 传感器），并**新增上下位机串口通信**。livox 链路全程保留，通过一个启动参数即可随时切回。

---

## 相较于源码的更改

### 1. Odin1 驱动迁移（mid360 → Odin1）

- 新增 **`odin_interface`** 适配节点：把 Odin1 的 `/odin1/odometry`（odom→imu）+ `/odin1/cloud_raw` 转换成 `lidar_odometry` + `registered_scan` + `map→odom`，输出接口与原有 `loam_interface` 完全一致，下游 `sensor_scan_generation` → terrain 链 → Nav2 **零改动**。
  - 点云变换时用 `PointXYZ` 读取、手动补 FLOAT32 `intensity` 字段（Odin 点云的 intensity 是 UINT8，PCL 无法直接转换）。
  - `map→odom`：SLAM 模式下发布恒等变换（Odin 的 `odom` 帧即全局地图帧），重定位模式下取逆 Odin 的 `odom→map`。
- 集成 **`odin_ros_driver`**（Odin1 官方驱动，含 `lib/liblydHostApi_amd.a` / `arm.a` 预编译静态库）。
- 新增 **`lidar:=livox|odin` 启动开关**：一键切换两种传感器，`params_file` 按 `lidar` 自动选择。
- 新增 `config/reality/odin1_nav2_params.yaml` 与 `launch/odin_driver_launch.py`（Odin 驱动 + 云台外参静态 TF）。
- Odin 时间戳对齐：`odin_ros_driver/config/control_command.yaml` 中 `use_host_ros_time: 1`（改用主机 ROS 时钟，解决 Odin 设备时间与 ROS 系统时间不一致导致 tf2 查不到变换的问题）。
- 无先验地图模式：全局代价地图去掉 `static_layer`，靠 Odin 内置 SLAM + terrain 链**在线建图**。

### 2. 串口通信（上下位机）

- 新增 **`pb_serial_comm`** 包：通过 USB 虚拟串口与下位机 STM32 收发二进制帧（帧头 `0xA5` + 命令字 + 长度 + 数据 + MODBUS CRC16 + 帧尾 `0x5A`）。
  - 下发：`cmd_vel`（底盘速度）、`cmd_gimbal_joint`（云台）、`cmd_shoot`（射击）。
  - 回传：底盘状态（里程计）、云台状态、裁判系统数据。
  - 串口未插时每 2 秒自动重试打开。协议细节见 [pb_serial_comm/README.md](pb_serial_comm/README.md)。

### 3. pb_nav2_plugins 修复

- 代价地图插件库改名 `layers` → **`pb_nav2_layers`**，解决与 Nav2 自带 `liblayers.so` 重名、导致 `undefined symbol: nav2_costmap_2d::ObstacleLayer` 的问题。

### 4. 去子模块化

- `livox_ros_driver2`、`point_lio`、`pb_nav2_plugins` 等 7 个 git 子模块内容全部转为普通文件，仓库**自包含**，clone 下来即可编译，无需 `git submodule update`。

### 5. 其他

- `pb2025_sentry_nav/package.xml`、`pb2025_nav_bringup/package.xml` 增补 `odin_interface`、`odin_ros_driver` 依赖。
- 新增移植文档 [docs/ODIN1驱动移植指南.md](docs/ODIN1驱动移植指南.md)。

---

## 运行

```bash
colcon build
source install/setup.bash

# Odin1（无先验地图，在线 SLAM + 导航）
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py lidar:=odin

# Livox mid360（原链路，需先验地图）
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py lidar:=livox world:=<MAP>

# 串口通信（STM32 虚拟串口，通常 /dev/ttyACM0）
ros2 launch pb_serial_comm serial_comm_launch.py device:=/dev/ttyACM0 baud_rate:=115200
```

> Odin1 外参标定：`odin_driver_launch.py` 里的 `ext_x/y/z/yaw/pitch/roll` 对应 `gimbal_yaw → lidar` 的位姿，需实测填入。

---

## 目录结构

```
pb2025_sentry_nav/
├── pb2025_nav_bringup/      # 顶层启动与参数（含 odin1_nav2_params.yaml、odin_driver_launch.py）
├── odin_interface/          # [新] Odin1 驱动适配节点
├── odin_ros_driver/         # [新] Odin1 官方驱动
├── pb_serial_comm/          # [新] 串口通信功能包
├── point_lio/               # LiDAR-Inertial 里程计（livox 用）
├── loam_interface/          # point_lio 输出转 odom 系（livox 用）
├── small_gicp_relocalization/  # 全局重定位（livox 用）
├── sensor_scan_generation/  # 底盘里程计与 TF 生成
├── terrain_analysis(_ext)/  # 地形分析，输出障碍点云
├── fake_vel_transform/      # 抵消云台自旋
├── pb_nav2_plugins/         # Nav2 插件（IntensityVoxelLayer，库已改名）
├── pb_omni_pid_pursuit_controller/  # 全向路径跟踪控制器
├── pb_teleop_twist_joy/     # 手柄控制
├── pointcloud_to_laserscan/ # 点云转 2D 激光
├── livox_ros_driver2/       # Livox 驱动（保留，livox 切换用）
└── docs/                    # 文档（含 ODIN1驱动移植指南）
```
