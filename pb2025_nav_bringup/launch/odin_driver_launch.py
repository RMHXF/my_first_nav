# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Odin1 驱动 + 云台外参。仅供 lidar:=odin 时由 rm_navigation_reality_launch.py 调用。
#
# 只启动 host_sdk_sample 本体（Odin 官方 launch 还会连带启动 pcd2depth / reprojection /
# image_overlay / RViz 等 demo 节点，集成时不需要）。
#
# 注意：host_sdk_sample 内部通过 get_package_source_directory() 直接读取
#       <odin_ros_driver 源码目录>/config/control_command.yaml，并不会读取 launch 传入的
#       config_file 参数。因此 Odin 的 custom_map_mode / sendcloudslam 等配置请直接改源码树
#       里的 control_command.yaml（本仓库 src/pb2025_sentry_nav/odin_ros_driver/config/）。
#
# 外参 gimbal_yaw -> lidar 需要实测标定后传入，默认全 0（占位）。

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    # 外参 gimbal_yaw -> lidar（x y z yaw pitch roll），实测标定。
    ext_x = LaunchConfiguration("ext_x")
    ext_y = LaunchConfiguration("ext_y")
    ext_z = LaunchConfiguration("ext_z")
    ext_yaw = LaunchConfiguration("ext_yaw")
    ext_pitch = LaunchConfiguration("ext_pitch")
    ext_roll = LaunchConfiguration("ext_roll")

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_ext_x = DeclareLaunchArgument("ext_x", default_value="0.0")
    declare_ext_y = DeclareLaunchArgument("ext_y", default_value="0.0")
    declare_ext_z = DeclareLaunchArgument("ext_z", default_value="0.0")
    declare_ext_yaw = DeclareLaunchArgument("ext_yaw", default_value="0.0")
    declare_ext_pitch = DeclareLaunchArgument("ext_pitch", default_value="0.0")
    declare_ext_roll = DeclareLaunchArgument("ext_roll", default_value="0.0")

    start_odin_driver = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        namespace=namespace,
        output="screen",
    )

    # 云台 -> Odin 外参静态 TF。校准后通过 launch 参数覆盖，例如：
    #   ext_x:=0.0 ext_y:=0.0 ext_z:=0.1 ext_yaw:=0.0 ext_pitch:=-0.5 ext_roll:=0.0
    start_extrinsic_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gimbal_yaw_to_lidar",
        namespace=namespace,
        arguments=[ext_x, ext_y, ext_z, ext_yaw, ext_pitch, ext_roll, "gimbal_yaw", "lidar"],
    )

    # 独立导航测试时 robot_state_publisher（robot_description）不运行，导致 gimbal_yaw 无父帧、
    # Nav2 的 robot_base_frame(gimbal_yaw_fake) 断链。这里补一条 base_footprint -> gimbal_yaw 恒等
    # 静态 TF 作为近似（假设云台与底盘同向）。完整系统应改用机器人主 bringup 的 robot_state_publisher。
    start_gimbal_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_footprint_to_gimbal_yaw",
        namespace=namespace,
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "gimbal_yaw"],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(declare_ext_x)
    ld.add_action(declare_ext_y)
    ld.add_action(declare_ext_z)
    ld.add_action(declare_ext_yaw)
    ld.add_action(declare_ext_pitch)
    ld.add_action(declare_ext_roll)
    ld.add_action(start_odin_driver)
    ld.add_action(start_extrinsic_tf)
    ld.add_action(start_gimbal_mount_tf)
    return ld
