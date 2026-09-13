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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("pb_serial_comm"),
        "config",
        "serial_comm.yaml",
    )

    declare_device = DeclareLaunchArgument(
        "device", default_value="/dev/ttyACM0", description="串口设备路径"
    )
    declare_baud_rate = DeclareLaunchArgument(
        "baud_rate", default_value="115200", description="波特率"
    )

    node = Node(
        package="pb_serial_comm",
        executable="serial_comm_node",
        name="serial_comm",
        output="screen",
        parameters=[config, {"device": LaunchConfiguration("device"),
                             "baud_rate": LaunchConfiguration("baud_rate")}],
    )

    return LaunchDescription([declare_device, declare_baud_rate, node])
