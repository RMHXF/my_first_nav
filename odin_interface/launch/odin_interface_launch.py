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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    declare_namespace = DeclareLaunchArgument(
        "namespace", default_value="", description="Namespace for the node"
    )

    start_odin_interface = Node(
        package="odin_interface",
        executable="odin_interface_node",
        name="odin_interface",
        namespace=namespace,
        remappings=remappings,
        output="screen",
        parameters=[
            {
                "input_odom_topic": "odin1/odometry",
                "input_cloud_topic": "odin1/cloud_raw",
                "odom_frame": "odom",
                "imu_frame": "imu",
                "lidar_frame": "lidar",
                "map_frame": "map",
                "publish_map_odom_tf": True,
                "map_odom_tf_rate": 20.0,
                # Fallback extrinsics (used when Odin TF is disabled). Calibrate
                # with: [x, y, z, roll, pitch, yaw] of `imu -> lidar`.
                "imu_to_lidar": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace)
    ld.add_action(start_odin_interface)
    return ld
