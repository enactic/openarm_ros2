# Copyright 2025 Reazon Holdings, Inc.
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

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "hardware_type",
            default_value="real",
            description="Hardware interface type: 'real', 'sim' (MuJoCo), or 'mock'",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mock_sensor_commands",
            default_value="false",
            description="Enable writable sensor interfaces when using mock hardware",
        )
    )

    hardware_type = LaunchConfiguration("hardware_type")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")

    moveit_config = (
        MoveItConfigsBuilder("openarm_bimanual",
                             package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path="config/openarm_bimanual.urdf.xacro",
            mappings={
                "hardware_type": hardware_type,
                "mock_sensor_commands": mock_sensor_commands,
            },
        )
        .to_moveit_configs()
    )

    demo_ld = generate_demo_launch(moveit_config)

    ld = LaunchDescription()

    for arg in declared_arguments:
        ld.add_action(arg)

    for entity in demo_ld.entities:
        ld.add_action(entity)

    return ld
