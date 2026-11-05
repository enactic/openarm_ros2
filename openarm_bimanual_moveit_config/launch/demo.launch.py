# Copyright 2025 Enactic, Inc.
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
import xml.etree.ElementTree as ET
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# All accepted arm_type values
VALID_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
    "v2.0", "v20", "v2_0", "openarm_v2.0", "openarm_v20", "openarm_v2_0",
}


def resolve_arm_config(arm_type_str: str) -> tuple[str, str]:
    """
    Resolve folder name and xacro file name from arm_type.
    Accepts: v1.0, v10, v1_0, openarm_v1.0, openarm_v10, openarm_v1_0 (and v2.0 variants)
    Raises ValueError if arm_type is not recognized.
    """
    if arm_type_str not in VALID_ARM_TYPES:
        raise ValueError(
            f"Invalid arm_type: '{arm_type_str}'. "
            f"Please specify openarm_v1.0 or openarm_v2.0."
        )
    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        return "openarm_v1.0", "openarm_v10.urdf.xacro"
    return "openarm_v2.0", "openarm_v20.urdf.xacro"


def apply_can_overrides(robot_description, left_can_interface, right_can_interface, can_fd):
    """Apply explicit launch overrides to the standard bimanual hardware systems."""
    can_fd = can_fd.strip().lower()
    if can_fd not in ("", "true", "false", "0", "1"):
        raise ValueError("can_fd must be true, false, 1, 0, or empty.")
    can_fd = {"1": "true", "0": "false"}.get(can_fd, can_fd)
    if not (left_can_interface or right_can_interface or can_fd):
        return robot_description

    robot = ET.fromstring(robot_description)
    for side, interface in (("left", left_can_interface), ("right", right_can_interface)):
        overrides = {name: value for name, value in (
            ("can_interface", interface.strip()), ("can_fd", can_fd)) if value}
        if not overrides:
            continue
        system_name = f"openarm_{side}_hardware_interface"
        hardware = robot.find(f"ros2_control[@name='{system_name}']/hardware")
        if hardware is None:
            raise ValueError(
                f"Cannot apply CAN overrides: missing {system_name}.")
        for name, value in overrides.items():
            parameter = hardware.find(f"param[@name='{name}']")
            if parameter is None:
                parameter = ET.SubElement(hardware, "param", name=name)
            parameter.text = value
    return ET.tostring(robot, encoding="unicode")


def generate_robot_description(
    context: LaunchContext,
    description_package,
    arm_type,
    use_fake_hardware,
    robot_preset,
    left_can_interface,
    right_can_interface,
    can_fd,
):
    description_package_str = context.perform_substitution(description_package)
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    robot_preset_str = context.perform_substitution(robot_preset)

    folder_name, file_name = resolve_arm_config(arm_type_str)

    xacro_path = os.path.join(
        get_package_share_directory(description_package_str),
        "assets", "robot", folder_name, "urdf", file_name
    )

    robot_description = xacro.process_file(
        xacro_path,
        mappings={
            "arm_type": arm_type_str,
            "bimanual": "true",
            "use_fake_hardware": use_fake_hardware_str,
            "ros2_control": "true",
            "robot_preset": robot_preset_str,
        },
    ).toprettyxml(indent="  ")

    return apply_can_overrides(
        robot_description,
        context.perform_substitution(left_can_interface),
        context.perform_substitution(right_can_interface),
        context.perform_substitution(can_fd),
    )


def robot_nodes_spawner(
    context: LaunchContext,
    description_package,
    arm_type,
    use_fake_hardware,
    controllers_file,
    robot_preset,
    arm_prefix,
    left_can_interface,
    right_can_interface,
    can_fd,
):
    robot_description = generate_robot_description(
        context,
        description_package,
        arm_type,
        use_fake_hardware,
        robot_preset,
        left_can_interface,
        right_can_interface,
        can_fd,
    )

    controllers_file_str = context.perform_substitution(controllers_file)
    robot_description_param = {"robot_description": robot_description}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description_param, controllers_file_str],
    )

    return [robot_state_pub_node, control_node] + moveit_nodes_spawner(
        context, arm_type, use_fake_hardware, robot_preset, robot_description
    )


def controller_spawner(context: LaunchContext, robot_controller):
    robot_controller_str = context.perform_substitution(robot_controller)

    if robot_controller_str == "forward_position_controller":
        left = "left_forward_position_controller"
        right = "right_forward_position_controller"
    elif robot_controller_str == "joint_trajectory_controller":
        left = "left_joint_trajectory_controller"
        right = "right_joint_trajectory_controller"
    else:
        raise ValueError(f"Unknown robot_controller: {robot_controller_str}")

    return [
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[left, right, "-c", "/controller_manager"],
        )
    ]


def moveit_nodes_spawner(context: LaunchContext, arm_type, use_fake_hardware,
                         robot_preset, robot_description):
    arm_type_str = context.perform_substitution(arm_type)
    use_fake_hardware_str = context.perform_substitution(use_fake_hardware)
    robot_preset_str = context.perform_substitution(robot_preset)

    description_pkg_path = get_package_share_directory("openarm_description")
    moveit_pkg_path = get_package_share_directory(
        "openarm_bimanual_moveit_config")

    folder_name, file_name = resolve_arm_config(arm_type_str)

    xacro_path = os.path.join(
        description_pkg_path, "assets", "robot", folder_name, "urdf", file_name
    )

    if any(x in arm_type_str for x in ("1.0", "10", "1_0")):
        config_dir = "openarm_v1.0"
    else:
        config_dir = "openarm_v2.0"

    moveit_config = (
        MoveItConfigsBuilder(
            "openarm", package_name="openarm_bimanual_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "arm_type": arm_type_str,
                "bimanual": "true",
                "use_fake_hardware": use_fake_hardware_str,
                "ros2_control": "true",
                "robot_preset": robot_preset_str,
            }
        )
        .robot_description_semantic(file_path=f"config/{config_dir}/openarm_bimanual.srdf")
        .robot_description_kinematics(file_path=f"config/{config_dir}/kinematics.yaml")
        .joint_limits(file_path=f"config/{config_dir}/joint_limits.yaml")
        .trajectory_execution(file_path=f"config/{config_dir}/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl"
        )
        .to_moveit_configs()
    )

    moveit_config.robot_description = {"robot_description": robot_description}
    moveit_params = moveit_config.to_dict()

    pilz_cartesian_limits_path = os.path.join(
        moveit_pkg_path, "config", config_dir, "pilz_cartesian_limits.yaml"
    )

    if os.path.exists(pilz_cartesian_limits_path):
        import yaml
        with open(pilz_cartesian_limits_path, 'r') as f:
            config_data = yaml.safe_load(f)
            if "cartesian_limits" in config_data:
                if "robot_description_planning" not in moveit_params:
                    moveit_params["robot_description_planning"] = {}
                moveit_params["robot_description_planning"].update(config_data)

    rviz_cfg = os.path.join(moveit_pkg_path, "config",
                            config_dir, "moveit.rviz")

    return [
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_params],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_cfg],
            parameters=[moveit_params],
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("description_package",
                              default_value="openarm_description"),
        DeclareLaunchArgument(
            "arm_type",
            default_value="openarm_v2.0",
            description="Arm type. Accepts: v1.0, v10, openarm_v1.0, v2.0, v20, openarm_v2.0, etc."
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
        ),
        DeclareLaunchArgument("runtime_config_package",
                              default_value="openarm_bringup"),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument(
            "robot_preset", default_value="default_bimanual"),
        DeclareLaunchArgument("left_can_interface", default_value=""),
        DeclareLaunchArgument("right_can_interface", default_value=""),
        DeclareLaunchArgument("can_fd", default_value=""),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_bimanual_moveit_controllers.yaml"),
    ]

    description_package = LaunchConfiguration("description_package")
    arm_type = LaunchConfiguration("arm_type")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_controller = LaunchConfiguration("robot_controller")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    robot_preset = LaunchConfiguration("robot_preset")
    left_can_interface = LaunchConfiguration("left_can_interface")
    right_can_interface = LaunchConfiguration("right_can_interface")
    can_fd = LaunchConfiguration("can_fd")
    arm_prefix = LaunchConfiguration("arm_prefix")

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config",
         "controllers", controllers_file]
    )

    robot_nodes_spawner_func = OpaqueFunction(
        function=robot_nodes_spawner,
        args=[
            description_package,
            arm_type,
            use_fake_hardware,
            controllers_file,
            robot_preset,
            arm_prefix,
            left_can_interface,
            right_can_interface,
            can_fd,
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    controller_spawner_func = OpaqueFunction(
        function=controller_spawner, args=[robot_controller]
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller",
                   "right_gripper_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_nodes_spawner_func,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(period=1.0, actions=[controller_spawner_func]),
            TimerAction(period=1.0, actions=[gripper_spawner]),
        ]
    )
