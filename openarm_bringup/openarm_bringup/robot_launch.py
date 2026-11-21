# Copyright 2026 Enactic, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Shared model and controller setup for bringup and MoveIt."""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


def resolve_model(arm_type, robot_preset):
    if robot_preset:
        preset = robot_preset
    elif arm_type in {
        "v1.0",
        "v10",
        "v1_0",
        "openarm_v1.0",
        "openarm_v10",
        "openarm_v1_0",
    }:
        preset = "v1/default_bimanual"
    elif arm_type in {
        "v2.0",
        "v20",
        "v2_0",
        "openarm_v2.0",
        "openarm_v20",
        "openarm_v2_0",
    }:
        preset = "v2/default_bimanual"
    else:
        raise ValueError(f"Unknown arm_type: {arm_type}")
    if preset not in {"v1/default_bimanual", "v2/default_bimanual"}:
        raise ValueError(
            f"{preset} requires a matching controller and MoveIt configuration; this entry supports standard bimanual presets"
        )
    return preset, "openarm_v1.0" if preset.startswith("v1/") else "openarm_v2.0"


def resolve_hardware(hardware_type, use_fake_hardware):
    if hardware_type:
        if hardware_type not in {"real", "mock", "mujoco"}:
            raise ValueError(f"Unknown hardware_type: {hardware_type}")
        return hardware_type
    value = use_fake_hardware.lower()
    if value not in {"true", "false", "1", "0"}:
        raise ValueError("use_fake_hardware must be true or false")
    return "mock" if value in {"true", "1"} else "real"


def start_robot(context, with_moveit=False):
    def arg(name): return LaunchConfiguration(name).perform(context)
    preset, version = resolve_model(arg("arm_type"), arg("robot_preset"))
    backend = resolve_hardware(arg("hardware_type"), arg("use_fake_hardware"))
    if backend == "mujoco" and version != "openarm_v2.0":
        raise ValueError(
            "The current Web MuJoCo bridge supports the standard V2 bimanual preset"
        )
    package = Path(get_package_share_directory("openarm_bringup"))
    description_package = Path(
        get_package_share_directory(arg("description_package")))
    entry = package / "urdf/openarm.urdf.xacro"
    mappings = {
        "description_root": str(description_package),
        "robot_preset": preset,
        "use_fake_hardware": "true" if backend == "mock" else "false",
        "hardware_config": str(package / "config/hardware/mujoco.yaml")
        if backend == "mujoco"
        else "",
        "mujoco_command_interface": arg("mujoco_command_interface"),
        **{
            name: arg(name)
            for name in ("left_can_interface", "right_can_interface", "can_fd")
        },
    }
    moveit = None
    if with_moveit:
        from moveit_configs_utils import MoveItConfigsBuilder

        moveit = (
            MoveItConfigsBuilder(
                "openarm", package_name="openarm_bimanual_moveit_config"
            )
            .robot_description(file_path=entry, mappings=mappings)
            .robot_description_semantic(
                file_path=f"config/{version}/openarm_bimanual.srdf"
            )
            .robot_description_kinematics(file_path=f"config/{version}/kinematics.yaml")
            .joint_limits(file_path=f"config/{version}/joint_limits.yaml")
            .trajectory_execution(file_path=f"config/{version}/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .to_moveit_configs()
        )
        description = moveit.robot_description["robot_description"]
    else:
        description = xacro.process_file(str(entry), mappings=mappings).toxml()
    # Use this exact expansion in every consumer, including MoveIt and RViz.
    model = {"robot_description": description}
    root = ET.fromstring(description)
    systems = root.findall("ros2_control")
    if backend == "mujoco":
        expected = set(
            yaml.safe_load(
                (package / "config/hardware/mujoco.yaml").read_text())
        )
        if {s.get("name") for s in systems} != expected or any(
            s.findtext(
                "hardware/plugin") != "openarm_mujoco_hardware/MujocoHardware"
            for s in systems
        ):
            raise ValueError(
                "MuJoCo backend configuration must cover every active control system"
            )
        get_package_share_directory("openarm_mujoco_hardware")
    namespace = arg("arm_prefix").strip("/")
    manager_name = (
        f"/{namespace}/controller_manager" if namespace else "/controller_manager"
    )
    controllers_file = Path(arg("controllers_file"))
    if not controllers_file.is_absolute():
        controllers_file = (
            Path(get_package_share_directory(arg("runtime_config_package")))
            / "config/controllers"
            / controllers_file
        )
    manager_parameters = [model, str(controllers_file)]
    if backend == "mujoco":
        manager_parameters.append(
            {
                "hardware_components_initial_state.inactive": [
                    s.get("name") for s in systems
                ]
            }
        )
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[model],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace=namespace,
            parameters=manager_parameters,
            output="screen",
        ),
    ]
    controller = arg("robot_controller")
    controller_names = [
        "joint_state_broadcaster",
        f"left_{controller}",
        f"right_{controller}",
        "left_gripper_controller",
        "right_gripper_controller",
    ]
    spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            *controller_names,
            "-c",
            manager_name,
            "--controller-manager-timeout",
            "60",
        ],
    )
    if backend == "mujoco":
        ready = Node(
            package="openarm_mujoco_hardware",
            executable="activate_when_connected.py",
            arguments=[manager_name, *[s.get("name") for s in systems]],
            output="screen",
        )
        actions += [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=ready,
                    on_exit=lambda event, _: [spawner]
                    if event.returncode == 0
                    else [
                        EmitEvent(
                            event=Shutdown(
                                reason="MuJoCo hardware did not become ready"
                            )
                        )
                    ],
                )
            ),
            ready,
        ]
    else:
        actions.append(spawner)
    if moveit:
        moveit_package = Path(
            get_package_share_directory("openarm_bimanual_moveit_config")
        )
        params = moveit.to_dict()
        limits = moveit_package / \
            f"config/{version}/pilz_cartesian_limits.yaml"
        if limits.exists():
            params.setdefault("robot_description_planning", {}).update(
                yaml.safe_load(limits.read_text())
            )
        actions.append(
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                namespace=namespace,
                parameters=[params],
                output="screen",
            )
        )
        rviz_config = moveit_package / f"config/{version}/moveit.rviz"
    else:
        params = model
        rviz_config = description_package / "rviz/bimanual.rviz"
    actions.append(
        Node(
            package="rviz2",
            executable="rviz2",
            namespace=namespace,
            arguments=["-d", str(rviz_config)],
            parameters=[params],
            output="log",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    return actions


def generate_launch_description(with_moveit=False):
    arguments = [
        DeclareLaunchArgument(
            "description_package", default_value="openarm_description"
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="v20.urdf.xacro",
            description="Retained legacy argument; robot_preset selects the model",
        ),
        DeclareLaunchArgument("arm_type", default_value="openarm_v2.0"),
        DeclareLaunchArgument(
            "robot_preset",
            default_value="",
            description="Full preset name; defaults to arm_type's standard bimanual model",
        ),
        DeclareLaunchArgument(
            "hardware_type",
            default_value="",
            choices=["", "real", "mock", "mujoco"],
            description="Overrides use_fake_hardware when supplied",
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "mujoco_command_interface",
            default_value="position",
            choices=["position", "effort"],
        ),
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            choices=["forward_position_controller",
                     "joint_trajectory_controller"],
        ),
        DeclareLaunchArgument(
            "runtime_config_package", default_value="openarm_bringup"
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="openarm_bimanual_moveit_controllers.yaml"
            if with_moveit
            else "openarm_bimanual_controllers.yaml",
        ),
        DeclareLaunchArgument("arm_prefix", default_value=""),
        DeclareLaunchArgument("left_can_interface", default_value="can1"),
        DeclareLaunchArgument("right_can_interface", default_value="can0"),
        DeclareLaunchArgument("can_fd", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
    ]
    return LaunchDescription(
        arguments
        + [OpaqueFunction(function=start_robot,
                          kwargs={"with_moveit": with_moveit})]
    )
