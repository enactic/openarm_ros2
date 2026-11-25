# Copyright 2026 Enactic, Inc.
# SPDX-License-Identifier: Apache-2.0
"""Shared model and controller setup for bringup and MoveIt."""

import importlib.util
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml

V1_ARM_TYPES = {"v1.0", "v10", "v1_0",
                "openarm_v1.0", "openarm_v10", "openarm_v1_0"}
V2_ARM_TYPES = {"v2.0", "v20", "v2_0",
                "openarm_v2.0", "openarm_v20", "openarm_v2_0"}
# real: built-in OpenArm hardware; mock: mock_components; mujoco: the
# ros-controls MuJoCo plugin; external: no ros2_control, another process
# consumes the controller command topics and publishes joint_states.
HARDWARE_TYPES = {"real", "mock", "mujoco", "external"}


def resolve_model(arm_type, robot_preset):
    if robot_preset:
        preset = robot_preset
    elif arm_type in V1_ARM_TYPES:
        preset = "v1/default_bimanual"
    elif arm_type in V2_ARM_TYPES:
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
        if hardware_type not in HARDWARE_TYPES:
            raise ValueError(f"Unknown hardware_type: {hardware_type}")
        return hardware_type
    value = use_fake_hardware.lower()
    if value not in {"true", "false", "1", "0"}:
        raise ValueError("use_fake_hardware must be true or false")
    return "mock" if value in {"true", "1"} else "real"


def resolve_mujoco_model(path):
    """Return the MJCF path, falling back to an installed openarm_mujoco."""
    if path:
        return str(Path(path).expanduser().resolve())
    spec = importlib.util.find_spec("openarm_mujoco")
    for root in (spec.submodule_search_locations or []) if spec else []:
        candidate = Path(root) / "v2/pedestal/pedestal.xml"
        if candidate.is_file():
            return str(candidate)
    raise ValueError(
        "hardware_type:=mujoco needs mujoco_model:=<MJCF path> or an installed openarm_mujoco"
    )


def write_hardware_config(config, cleanup):
    directory = tempfile.TemporaryDirectory(prefix="openarm-hardware-")
    cleanup.append(directory)
    path = Path(directory.name) / "hardware.yaml"
    path.write_text(yaml.safe_dump(config))
    return str(path)


def start_robot(context, with_moveit=False):
    def arg(name): return LaunchConfiguration(name).perform(context)
    preset, version = resolve_model(arg("arm_type"), arg("robot_preset"))
    backend = resolve_hardware(arg("hardware_type"), arg("use_fake_hardware"))
    if backend == "mujoco" and version != "openarm_v2.0":
        raise ValueError(
            "The MuJoCo backend supports the standard V2 bimanual preset")
    package = Path(get_package_share_directory("openarm_bringup"))
    description_package = Path(
        get_package_share_directory(arg("description_package")))
    entry = description_package / "urdf/openarm.urdf.xacro"
    sim_time = backend == "mujoco"
    cleanup = []

    # The hardware configuration file replaces the preset systems; see
    # openarm_description's README. Without one, mock and real use the
    # description's built-in systems.
    hardware_config = arg("hardware_config")
    if not hardware_config and backend == "mujoco":
        config = yaml.safe_load(
            (package / "config/hardware/mujoco.yaml").read_text())
        for system in config["systems"].values():
            system.setdefault("params", {}).update(
                mujoco_model=resolve_mujoco_model(arg("mujoco_model")),
                headless=arg("mujoco_headless").lower() in {"true", "1"})
        hardware_config = write_hardware_config(config, cleanup)
    mappings = {
        "robot_preset": preset,
        "ros2_control": "false" if backend == "external" else "true",
        "use_fake_hardware": "true" if backend == "mock" else "false",
        "hardware_config": hardware_config,
    }
    if backend == "real" and not hardware_config:
        mappings.update({name: arg(name) for name in (
            "left_can_interface", "right_can_interface", "can_fd")})

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
    clock = {"use_sim_time": sim_time}
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
    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=namespace,
            parameters=[model, clock],
            output="screen",
        ),
    ]
    if cleanup:
        def remove_temporary_files(event, context):
            for directory in cleanup:
                directory.cleanup()

        actions.append(RegisterEventHandler(
            OnShutdown(on_shutdown=remove_temporary_files)))
    if backend != "external":
        # The MuJoCo plugin steps the simulation inside its own control node.
        actions.append(
            Node(
                package="mujoco_ros2_control" if backend == "mujoco" else "controller_manager",
                executable="ros2_control_node",
                namespace=namespace,
                parameters=[model, str(controllers_file), clock],
                output="screen",
            )
        )
        controller = arg("robot_controller")
        actions.append(
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "joint_state_broadcaster",
                    f"left_{controller}",
                    f"right_{controller}",
                    "left_gripper_controller",
                    "right_gripper_controller",
                    "-c",
                    manager_name,
                    "--controller-manager-timeout",
                    "60",
                ],
            )
        )
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
                parameters=[params, clock],
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
            parameters=[params, clock],
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
            choices=["", *sorted(HARDWARE_TYPES)],
            description="Overrides use_fake_hardware when supplied",
        ),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
        DeclareLaunchArgument(
            "hardware_config",
            default_value="",
            description="openarm_description hardware configuration replacing the preset systems",
        ),
        DeclareLaunchArgument(
            "mujoco_model",
            default_value="",
            description="MJCF scene for hardware_type:=mujoco; defaults to openarm_mujoco's V2 pedestal",
        ),
        DeclareLaunchArgument("mujoco_headless", default_value="false"),
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
