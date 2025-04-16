import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
import pathlib


def generate_launch_description():
    
    
    openarm_mujoco_ros2_control_path = pathlib.Path(FindPackageShare('openarm_mujoco_ros2_control').find('openarm_mujoco_ros2_control'))

    urdf_file = os.path.join(openarm_mujoco_ros2_control_path,
                              'urdf',
                              'openarm_bimanual_mujoco.urdf')

    robot_description = {'robot_description': ParameterValue(open(urdf_file).read(), value_type=str)}

    openarm_bimanual_teleop_path = pathlib.Path(FindPackageShare('openarm_bimanual_teleop').find('openarm_bimanual_teleop'))
    openarm_bimanual_moveit_config_path = pathlib.Path(FindPackageShare('openarm_bimanual_moveit_config').find('openarm_bimanual_moveit_config'))

    controller_config_file = str((openarm_bimanual_teleop_path/ 'config'/ 'bimanual_controllers.yaml').resolve())
    controller_config_file = str((openarm_bimanual_moveit_config_path/ 'config'/ 'ros2_controllers.yaml').resolve())


    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': ParameterValue(str((openarm_mujoco_ros2_control_path/ 'mjcf'/ 'openarm_bimanual.mjcf.xml').resolve()), value_type=str),
             'update_rate': 100},
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        # update_rate_launch_arg,
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])