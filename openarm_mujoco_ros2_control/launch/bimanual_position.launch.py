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
# import xacro


def generate_launch_description():
    
    
    openarm_mujoco_ros2_control_path = pathlib.Path(FindPackageShare('openarm_mujoco_ros2_control').find('openarm_mujoco_ros2_control'))

    urdf_file = os.path.join(openarm_mujoco_ros2_control_path,
                              'urdf',
                              'openarm_bimanual_mujoco.urdf')

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    robot_description = {'robot_description': open(urdf_file).read()}

    openarm_bimanual_teleop_path = pathlib.Path(FindPackageShare('openarm_bimanual_teleop').find('openarm_bimanual_teleop'))

    controller_config_file = str((openarm_bimanual_teleop_path/ 'config'/ 'controllers.yaml').resolve())

    update_rate = LaunchConfiguration("update_rate")
    update_rate_launch_arg = DeclareLaunchArgument("update_rate", default_value="100.0")


    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            # controller_config_file,
            update_rate,
            {'mujoco_model_path': ParameterValue(str((openarm_mujoco_ros2_control_path/ 'mjcf'/ 'openarm_bimanual.mjcf.xml').resolve()), value_type=str)},
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

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        update_rate_launch_arg,
        node_mujoco_ros2_control,
        node_robot_state_publisher
    ])