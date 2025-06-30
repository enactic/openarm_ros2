import xacro
from pathlib import Path

from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = Path(
        launch_ros.substitutions.FindPackageShare(package="openarm_bimanual_description").find(
            "openarm_bimanual_description"
        )
    )
    default_model_path = pkg_share / "urdf/openarm_bimanual.urdf.xacro"
    default_rviz_config_path = pkg_share / "rviz/robot_description.rviz"

    # Parse xacro to get robot_description XML
    doc = xacro.process_file(str(default_model_path))
    robot_description_xml = doc.toxml()

    # Directly read udrf file (no xacro)
    # robot_description_xml = Path(default_model_path).read_text()

    # Node to publish joint states from /joint_states topic or GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # Node to publish robot state TFs based on robot_description and /joint_states
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_xml,
             "use_sim_time": False}
        ],
    )

    # RViz node to visualize the robot model and TF
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(default_rviz_config_path)],
        parameters=[{"use_sim_time": False}],
    )

    joint_state_merger_node = Node(
        package="openarm_bimanual_description",
        executable="joint_state_merger.py",
        name="joint_state_merger",
        output="screen",
        parameters=[{"urdf_path": str(default_model_path)}],
    )


    return LaunchDescription([
        #joint_state_publisher_node,
        robot_state_publisher_node,
        joint_state_merger_node,
        rviz_node,
    ])
