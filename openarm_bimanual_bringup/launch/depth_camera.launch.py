from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense_share_dir = FindPackageShare("realsense2_camera")
    rs_launch_path = [realsense_share_dir, "/launch/rs_launch.py"]

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch_path),
            launch_arguments={"pointcloud.enable": "true"}.items()
        )
    ])
