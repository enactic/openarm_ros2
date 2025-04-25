# ROS2 packages for OpenArm robots

[Quickstart](#installation)


https://github.com/user-attachments/assets/90b44ef4-5cdc-4bf5-b56f-be2a5ff264b4



- openarm_bimanual_description: dual arm urdf with torso and realsense head camera
- openarm_bimanual_moveit_config: bimanual motion planning with OctoMap occupancy grid mapping
- openarm_bimanual_bringup: setup scripts for bimanual openarm
- openarm_bringup: setup scripts for single physical openarm
- openarm_description: single arm urdf
- openarm_hardware: hardware interface for ros2_control
- openarm_moveit_config: motion planning with [moveit2](https://github.com/moveit/moveit2)


### Description Packages

Each link has a visual mesh and a collision mesh, as shown in the figures below:
  
<img width="412" alt="visual meshes of openarm_bimanual_description urdf in rviz2" src="https://github.com/user-attachments/assets/9020efc3-69bc-420d-93a1-305885925638" />
<img width="383" alt="collision meshes of openarm_bimanual_description urdf in rviz2" src="https://github.com/user-attachments/assets/6f62184e-ccea-4859-9364-7c7d1b8def86" />

### MoveIt2 Support

https://github.com/user-attachments/assets/a0f962e5-6150-49ce-b18e-9914bcb322ef

## Installation

1. [Install ROS2 and ros-dev-tools](https://docs.ros.org/en/humble/Installation.html) (tested on Humble with Ubuntu 22.04)
2. [Create a ROS2 workspace and source the overlay](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)

```sh
source /opt/ros/humble/setup.bash # change humble to your ROS2 distro
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/reazon-research/openarm_ros2.git
```

3. [Install dependencies with rosdep](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html) and [build the packages with colcon](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

```sh
cd ~/ros2_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src -r

sudo apt install -y python3-colcon-common-extensions
colcon build
```

4. *In a new terminal*, source the workspace setup script

```sh
cd ~/ros2_ws
source install/setup.bash
```

5. Test the installation by launching a demo. It may be necessary to restart your computer once.

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```


## License

All packages of `openarm_ros2` are licensed under the [BSD-3-Clause](https://opensource.org/license/bsd-3-clause).
