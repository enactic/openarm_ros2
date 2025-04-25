# MoveIt2 on Bimanual Openarms

Ensure the ROS2 packages and dependencies are installed by following the instructions in `openarm_ros2/README.md`.

## Physical Hardware
1. Run `init_can.sh` from `openarm_bringup/utils`. 
   By default, can0 is the right arm and can1 is the left arm, but this can be adjusted in the ros2_control definition in `openarm_description/urdf/openarm.ros2_control.xacro`.

2. Optionally, start the head-mounted realsense camera. This enables the octomap occupancy grid for planning around obstacles.
   
```sh
ros2 launch openarm_bimanual_bringup depth_camera.launch.py
```

## Launch the demo

```sh
ros2 launch openarm_bimanual_moveit_config demo.launch.py
```
