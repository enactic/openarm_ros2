# OpenArm Bringup

This package provides launch files to bring up the OpenArm robot system.

## Quick Start

Launch the OpenArm with v2.0 configuration and fake hardware:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py
```

Launch the OpenArm with v1.0 configuration and real hardware:

```bash
ros2 launch openarm_bringup openarm.bimanual.launch.py arm_type:=v10 use_fake_hardware:=false
```

## Launch Files

- `openarm.bimanual.launch.py` - Dual arm configuration

## Key Parameters

- `arm_type` - Arm type: `v1.0`/`v10`/`openarm_v1.0` or `v2.0`/`v20`/`openarm_v2.0` (default: openarm_v2.0)
- `use_fake_hardware` - Use fake hardware instead of real hardware (default: true)
- `right_can_interface` - CAN interface to use for the right arm (default: can0)
- `left_can_interface` - CAN interface to use for the left arm (default: can1)
- `can_fd` - Use CAN-FD. Set to `false` for CAN 2.0 (default: true)
- `robot_controller` - Controller type: `joint_trajectory_controller` (default) or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization
