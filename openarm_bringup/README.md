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
- `robot_preset` - Description preset containing each component's `control` configuration (default: `default_bimanual`). Set `can_interface` and `can_fd` directly in that preset.
- `left_can_interface` - Override the left arm's CAN device for this launch (default: empty, using the preset).
- `right_can_interface` - Override the right arm's CAN device for this launch (default: empty, using the preset).
- `can_fd` - Override CAN-FD mode for both arms with `true` or `false` (default: empty, using the preset).
- `robot_controller` - Controller type: `joint_trajectory_controller` (default) or `forward_position_controller`
- `arm_prefix` - Namespace for the robot's nodes and topics, for example `leader` (default: empty, no namespace)

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization

Explicit CAN launch arguments override the preset's values in the generated
`ros2_control` hardware configuration. The standard bimanual launch targets
`openarm_left_hardware_interface` and `openarm_right_hardware_interface`.
For example, select a different left device and use CAN 2.0 with
`left_can_interface:=can2 can_fd:=false`.
