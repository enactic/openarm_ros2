# OpenArm MuJoCo Hardware Interface

This package provides a ros2_control hardware interface for simulating OpenArm using the MuJoCo physics engine in place of physical hardware. It connects to a WebAssembly instance of MuJoCo through WebSockets.

## Usage

Certain OpenArm packages have been configured to use this interface by specifying the `hardware_type` flag.
This flag defaults to `real`, which assumes the use of physical hardware.
Setting the flag to `mock` uses an interface that writes commands to states directly (with some delays).
Setting the flag to `sim` uses the MuJoCo hardware interface.

For example, to use MoveIt2 with simulated bimanual hardware, first run MuJoCo by visiting:

[github.com/thomasonzhou/mujoco_anywhere](https://github.com/thomasonzhou/mujoco_anywhere)

Then run the original command with the `hardware_type` flag:
```sh
ros2 launch -d openarm_bimanual_moveit_config demo.launch.py hardware_type:=sim
```

*It may be necessary to install the nlohmann-json-dev library before building*

Please note that running multiple instances of the website will cause conflicting signals. Future configurations will allow for multiple instances to run simultaneously.

## Configuration

### Hardware Plugin Config

The hardware plugin is specified in `openarm_description/openarm.ros2_control.xacro` as follows:

```xml
<ros2_control name="openarm_system" type="system">
  <hardware>
    <plugin>openarm_mujoco_hardware/MujocoHardware</plugin>
    <param name="prefix">left_</param>
    <param name="websocket_port">1337</param>
  </hardware>
  <!-- Joint configurations -->
</ros2_control>
```

When using OpenArm in a bimanual configuration, the WebSocket ports default to 1337 for right arm and 1338 for left arm commands. However, in practice commands can be sent and states can be received through any connected port.
