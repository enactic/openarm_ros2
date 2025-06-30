#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from pathlib import Path
import xacro

class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # Get URDF file parameter
        urdf_path = self.declare_parameter('urdf_path', '').get_parameter_value().string_value
        if not urdf_path or not Path(urdf_path).exists():
            self.get_logger().error(f"Invalid or missing URDF path: {urdf_path}")
            rclpy.shutdown()
            return

        # Parse URDF directly from file (not xacro)
        # self.robot = URDF.from_xml_file(urdf_path)

        # Process xacro and parse URDF
        doc = xacro.process_file(urdf_path)
        robot_description_xml = doc.toxml()
        self.robot = URDF.from_xml_string(robot_description_xml)

        # Initialize full joint position dict with zeros
        self.joint_positions = {
            j.name: 0.0 for j in self.robot.joints if j.type != 'fixed'
        }
        self.joint_names = list(self.joint_positions.keys())

        # Store joint limits (min/max) for normalization
        self.joint_limits = {
            j.name: (j.limit.lower, j.limit.upper) if j.limit else (-3.14, 3.14)
            for j in self.robot.joints if j.type != 'fixed'
        }

        # Publisher for merged joint states
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # Subscription to your partial updates — change topic & msg type as needed
        self.partial_update_sub = self.create_subscription(
            JointState, '/joint_states_partial', self.partial_update_callback, 10
        )

        # Optionally subscribe to existing /joint_states to keep current positions up to date
        self.existing_joint_states_sub = self.create_subscription(
            JointState, '/joint_states', self.existing_joint_states_callback, 10
        )

        # Timer to publish merged joint states regularly
        self.timer = self.create_timer(0.05, self.publish_merged_joint_states)

    def partial_update_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = pos

    def existing_joint_states_callback(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            if name in self.joint_positions:
                self.joint_positions[name] = pos

    def publish_merged_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.joint_positions[n] for n in self.joint_names]
        self.joint_state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
