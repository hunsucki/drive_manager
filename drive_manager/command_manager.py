#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class CommandManager(Node):
    """Bridges mobile app commands to the internal mission driver."""

    def __init__(self):
        super().__init__(
            "command_manager",
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter_if_missing("command_topic", "/robot_command")
        self.declare_parameter_if_missing("status_topic", "/robot_status")
        self.declare_parameter_if_missing("mission_command_topic", "/mission_command")
        self.declare_parameter_if_missing("mission_status_topic", "/mission_status")

        self.estop_latched = False

        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            10,
        )
        self.mission_command_pub = self.create_publisher(
            String,
            self.get_parameter("mission_command_topic").value,
            10,
        )

        self.command_sub = self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self.command_callback,
            10,
        )
        self.mission_status_sub = self.create_subscription(
            String,
            self.get_parameter("mission_status_topic").value,
            self.mission_status_callback,
            10,
        )

        self.publish_status("IDLE")
        self.get_logger().info(
            "Command manager ready: "
            f"{self.get_parameter('command_topic').value} -> "
            f"{self.get_parameter('mission_command_topic').value}"
        )

    def declare_parameter_if_missing(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def command_callback(self, msg):
        command = self.normalize_command(msg.data)

        if command == "RESET":
            self.estop_latched = False
            self.forward_mission_command(command)
            self.publish_status("IDLE")
            return

        if command == "ESTOP":
            self.estop_latched = True
            self.forward_mission_command(command)
            self.publish_status("ESTOP_LATCHED")
            return

        if command == "STOP":
            self.forward_mission_command(command)
            self.publish_status("STOP_REQUESTED")
            return

        if command not in ("START", "HOME"):
            self.publish_status(f"ERROR unknown_command {command}")
            self.get_logger().warn(f"Unknown app command: {command}")
            return

        if self.estop_latched:
            self.publish_status("ESTOP_LATCHED reset_required")
            self.get_logger().warn(f"Ignoring {command}; RESET is required")
            return

        self.forward_mission_command(command)
        self.publish_status(f"COMMAND_SENT {command}")

    def mission_status_callback(self, msg):
        if msg.data == "ESTOP_LATCHED":
            self.estop_latched = True
        elif msg.data == "IDLE":
            self.estop_latched = False

        self.publish_status(msg.data)

    def normalize_command(self, command):
        command = command.strip().upper()
        aliases = {
            "GO": "START",
            "RETURN": "HOME",
            "DOCK": "HOME",
            "EMERGENCY_STOP": "ESTOP",
            "E_STOP": "ESTOP",
            "CANCEL": "STOP",
        }
        return aliases.get(command, command)

    def forward_mission_command(self, command):
        msg = String()
        msg.data = command
        self.mission_command_pub.publish(msg)
        self.get_logger().info(f"Forwarded mission command: {command}")

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")


def main():
    rclpy.init()
    node = CommandManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
