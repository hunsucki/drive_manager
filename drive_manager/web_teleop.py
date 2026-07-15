#!/usr/bin/env python3

import math
import threading
import time

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def clamp_twist(command, max_linear_x, max_angular_z):
    """Return a finite differential-drive Twist limited to configured bounds."""
    output = Twist()
    linear_x = command.linear.x if math.isfinite(command.linear.x) else 0.0
    angular_z = command.angular.z if math.isfinite(command.angular.z) else 0.0
    linear_limit = abs(float(max_linear_x))
    angular_limit = abs(float(max_angular_z))
    output.linear.x = max(-linear_limit, min(linear_limit, linear_x))
    output.angular.z = max(-angular_limit, min(angular_limit, angular_z))
    return output


class WebTeleop(Node):
    """Arbitrates collision-checked and low-speed recovery teleoperation."""

    DISABLED = "DISABLED"
    SAFE = "SAFE"
    FORCE = "FORCE"

    def __init__(self):
        super().__init__(
            "web_teleop",
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter_if_missing("safe_input_topic", "/cmd_vel_web_safe")
        self.declare_parameter_if_missing("force_input_topic", "/cmd_vel_web_force")
        self.declare_parameter_if_missing("safe_output_topic", "/cmd_vel_nav")
        self.declare_parameter_if_missing("force_output_topic", "/cmd_vel")
        self.declare_parameter_if_missing("status_topic", "/web_teleop/status")
        self.declare_parameter_if_missing("active_topic", "/web_teleop/active")
        self.declare_parameter_if_missing("command_timeout_sec", 0.35)
        self.declare_parameter_if_missing("stop_publish_duration_sec", 0.5)
        self.declare_parameter_if_missing("safe_max_linear_x", 0.15)
        self.declare_parameter_if_missing("safe_max_angular_z", 0.4)
        self.declare_parameter_if_missing("force_max_linear_x", 0.08)
        self.declare_parameter_if_missing("force_max_angular_z", 0.25)
        self.declare_parameter_if_missing("service_timeout_sec", 10.0)

        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("status_topic").value,
            state_qos,
        )
        self.active_pub = self.create_publisher(
            Bool,
            self.get_parameter("active_topic").value,
            state_qos,
        )
        self.safe_pub = self.create_publisher(
            Twist,
            self.get_parameter("safe_output_topic").value,
            10,
        )
        self.force_pub = self.create_publisher(
            Twist,
            self.get_parameter("force_output_topic").value,
            10,
        )
        self.safe_sub = self.create_subscription(
            Twist,
            self.get_parameter("safe_input_topic").value,
            self.safe_callback,
            10,
        )
        self.force_sub = self.create_subscription(
            Twist,
            self.get_parameter("force_input_topic").value,
            self.force_callback,
            10,
        )

        self.cancel_goal_client = self.create_client(
            CancelGoal,
            "/navigate_to_pose/_action/cancel_goal",
        )
        self.supervisor_clients = {
            "start_localization": self.create_client(
                Trigger,
                "/nav2_supervisor/start_localization",
            ),
            "start_navigation": self.create_client(
                Trigger,
                "/nav2_supervisor/start_navigation",
            ),
            "pause_navigation": self.create_client(
                Trigger,
                "/nav2_supervisor/pause_navigation",
            ),
        }

        self.lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.latest_commands = {
            self.SAFE: (Twist(), 0.0),
            self.FORCE: (Twist(), 0.0),
        }
        self.mode = self.DISABLED
        self.transitioning = False
        self.stop_mode = None
        self.stop_until = 0.0
        self.last_status = None

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.publish_state(self.DISABLED, active=False)
        self.get_logger().info(
            "Web teleop ready: safe uses Nav2 safety chain; force bypasses it"
        )

    def declare_parameter_if_missing(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def safe_callback(self, msg):
        command = clamp_twist(
            msg,
            self.get_parameter("safe_max_linear_x").value,
            self.get_parameter("safe_max_angular_z").value,
        )
        with self.lock:
            self.latest_commands[self.SAFE] = (command, time.monotonic())

    def force_callback(self, msg):
        command = clamp_twist(
            msg,
            self.get_parameter("force_max_linear_x").value,
            self.get_parameter("force_max_angular_z").value,
        )
        with self.lock:
            self.latest_commands[self.FORCE] = (command, time.monotonic())

    def timer_callback(self):
        now = time.monotonic()
        timeout = max(
            0.05,
            float(self.get_parameter("command_timeout_sec").value),
        )

        start_transition = None
        command_to_publish = None
        output_mode = None
        zero_mode = None
        publish_disabled = False

        with self.lock:
            safe_command, safe_time = self.latest_commands[self.SAFE]
            force_command, force_time = self.latest_commands[self.FORCE]
            safe_fresh = now - safe_time <= timeout
            force_fresh = now - force_time <= timeout
            desired_mode = (
                self.FORCE
                if force_fresh
                else self.SAFE
                if safe_fresh
                else self.DISABLED
            )

            if self.stop_mode is not None and now <= self.stop_until:
                zero_mode = self.stop_mode
            elif self.stop_mode is not None:
                self.stop_mode = None

            if self.transitioning:
                return_after_zero = True
            elif desired_mode == self.DISABLED and self.mode != self.DISABLED:
                zero_mode = self.mode
                self.stop_mode = self.mode
                self.stop_until = now + max(
                    0.0,
                    float(self.get_parameter("stop_publish_duration_sec").value),
                )
                self.mode = self.DISABLED
                publish_disabled = True
                return_after_zero = True
            elif desired_mode != self.DISABLED and desired_mode != self.mode:
                zero_mode = self.mode if self.mode != self.DISABLED else zero_mode
                self.transitioning = True
                start_transition = desired_mode
                return_after_zero = True
            else:
                return_after_zero = False
                if desired_mode == self.SAFE:
                    command_to_publish = safe_command
                    output_mode = self.SAFE
                elif desired_mode == self.FORCE:
                    command_to_publish = force_command
                    output_mode = self.FORCE

        if zero_mode is not None:
            self.publish_zero(zero_mode)
        if publish_disabled:
            self.publish_state(self.DISABLED, active=False)
        if start_transition is not None:
            self.publish_state(f"TRANSITIONING_{start_transition}", active=True)
            threading.Thread(
                target=self.transition_mode,
                args=(start_transition,),
                daemon=True,
            ).start()
        if return_after_zero:
            return
        if output_mode == self.SAFE:
            self.safe_pub.publish(command_to_publish)
        elif output_mode == self.FORCE:
            self.force_pub.publish(command_to_publish)

    def transition_mode(self, desired_mode):
        success = False
        error = "unknown transition error"
        try:
            if desired_mode == self.FORCE:
                # Cancellation is best-effort here. A confirmed lifecycle pause is
                # the condition that prevents Nav2 from competing on /cmd_vel.
                self.cancel_navigation_goal(required=False)
                success, error = self.call_supervisor("pause_navigation")
            else:
                success, error = self.call_supervisor("start_localization")
                if success:
                    success, error = self.call_supervisor("start_navigation")
                if success:
                    success, error = self.cancel_navigation_goal(required=True)
        except Exception as exc:  # keep malformed external state fail-closed
            error = str(exc)
            self.get_logger().error(f"Web teleop transition failed: {exc}")

        now = time.monotonic()
        timeout = max(
            0.05,
            float(self.get_parameter("command_timeout_sec").value),
        )
        with self.lock:
            _, command_time = self.latest_commands[desired_mode]
            command_still_fresh = now - command_time <= timeout
            if success and command_still_fresh and not self.shutdown_event.is_set():
                self.mode = desired_mode
                new_state = desired_mode
                active = True
            else:
                self.mode = self.DISABLED
                self.stop_mode = desired_mode
                self.stop_until = now + max(
                    0.0,
                    float(self.get_parameter("stop_publish_duration_sec").value),
                )
                new_state = (
                    self.DISABLED
                    if success and not command_still_fresh
                    else f"ERROR {desired_mode}: {error}"
                )
                active = False
            self.transitioning = False

        self.publish_zero(desired_mode)
        self.publish_state(new_state, active=active)

    def cancel_navigation_goal(self, required):
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not self.cancel_goal_client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            message = "navigate_to_pose cancel service unavailable"
            if required:
                return False, message
            self.get_logger().warn(message)
            return True, message

        result = self.call_service(
            self.cancel_goal_client,
            CancelGoal.Request(),
            timeout,
        )
        if result is None:
            message = "navigate_to_pose cancellation timed out"
            return (False, message) if required else (True, message)
        if result.return_code != CancelGoal.Response.ERROR_NONE:
            message = f"navigate_to_pose cancellation error {result.return_code}"
            return (False, message) if required else (True, message)
        return True, "navigation goals cancelled"

    def call_supervisor(self, operation):
        client = self.supervisor_clients[operation]
        timeout = float(self.get_parameter("service_timeout_sec").value)
        if not client.wait_for_service(timeout_sec=min(timeout, 2.0)):
            return False, f"supervisor {operation} service unavailable"
        result = self.call_service(client, Trigger.Request(), timeout)
        if result is None:
            return False, f"supervisor {operation} timed out"
        return bool(result.success), result.message

    def call_service(self, client, request, timeout_sec):
        future = client.call_async(request)
        completed = threading.Event()
        result_holder = {}

        def done_callback(done_future):
            try:
                result_holder["result"] = done_future.result()
            except Exception as exc:
                result_holder["exception"] = exc
            completed.set()

        future.add_done_callback(done_callback)
        completed.wait(timeout=max(0.1, float(timeout_sec)))
        if not completed.is_set():
            return None
        if "exception" in result_holder:
            self.get_logger().error(
                f"Web teleop service call failed: {result_holder['exception']}"
            )
            return None
        return result_holder.get("result")

    def publish_zero(self, mode):
        if not rclpy.ok():
            return
        if mode == self.SAFE:
            self.safe_pub.publish(Twist())
        elif mode == self.FORCE:
            self.force_pub.publish(Twist())

    def publish_state(self, state, active):
        if not rclpy.ok():
            return
        if state != self.last_status:
            status_msg = String()
            status_msg.data = state
            self.status_pub.publish(status_msg)
            self.last_status = state
            self.get_logger().info(f"Status: {state}")
        active_msg = Bool()
        active_msg.data = bool(active)
        self.active_pub.publish(active_msg)

    def destroy_node(self):
        self.shutdown_event.set()
        with self.lock:
            current_mode = self.mode
            self.mode = self.DISABLED
        self.publish_zero(current_mode)
        self.publish_state(self.DISABLED, active=False)
        super().destroy_node()


def main():
    rclpy.init()
    node = WebTeleop()
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
