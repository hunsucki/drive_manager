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


def twist_is_zero(command):
    """Return whether the supported differential-drive axes are both zero."""
    return command.linear.x == 0.0 and command.angular.z == 0.0


def mission_status_forces_safe(status):
    """Return whether a mission status must immediately disarm web teleop."""
    normalized = str(status).strip().upper()
    exact_statuses = {
        "START_MISSION",
        "START_ESCAPE",
        "RETURNING_HOME",
        "RETURNING_HOME_REQUESTED",
        "DOCKING",
        "DOCKED_NAV2_INACTIVE",
        "RESETTING_NAV2",
        "STARTING_LOCALIZATION",
        "WAITING_FOR_LOCALIZATION",
        "STARTING_NAVIGATION",
        "WAITING_FOR_ROBOT",
        "ROBOT_LOST_NAV2_STOPPING",
        "STOPPED",
        "ESTOP_LATCHED",
    }
    return (
        normalized in exact_statuses
        or normalized.startswith("DOCKING")
        or normalized.startswith("ESTOP_LATCHED ")
    )


def mission_status_blocks_force(status):
    """Return whether a new FORCE request is unsafe in the current mission state."""
    normalized = str(status).strip().upper()
    if normalized in {"STOPPED", "DOCKED_NAV2_INACTIVE"}:
        return False
    return mission_status_forces_safe(normalized)


class WebTeleop(Node):
    """Arbitrate safe teleoperation and an explicitly latched FORCE mode."""

    DISABLED = "DISABLED"
    SAFE = "SAFE"
    FORCE = "FORCE"

    def __init__(self):
        super().__init__(
            "web_teleop",
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter_if_missing("mode_request_topic", "/web_teleop/mode_request")
        self.declare_parameter_if_missing("safe_input_topic", "/cmd_vel_web_safe")
        self.declare_parameter_if_missing("force_input_topic", "/cmd_vel_web_force")
        self.declare_parameter_if_missing("safe_output_topic", "/cmd_vel_nav")
        self.declare_parameter_if_missing("force_output_topic", "/cmd_vel")
        self.declare_parameter_if_missing("status_topic", "/web_teleop/status")
        self.declare_parameter_if_missing("active_topic", "/web_teleop/active")
        self.declare_parameter_if_missing("safety_command_topic", "/robot_command")
        self.declare_parameter_if_missing("mission_status_topic", "/mission_status")
        self.declare_parameter_if_missing("drive_status_topic", "/robot_status")
        self.declare_parameter_if_missing(
            "supervisor_status_topic",
            "/nav2_supervisor/status",
        )
        self.declare_parameter_if_missing("command_timeout_sec", 0.35)
        self.declare_parameter_if_missing("stop_publish_duration_sec", 0.5)
        self.declare_parameter_if_missing("state_publish_period_sec", 1.0)
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

        self.mode_request_sub = self.create_subscription(
            String,
            self.get_parameter("mode_request_topic").value,
            self.mode_request_callback,
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
        self.safety_command_sub = self.create_subscription(
            String,
            self.get_parameter("safety_command_topic").value,
            self.safety_command_callback,
            10,
        )
        self.mission_status_sub = self.create_subscription(
            String,
            self.get_parameter("mission_status_topic").value,
            self.mission_status_callback,
            state_qos,
        )
        self.drive_status_sub = self.create_subscription(
            String,
            self.get_parameter("drive_status_topic").value,
            self.drive_status_callback,
            10,
        )
        self.supervisor_status_sub = self.create_subscription(
            String,
            self.get_parameter("supervisor_status_topic").value,
            self.supervisor_status_callback,
            state_qos,
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
        self.transition_call_lock = threading.Lock()
        self.state_publish_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.latest_commands = {
            self.SAFE: (Twist(), 0.0),
            self.FORCE: (Twist(), 0.0),
        }
        self.mode = self.DISABLED
        self.transitioning = False
        self.transition_target = None
        self.transition_generation = 0
        self.last_mission_status = ""
        self.stop_until = 0.0
        self.current_status = None
        self.current_active = None
        self.last_state_publish_time = 0.0

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.publish_zero(self.SAFE)
        self.publish_zero(self.FORCE)
        self.publish_state(self.DISABLED, active=False, force=True)
        self.get_logger().info(
            "Web teleop ready: FORCE is explicitly armed and watchdog stops "
            "motion without disarming it"
        )

    def declare_parameter_if_missing(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def mode_request_callback(self, msg):
        requested_mode = msg.data.strip().upper()
        if requested_mode == self.FORCE:
            self.request_force()
        elif requested_mode == self.SAFE:
            self.request_safe()
        else:
            self.get_logger().warn(
                f"Ignoring unknown web teleop mode request: {requested_mode}"
            )

    def request_force(self):
        transition_token = None
        blocked_status = None
        idempotent = False

        with self.lock:
            if mission_status_blocks_force(self.last_mission_status):
                blocked_status = self.last_mission_status
            elif self.mode == self.FORCE and not self.transitioning:
                idempotent = True
            elif self.transitioning and self.transition_target == self.FORCE:
                idempotent = True
            else:
                self.transition_generation += 1
                transition_token = self.transition_generation
                self.transitioning = True
                self.transition_target = self.FORCE
                self.latest_commands[self.SAFE] = (Twist(), 0.0)
                self.latest_commands[self.FORCE] = (Twist(), 0.0)
                self.begin_stop_window_locked()

        if blocked_status is not None:
            self.force_safe(
                f"FORCE_NOT_ALLOWED_{blocked_status}",
                error=True,
            )
            self.get_logger().warn(
                f"Ignoring FORCE in unsafe mission state: {blocked_status}"
            )
            return

        if idempotent:
            if self.mode == self.FORCE:
                self.publish_state(self.FORCE, active=True, force=True)
            return

        self.publish_zero(self.SAFE)
        self.publish_zero(self.FORCE)
        self.publish_state("TRANSITIONING_FORCE", active=True)
        self.start_transition(self.FORCE, transition_token)

    def request_safe(self):
        self.publish_zero(self.SAFE)
        self.publish_zero(self.FORCE)

        transition_token = None
        idempotent_state = None
        with self.lock:
            self.latest_commands[self.SAFE] = (Twist(), 0.0)
            self.latest_commands[self.FORCE] = (Twist(), 0.0)
            self.begin_stop_window_locked()

            if self.transitioning and self.transition_target == self.SAFE:
                idempotent_state = "TRANSITIONING_SAFE"
            elif self.mode == self.SAFE and not self.transitioning:
                idempotent_state = self.SAFE
            elif (
                self.mode == self.DISABLED
                and not self.transitioning
                and mission_status_blocks_force(self.last_mission_status)
            ):
                idempotent_state = self.DISABLED
            else:
                self.transition_generation += 1
                transition_token = self.transition_generation
                self.transitioning = True
                self.transition_target = self.SAFE

        if idempotent_state is not None:
            active = idempotent_state == "TRANSITIONING_SAFE"
            self.publish_state(idempotent_state, active=active, force=True)
            return

        self.publish_state("TRANSITIONING_SAFE", active=True)
        self.start_transition(self.SAFE, transition_token)

    def start_transition(self, desired_mode, transition_token):
        threading.Thread(
            target=self.transition_mode,
            args=(desired_mode, transition_token),
            daemon=True,
        ).start()

    def transition_mode(self, desired_mode, transition_token):
        success = False
        error = "unknown transition error"
        try:
            with self.transition_call_lock:
                if not self.transition_is_current(desired_mode, transition_token):
                    return

                if desired_mode == self.FORCE:
                    # A confirmed lifecycle pause is the interlock that prevents
                    # Nav2 from competing with direct FORCE output on /cmd_vel.
                    self.cancel_navigation_goal(required=False)
                    if not self.transition_is_current(desired_mode, transition_token):
                        return
                    success, error = self.call_supervisor("pause_navigation")
                else:
                    success, error = self.call_supervisor("start_localization")
                    if success and self.transition_is_current(
                        desired_mode,
                        transition_token,
                    ):
                        success, error = self.call_supervisor("start_navigation")
                    if success and self.transition_is_current(
                        desired_mode,
                        transition_token,
                    ):
                        success, error = self.cancel_navigation_goal(required=True)
        except Exception as exc:  # keep malformed external state fail-closed
            error = str(exc)
            self.get_logger().error(f"Web teleop transition failed: {exc}")

        with self.lock:
            if not self.transition_is_current_locked(desired_mode, transition_token):
                return

            if (
                desired_mode == self.FORCE
                and mission_status_blocks_force(self.last_mission_status)
            ):
                success = False
                error = f"unsafe mission state {self.last_mission_status}"

            if success and not self.shutdown_event.is_set():
                self.mode = desired_mode
                new_state = desired_mode
                active = desired_mode == self.FORCE
                self.stop_until = 0.0
            else:
                self.mode = self.DISABLED
                self.begin_stop_window_locked()
                new_state = f"ERROR {desired_mode}: {error}"
                active = False

            self.transitioning = False
            self.transition_target = None

        self.publish_zero(self.SAFE)
        self.publish_zero(self.FORCE)
        self.publish_state(new_state, active=active)

    def transition_is_current(self, desired_mode, transition_token):
        with self.lock:
            return self.transition_is_current_locked(desired_mode, transition_token)

    def transition_is_current_locked(self, desired_mode, transition_token):
        return (
            self.transitioning
            and self.transition_target == desired_mode
            and self.transition_generation == transition_token
            and not self.shutdown_event.is_set()
        )

    def safe_callback(self, msg):
        command = clamp_twist(
            msg,
            self.get_parameter("safe_max_linear_x").value,
            self.get_parameter("safe_max_angular_z").value,
        )
        with self.lock:
            if self.mode != self.SAFE or self.transitioning:
                return
            self.latest_commands[self.SAFE] = (command, time.monotonic())
        if twist_is_zero(command):
            self.publish_zero(self.SAFE)

    def force_callback(self, msg):
        command = clamp_twist(
            msg,
            self.get_parameter("force_max_linear_x").value,
            self.get_parameter("force_max_angular_z").value,
        )
        with self.lock:
            if self.mode != self.FORCE or self.transitioning:
                return
            self.latest_commands[self.FORCE] = (command, time.monotonic())
        if twist_is_zero(command):
            self.publish_zero(self.FORCE)

    def safety_command_callback(self, msg):
        command = msg.data.strip().upper()
        aliases = {
            "EMERGENCY_STOP": "ESTOP",
            "E_STOP": "ESTOP",
            "CANCEL": "STOP",
        }
        command = aliases.get(command, command)
        if command in {"ESTOP", "STOP"}:
            self.force_safe(f"COMMAND_{command}")

    def mission_status_callback(self, msg):
        status = msg.data.strip().upper()
        with self.lock:
            self.last_mission_status = status
        if mission_status_forces_safe(status):
            self.force_safe(f"MISSION_{status}", error=status.startswith("ERROR"))

    def supervisor_status_callback(self, msg):
        status = msg.data.strip().upper()
        if status.startswith("ERROR"):
            self.get_logger().warn(
                f"Nav2 supervisor reported {status}; FORCE remains available"
            )

    def drive_status_callback(self, msg):
        status = msg.data.strip().upper()
        if status.startswith("ERROR"):
            self.get_logger().warn(
                f"Drive manager reported {status}; FORCE remains available"
            )

    def force_safe(self, reason, error=False):
        with self.lock:
            self.transition_generation += 1
            self.transitioning = False
            self.transition_target = None
            self.mode = self.DISABLED
            self.latest_commands[self.SAFE] = (Twist(), 0.0)
            self.latest_commands[self.FORCE] = (Twist(), 0.0)
            self.begin_stop_window_locked()

        self.publish_zero(self.SAFE)
        self.publish_zero(self.FORCE)
        state = f"ERROR SAFETY {reason}" if error else self.DISABLED
        self.publish_state(state, active=False, force=True)
        self.get_logger().warn(f"Web teleop forced safe: {reason}")

    def begin_stop_window_locked(self):
        self.stop_until = time.monotonic() + max(
            0.0,
            float(self.get_parameter("stop_publish_duration_sec").value),
        )

    def timer_callback(self):
        now = time.monotonic()
        timeout = max(
            0.05,
            float(self.get_parameter("command_timeout_sec").value),
        )

        output_mode = None
        command_to_publish = None
        repeat_stop = False
        state = None
        active = None

        with self.lock:
            repeat_stop = now <= self.stop_until
            if self.transitioning:
                pass
            elif self.mode == self.FORCE:
                force_command, force_time = self.latest_commands[self.FORCE]
                force_fresh = now - force_time <= timeout
                command_to_publish = force_command if force_fresh else Twist()
                output_mode = self.FORCE
                state = self.FORCE
                active = True
            elif self.mode == self.SAFE:
                safe_command, safe_time = self.latest_commands[self.SAFE]
                safe_fresh = now - safe_time <= timeout
                command_to_publish = safe_command if safe_fresh else Twist()
                output_mode = self.SAFE
                state = self.SAFE
                active = safe_fresh

        if repeat_stop:
            self.publish_zero(self.SAFE)
            self.publish_zero(self.FORCE)
        elif output_mode == self.SAFE:
            self.safe_pub.publish(command_to_publish)
        elif output_mode == self.FORCE:
            self.force_pub.publish(command_to_publish)

        if state is not None:
            self.publish_state(state, active=active)
        else:
            self.republish_current_state()

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

    def publish_state(self, state, active, force=False):
        if not rclpy.ok():
            return

        now = time.monotonic()
        period = max(
            0.1,
            float(self.get_parameter("state_publish_period_sec").value),
        )
        with self.state_publish_lock:
            status_changed = state != self.current_status
            active_changed = bool(active) != self.current_active
            period_elapsed = now - self.last_state_publish_time >= period
            self.current_status = state
            self.current_active = bool(active)

            if status_changed or force or period_elapsed:
                status_msg = String()
                status_msg.data = state
                self.status_pub.publish(status_msg)
            if status_changed or active_changed or force or period_elapsed:
                active_msg = Bool()
                active_msg.data = bool(active)
                self.active_pub.publish(active_msg)
            if status_changed or active_changed or force or period_elapsed:
                self.last_state_publish_time = now

        if status_changed:
            self.get_logger().info(f"Status: {state}")

    def republish_current_state(self):
        with self.state_publish_lock:
            state = self.current_status
            active = self.current_active
        if state is not None:
            self.publish_state(state, active=active)

    def destroy_node(self):
        self.shutdown_event.set()
        self.force_safe("NODE_SHUTDOWN")
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
