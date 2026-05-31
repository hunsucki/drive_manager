#!/usr/bin/env python3

import json
import math
import queue
import shlex
import subprocess
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def calc_yaw_from_to(x_from, y_from, x_to, y_to):
    return math.atan2(y_to - y_from, x_to - x_from)


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class MissionDriver(Node):
    """Executes navigation missions received from command_manager."""

    def __init__(self):
        super().__init__(
            "mission_driver",
            automatically_declare_parameters_from_overrides=True,
        )

        self.declare_parameter_if_missing("mission_command_topic", "/mission_command")
        self.declare_parameter_if_missing("mission_status_topic", "/mission_status")
        self.declare_parameter_if_missing("route_points_topic", "/mission_route_points")
        self.declare_parameter_if_missing("route_points_publish_period_sec", 1.0)
        self.declare_parameter_if_missing("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter_if_missing("navigate_action", "/navigate_to_pose")
        self.declare_parameter_if_missing("ensure_nav2_active", True)

        self.declare_parameter_if_missing("home_to_patrol_pose", [-0.215, -0.045, 0.0])
        self.declare_parameter_if_missing(
            "home_to_dock_pose",
            [-0.215, -0.045, math.pi],
        )
        self.declare_parameter_if_missing("patrol_points", ["point_1"])
        self.declare_parameter_if_missing("patrol.point_1", [3.685, -0.045])

        self.declare_parameter_if_missing("start_escape_enabled", True)
        self.declare_parameter_if_missing("start_escape_linear_x", -0.10)
        self.declare_parameter_if_missing("start_escape_angular_z", 0.0)
        self.declare_parameter_if_missing("start_escape_duration_sec", 2.0)
        self.declare_parameter_if_missing("start_escape_stop_sec", 0.5)

        self.declare_parameter_if_missing("stop_zero_seconds", 1.5)
        self.declare_parameter_if_missing("action_server_timeout", 10.0)
        self.declare_parameter_if_missing("docking_mode", "ssh")
        self.declare_parameter_if_missing("docking_command", [""])
        self.declare_parameter_if_missing("docking_ssh_user", "pi")
        self.declare_parameter_if_missing("docking_ssh_host", "")
        self.declare_parameter_if_missing("docking_ssh_port", 22)
        self.declare_parameter_if_missing("docking_ssh_identity_file", "")
        self.declare_parameter_if_missing("docking_ssh_strict_host_key_checking", "accept-new")
        self.declare_parameter_if_missing(
            "docking_remote_setup_files",
            ["/opt/ros/jazzy/setup.bash"],
        )
        self.declare_parameter_if_missing(
            "docking_remote_command",
            "ros2 run docking dock_turn_backup",
        )
        self.declare_parameter_if_missing("docking_timeout_sec", 120.0)
        self.declare_parameter_if_missing("docking_stop_grace_sec", 3.0)

        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("mission_status_topic").value,
            10,
        )
        self.route_points_pub = self.create_publisher(
            String,
            self.get_parameter("route_points_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.get_parameter("cmd_vel_topic").value,
            10,
        )
        self.command_sub = self.create_subscription(
            String,
            self.get_parameter("mission_command_topic").value,
            self.command_callback,
            10,
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_action").value,
        )

        self.command_queue = queue.Queue()
        self.shutdown_event = threading.Event()
        self.state_lock = threading.Lock()
        self.active_goal_handle = None
        self.active_goal_label = None
        self.mission_active = False
        self.interrupt_reason = None
        self.estop_latched = False
        self.zero_until_time = 0.0
        self.last_feedback_time = 0.0

        self.zero_timer = self.create_timer(0.1, self.zero_timer_callback)
        route_points_period_sec = max(
            0.1,
            float(self.get_parameter("route_points_publish_period_sec").value),
        )
        self.route_points_timer = self.create_timer(
            route_points_period_sec,
            self.publish_route_points,
        )
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.publish_route_points()
        self.publish_status("IDLE")
        self.get_logger().info(
            f"Mission driver ready on {self.get_parameter('mission_command_topic').value}"
        )

    def declare_parameter_if_missing(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command == "RESET":
            self.estop_latched = False
            self.publish_status("IDLE")
            self.get_logger().info("ESTOP latch reset")
            return

        if command == "ESTOP":
            self.estop_latched = True
            self.clear_pending_commands()
            self.stop_robot("ESTOP")
            return

        if command == "STOP":
            self.clear_pending_commands()
            self.stop_robot("STOP")
            return

        if command not in ("START", "HOME"):
            self.publish_status(f"ERROR unknown_mission_command {command}")
            self.get_logger().warn(f"Unknown mission command: {command}")
            return

        if self.estop_latched:
            self.publish_status("ESTOP_LATCHED reset_required")
            self.get_logger().warn(f"Ignoring {command}; RESET is required")
            return

        if command == "START" and self.is_mission_active():
            self.publish_status("BUSY")
            self.get_logger().warn("Ignoring START while a mission is active")
            return

        if command == "HOME" and self.is_mission_active():
            self.clear_pending_commands()
            self.set_interrupt_reason("HOME")
            self.cancel_active_goal()
            self.publish_status("RETURNING_HOME_REQUESTED")

        self.command_queue.put(command)
        self.get_logger().info(f"Queued mission command: {command}")

    def worker_loop(self):
        while not self.shutdown_event.is_set():
            try:
                command = self.command_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                self.set_mission_active(True)
                if command == "START":
                    self.handle_start()
                elif command == "HOME":
                    self.handle_home()
            except Exception as exc:
                self.publish_status(f"ERROR {command}")
                self.get_logger().error(f"Failed to handle {command}: {exc}")
            finally:
                self.set_mission_active(False)
                self.command_queue.task_done()

    def handle_start(self):
        self.publish_status("START_MISSION")
        if not self.prepare_navigation():
            self.publish_status("ERROR nav2_not_ready")
            return

        home_to_patrol_pose = self.get_pose3_parameter("home_to_patrol_pose")
        home_to_dock_pose = self.get_pose3_parameter("home_to_dock_pose")
        patrol_points = self.get_patrol_points()
        if (
            home_to_patrol_pose is None
            or home_to_dock_pose is None
            or patrol_points is None
        ):
            self.publish_status("ERROR invalid_start_mission")
            return

        if not self.run_start_escape():
            self.publish_command_result("START", False)
            return

        self.log_start_mission(home_to_patrol_pose, patrol_points, home_to_dock_pose)

        if not self.navigate_or_abort("START", "HOME_TO_PATROL", *home_to_patrol_pose):
            return

        home_to_dock_xy = (home_to_dock_pose[0], home_to_dock_pose[1])
        for index, (name, x, y) in enumerate(patrol_points):
            if index + 1 < len(patrol_points):
                _, next_x, next_y = patrol_points[index + 1]
            else:
                next_x, next_y = home_to_dock_xy

            yaw = calc_yaw_from_to(x, y, next_x, next_y)
            label = f"PATROL_{index + 1}_{name}"
            if not self.navigate_or_abort("START", label, x, y, yaw):
                return

        if not self.navigate_or_abort("START", "HOME_TO_DOCK", *home_to_dock_pose):
            return

        if not self.run_docking_step("START"):
            self.publish_command_result("START", False)
            return

        self.publish_command_result("START", True)

    def handle_home(self):
        self.publish_status("RETURNING_HOME")
        if not self.prepare_navigation():
            self.publish_status("ERROR nav2_not_ready")
            return

        home_to_dock_pose = self.get_pose3_parameter("home_to_dock_pose")
        if home_to_dock_pose is None:
            self.publish_status("ERROR invalid_home_to_dock_pose")
            return

        if not self.navigate_or_abort("HOME", "HOME_TO_DOCK", *home_to_dock_pose):
            return

        self.publish_command_result("HOME", self.run_docking_step("HOME"))

    def navigate_or_abort(self, command, label, x, y, yaw):
        if self.has_interrupt_reason():
            self.publish_command_result(command, False)
            return False

        self.publish_status(f"NAVIGATING {label}")
        if not self.navigate_to_named_pose(label, x, y, yaw):
            self.publish_command_result(command, False)
            return False

        if self.has_interrupt_reason():
            self.publish_command_result(command, False)
            return False

        return True

    def run_start_escape(self):
        if not bool(self.get_parameter("start_escape_enabled").value):
            return True

        linear_x = float(self.get_parameter("start_escape_linear_x").value)
        angular_z = float(self.get_parameter("start_escape_angular_z").value)
        duration_sec = float(self.get_parameter("start_escape_duration_sec").value)
        stop_sec = float(self.get_parameter("start_escape_stop_sec").value)

        if duration_sec <= 0.0:
            return True

        self.publish_status("START_ESCAPE")
        self.get_logger().warn(
            "Running start escape cmd_vel before Nav2: "
            f"linear_x={linear_x:.3f}, angular_z={angular_z:.3f}, "
            f"duration={duration_sec:.2f}s"
        )

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        start_time = time.time()
        while time.time() - start_time < duration_sec:
            if self.has_interrupt_reason() or self.shutdown_event.is_set():
                self.publish_zero_velocity()
                return False
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)

        stop_start_time = time.time()
        while time.time() - stop_start_time < stop_sec:
            self.publish_zero_velocity()
            time.sleep(0.1)

        return True

    def get_patrol_points(self):
        point_names = list(self.get_parameter("patrol_points").value)
        if not point_names:
            self.get_logger().error("patrol_points is empty")
            return None

        points = []
        for point_name in point_names:
            xy = self.get_xy_parameter(f"patrol.{point_name}")
            if xy is None:
                return None
            points.append((point_name, xy[0], xy[1]))
        return points

    def get_pose3_parameter(self, name):
        if not self.has_parameter(name):
            self.declare_parameter(name, [])

        value = list(self.get_parameter(name).value)
        if len(value) != 3:
            self.get_logger().error(f"{name} must be [x, y, yaw], got {value}")
            return None

        try:
            return float(value[0]), float(value[1]), float(value[2])
        except (TypeError, ValueError):
            self.get_logger().error(f"{name} must contain numeric values: {value}")
            return None

    def get_xy_parameter(self, name):
        if not self.has_parameter(name):
            self.declare_parameter(name, [])

        value = list(self.get_parameter(name).value)
        if len(value) == 3:
            self.get_logger().warn(
                f"{name} has yaw, but patrol yaw is computed from the next target"
            )
            value = value[:2]

        if len(value) != 2:
            self.get_logger().error(f"{name} must be [x, y], got {value}")
            return None

        try:
            return float(value[0]), float(value[1])
        except (TypeError, ValueError):
            self.get_logger().error(f"{name} must contain numeric values: {value}")
            return None

    def log_start_mission(self, home_to_patrol_pose, patrol_points, home_to_dock_pose):
        self.get_logger().info(
            "START sequence: HOME_TO_PATROL -> "
            + " -> ".join(name for name, _, _ in patrol_points)
            + " -> HOME_TO_DOCK"
        )
        self.get_logger().info(
            "HOME_TO_PATROL goal: "
            f"x={home_to_patrol_pose[0]:.3f}, "
            f"y={home_to_patrol_pose[1]:.3f}, "
            f"yaw={home_to_patrol_pose[2]:.3f}"
        )
        for index, (name, x, y) in enumerate(patrol_points):
            self.get_logger().info(
                f"PATROL_{index + 1}_{name} goal: x={x:.3f}, y={y:.3f}, "
                "yaw=auto"
            )
        self.get_logger().info(
            "HOME_TO_DOCK goal: "
            f"x={home_to_dock_pose[0]:.3f}, "
            f"y={home_to_dock_pose[1]:.3f}, "
            f"yaw={home_to_dock_pose[2]:.3f}"
        )

    def build_route_points_payload(self):
        home_to_patrol_pose = self.get_pose3_parameter("home_to_patrol_pose")
        home_to_dock_pose = self.get_pose3_parameter("home_to_dock_pose")
        patrol_points = self.get_patrol_points()
        if (
            home_to_patrol_pose is None
            or home_to_dock_pose is None
            or patrol_points is None
        ):
            return None

        payload = {
            "frame_id": "map",
            "home_to_patrol_pose": self.pose3_to_dict(home_to_patrol_pose),
            "home_to_dock_pose": self.pose3_to_dict(home_to_dock_pose),
            "patrol_points": [
                {
                    "name": name,
                    "x": x,
                    "y": y,
                }
                for name, x, y in patrol_points
            ],
            "navigation_sequence": [
                {
                    "name": "HOME_TO_PATROL",
                    "type": "home",
                    **self.pose3_to_dict(home_to_patrol_pose),
                }
            ],
        }

        home_to_dock_xy = (home_to_dock_pose[0], home_to_dock_pose[1])
        for index, (name, x, y) in enumerate(patrol_points):
            if index + 1 < len(patrol_points):
                _, next_x, next_y = patrol_points[index + 1]
            else:
                next_x, next_y = home_to_dock_xy

            payload["navigation_sequence"].append(
                {
                    "name": name,
                    "type": "patrol",
                    "x": x,
                    "y": y,
                    "yaw": calc_yaw_from_to(x, y, next_x, next_y),
                    "yaw_source": "next_target",
                }
            )

        payload["navigation_sequence"].append(
            {
                "name": "HOME_TO_DOCK",
                "type": "home",
                **self.pose3_to_dict(home_to_dock_pose),
            }
        )
        return payload

    def pose3_to_dict(self, pose):
        return {
            "x": pose[0],
            "y": pose[1],
            "yaw": pose[2],
        }

    def publish_route_points(self):
        payload = self.build_route_points_payload()
        if payload is None:
            return

        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.route_points_pub.publish(msg)

    def run_docking_step(self, command):
        docking_command = self.get_docking_command()
        if not docking_command:
            self.publish_status("DOCKING_NOT_IMPLEMENTED")
            self.get_logger().warn(
                f"{command} reached docking step, but docking_command is empty."
            )
            return True

        self.publish_status("DOCKING")
        return self.run_docking_command(docking_command)

    def get_docking_command(self):
        docking_mode = str(self.get_parameter("docking_mode").value).strip().lower()
        if docking_mode == "ssh":
            return self.build_ssh_docking_command()
        if docking_mode == "custom":
            return self.get_custom_docking_command()
        if docking_mode in ("none", "placeholder", ""):
            return []

        self.get_logger().error(f"Unknown docking_mode: {docking_mode}")
        return []

    def get_custom_docking_command(self):
        value = self.get_parameter("docking_command").value
        if isinstance(value, str):
            value = shlex.split(value)

        return [str(part) for part in value if str(part).strip()]

    def build_ssh_docking_command(self):
        ssh_user = str(self.get_parameter("docking_ssh_user").value).strip()
        ssh_host = str(self.get_parameter("docking_ssh_host").value).strip()
        ssh_port = int(self.get_parameter("docking_ssh_port").value)
        identity_file = str(
            self.get_parameter("docking_ssh_identity_file").value
        ).strip()
        strict_host_key_checking = str(
            self.get_parameter("docking_ssh_strict_host_key_checking").value
        ).strip()

        if not ssh_user or not ssh_host:
            self.get_logger().error(
                "docking_mode is ssh, but docking_ssh_user or docking_ssh_host is empty"
            )
            return []

        remote_script = self.build_remote_docking_script()
        ssh_target = f"{ssh_user}@{ssh_host}"
        ssh_command = [
            "ssh",
            "-o",
            "BatchMode=yes",
            "-o",
            f"StrictHostKeyChecking={strict_host_key_checking}",
            "-p",
            str(ssh_port),
        ]

        if identity_file:
            ssh_command.extend(["-i", identity_file])

        ssh_command.extend(
            [
                ssh_target,
                "bash -lc " + shlex.quote(remote_script),
            ]
        )
        return ssh_command

    def build_remote_docking_script(self):
        setup_files = [
            str(path).strip()
            for path in self.get_parameter("docking_remote_setup_files").value
            if str(path).strip()
        ]
        remote_command = str(
            self.get_parameter("docking_remote_command").value
        ).strip()

        source_parts = [
            f"source {shlex.quote(setup_file)}"
            for setup_file in setup_files
        ]
        return " && ".join(source_parts + [f"exec {remote_command}"])

    def run_docking_command(self, docking_command):
        timeout_sec = float(self.get_parameter("docking_timeout_sec").value)
        stop_grace_sec = float(self.get_parameter("docking_stop_grace_sec").value)

        self.get_logger().info(
            "Running docking command: " + shlex.join(docking_command)
        )

        try:
            process = subprocess.Popen(docking_command)
        except OSError as exc:
            self.get_logger().error(f"Failed to start docking command: {exc}")
            return False

        start_time = time.time()
        while not self.shutdown_event.is_set():
            return_code = process.poll()
            if return_code is not None:
                if return_code == 0:
                    self.get_logger().info("Docking command succeeded")
                    return True

                self.get_logger().error(
                    f"Docking command failed with exit code {return_code}"
                )
                return False

            if self.has_interrupt_reason():
                self.get_logger().warn("Stopping docking command due to interrupt")
                self.stop_process(process, stop_grace_sec)
                return False

            if timeout_sec > 0.0 and time.time() - start_time > timeout_sec:
                self.get_logger().error(
                    f"Docking command timed out after {timeout_sec:.1f} seconds"
                )
                self.stop_process(process, stop_grace_sec)
                return False

            time.sleep(0.2)

        self.stop_process(process, stop_grace_sec)
        return False

    def stop_process(self, process, grace_sec):
        if process.poll() is not None:
            return

        process.terminate()
        try:
            process.wait(timeout=max(0.1, grace_sec))
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def publish_command_result(self, command, ok):
        if ok:
            self.publish_status(f"SUCCEEDED {command}")
            return

        reason = self.consume_interrupt_reason()
        if reason == "ESTOP":
            self.publish_status("ESTOP_LATCHED")
        elif reason == "STOP":
            self.publish_status("STOPPED")
        elif reason == "HOME":
            self.get_logger().info(f"{command} interrupted by HOME command")
        else:
            self.publish_status(f"FAILED {command}")

    def prepare_navigation(self):
        if not self.get_parameter("ensure_nav2_active").value:
            return True
        return self.ensure_navigation_active()

    def make_pose_stamped(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def navigate_to_named_pose(self, label, x, y, yaw):
        timeout = self.get_parameter("action_server_timeout").value
        if not self.nav_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("/navigate_to_pose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.make_pose_stamped(x, y, yaw)

        send_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.navigation_feedback_callback,
        )
        goal_handle = self.wait_for_future(send_future, timeout)

        if goal_handle is None:
            self.get_logger().error(f"{label} goal send timed out")
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} goal rejected")
            return False

        with self.state_lock:
            self.active_goal_handle = goal_handle
            self.active_goal_label = label

        result_future = goal_handle.get_result_async()
        result_response = self.wait_for_future(result_future, None)

        with self.state_lock:
            if self.active_goal_handle == goal_handle:
                self.active_goal_handle = None
                self.active_goal_label = None

        if result_response is None:
            self.get_logger().error(f"{label} result unavailable")
            return False

        status = result_response.status
        result = result_response.result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"{label} succeeded")
            return True

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"{label} canceled")
            return False

        self.get_logger().error(
            f"{label} failed with status={status}, "
            f"error_code={result.error_code}, error_msg={result.error_msg}"
        )
        return False

    def navigation_feedback_callback(self, feedback_msg):
        now = time.time()
        if now - self.last_feedback_time < 1.0:
            return
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f"Navigation remaining distance: {distance:.2f} m")
        self.last_feedback_time = now

    def stop_robot(self, reason):
        self.set_interrupt_reason(reason)
        self.cancel_active_goal()
        stop_seconds = self.get_parameter("stop_zero_seconds").value
        self.zero_until_time = time.time() + float(stop_seconds)
        self.publish_zero_velocity()
        self.publish_status("STOPPED" if reason == "STOP" else "ESTOP_LATCHED")
        self.get_logger().warn(f"{reason}: active goal canceled and zero velocity sent")

    def cancel_active_goal(self):
        with self.state_lock:
            goal_handle = self.active_goal_handle
            label = self.active_goal_label

        if goal_handle is None:
            return

        self.get_logger().warn(f"Canceling active goal: {label}")
        cancel_future = goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        try:
            response = future.result()
            if response is not None and len(response.goals_canceling) > 0:
                self.get_logger().info("Cancel request accepted")
            else:
                self.get_logger().warn("Cancel request returned no canceling goals")
        except Exception as exc:
            self.get_logger().error(f"Cancel request failed: {exc}")

    def has_active_goal(self):
        with self.state_lock:
            return self.active_goal_handle is not None

    def is_mission_active(self):
        with self.state_lock:
            return self.mission_active

    def set_mission_active(self, active):
        with self.state_lock:
            self.mission_active = active

    def set_interrupt_reason(self, reason):
        with self.state_lock:
            self.interrupt_reason = reason

    def has_interrupt_reason(self):
        with self.state_lock:
            return self.interrupt_reason is not None

    def consume_interrupt_reason(self):
        with self.state_lock:
            reason = self.interrupt_reason
            self.interrupt_reason = None
            return reason

    def clear_pending_commands(self):
        while True:
            try:
                self.command_queue.get_nowait()
                self.command_queue.task_done()
            except queue.Empty:
                break

    def zero_timer_callback(self):
        if time.time() < self.zero_until_time:
            self.publish_zero_velocity()

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")

    def wait_for_future(self, future, timeout_sec):
        event = threading.Event()
        result_holder = {}

        def done_callback(done_future):
            try:
                result_holder["result"] = done_future.result()
            except Exception as exc:
                result_holder["exception"] = exc
            event.set()

        future.add_done_callback(done_callback)

        if timeout_sec is None:
            while not self.shutdown_event.is_set():
                if event.wait(timeout=0.2):
                    break
        else:
            event.wait(timeout=float(timeout_sec))

        if not event.is_set():
            return None

        if "exception" in result_holder:
            raise result_holder["exception"]

        return result_holder.get("result")

    def call_service(self, client, request, timeout_sec=10.0):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None

        future = client.call_async(request)
        return self.wait_for_future(future, timeout_sec)

    def get_lifecycle_state(self, node_name):
        client = self.create_client(GetState, f"{node_name}/get_state")
        response = self.call_service(client, GetState.Request(), timeout_sec=2.0)
        self.destroy_client(client)

        if response is None:
            return None

        return response.current_state.id

    def change_lifecycle_state(self, node_name, transition_id):
        client = self.create_client(ChangeState, f"{node_name}/change_state")
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self.call_service(client, request, timeout_sec=10.0)
        self.destroy_client(client)

        return response is not None and response.success

    def activate_lifecycle_node(self, node_name):
        state = self.get_lifecycle_state(node_name)

        if state == 3:
            return True

        if state == 1:
            self.get_logger().info(f"Configuring {node_name}")
            if not self.change_lifecycle_state(
                node_name,
                Transition.TRANSITION_CONFIGURE,
            ):
                self.get_logger().warn(f"Failed to configure {node_name}")
                return False
            state = self.get_lifecycle_state(node_name)

        if state == 2:
            self.get_logger().info(f"Activating {node_name}")
            if not self.change_lifecycle_state(
                node_name,
                Transition.TRANSITION_ACTIVATE,
            ):
                self.get_logger().warn(f"Failed to activate {node_name}")
                return False

        return self.get_lifecycle_state(node_name) == 3

    def ensure_navigation_active(self):
        required_nodes = [
            "/local_costmap/local_costmap",
            "/global_costmap/global_costmap",
            "/controller_server",
            "/planner_server",
            "/smoother_server",
            "/bt_navigator",
            "/behavior_server",
            "/velocity_smoother",
            "/collision_monitor",
        ]

        inactive_nodes = [
            node_name
            for node_name in required_nodes
            if self.get_lifecycle_state(node_name) != 3
        ]

        if not inactive_nodes:
            return True

        self.get_logger().info(
            "Starting Nav2 lifecycle nodes: " + ", ".join(inactive_nodes)
        )

        manager_client = self.create_client(
            ManageLifecycleNodes,
            "/lifecycle_manager_navigation/manage_nodes",
        )
        manager_request = ManageLifecycleNodes.Request()
        manager_request.command = ManageLifecycleNodes.Request.STARTUP
        manager_response = self.call_service(
            manager_client,
            manager_request,
            timeout_sec=20.0,
        )
        self.destroy_client(manager_client)

        if manager_response is not None and manager_response.success:
            start_time = time.time()
            while time.time() - start_time < 10.0:
                if all(
                    self.get_lifecycle_state(node_name) == 3
                    for node_name in required_nodes
                ):
                    return True
                time.sleep(0.2)

        self.get_logger().warn(
            "Lifecycle manager startup did not activate every node; "
            "trying direct transitions"
        )

        failed_nodes = [
            node_name
            for node_name in required_nodes
            if not self.activate_lifecycle_node(node_name)
        ]

        if failed_nodes:
            self.get_logger().error(
                "Nav2 lifecycle nodes are not active: " + ", ".join(failed_nodes)
            )
            return False

        return True

    def destroy_node(self):
        self.shutdown_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=1.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = MissionDriver()
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
