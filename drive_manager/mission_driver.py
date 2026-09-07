#!/usr/bin/env python3

import copy
import json
import math
import os
import queue
import re
import shlex
import signal
import subprocess
import threading
import time
import uuid

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from drive_manager.capture_sync import CaptureSync, build_sync_command
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from inspection_interfaces.msg import CaptureZone
from inspection_interfaces.srv import (
    AbortCaptureRun,
    CapturePair,
    FinishCaptureRun,
    StartCaptureRun,
)
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def calc_yaw_from_to(x_from, y_from, x_to, y_to):
    return math.atan2(y_to - y_from, x_to - x_from)


def yaw_to_quaternion(yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def make_capture_zone(name, coordinates):
    """Build a rectangle from two opposite map-frame corner points."""
    try:
        values = [float(value) for value in coordinates]
    except (TypeError, ValueError) as exc:
        raise ValueError("coordinates must be numeric") from exc

    if len(values) != 4:
        raise ValueError("zone must contain two map points [x1, y1, x2, y2]")
    if not all(math.isfinite(value) for value in values):
        raise ValueError("coordinates must be finite")

    x1, y1, x2, y2 = values
    min_x, max_x = sorted((x1, x2))
    min_y, max_y = sorted((y1, y2))
    if min_x == max_x or min_y == max_y:
        raise ValueError("two points must define a rectangle with non-zero area")

    return {
        "name": str(name),
        "min_x": min_x,
        "min_y": min_y,
        "max_x": max_x,
        "max_y": max_y,
    }


def point_is_in_capture_zone(x, y, zone):
    """Return True when a map point is inside or on the rectangle boundary."""
    x = float(x)
    y = float(y)
    return (
        zone["min_x"] <= x <= zone["max_x"]
        and zone["min_y"] <= y <= zone["max_y"]
    )


def find_capture_zone(x, y, zones):
    """Return the first configured zone containing a map point."""
    for zone in zones:
        if point_is_in_capture_zone(x, y, zone):
            return zone
    return None


def capture_distance_reached(x, y, last_position, minimum_distance):
    """Return True for the first capture or after enough map-frame movement."""
    if last_position is None:
        return True
    distance = math.hypot(
        float(x) - last_position[0],
        float(y) - last_position[1],
    )
    minimum_distance = float(minimum_distance)
    return distance >= minimum_distance or math.isclose(
        distance,
        minimum_distance,
        abs_tol=1e-9,
    )


def robot_motion_allows_capture(
    linear_speed,
    angular_speed,
    minimum_linear_speed,
    maximum_angular_speed,
):
    """Allow capture triggers during translation, not in-place yaw alignment."""
    if linear_speed is None or angular_speed is None:
        return False
    values = (
        linear_speed,
        angular_speed,
        minimum_linear_speed,
        maximum_angular_speed,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return (
        float(linear_speed) >= float(minimum_linear_speed)
        and float(angular_speed) <= float(maximum_angular_speed)
    )


def normalize_capture_zone_names(zone_names):
    """Validate ordered, unique zone IDs suitable for ROS parameter names."""
    normalized = []
    seen = set()
    for raw_name in zone_names:
        name = str(raw_name).strip()
        if not name:
            raise ValueError("capture zone names must not be empty")
        if re.fullmatch(r"[A-Za-z0-9_-]+", name) is None:
            raise ValueError(
                f"invalid capture zone name {name!r}; use letters, numbers, _ or -"
            )
        if name in seen:
            raise ValueError(f"duplicate capture zone name: {name}")
        normalized.append(name)
        seen.add(name)
    return normalized


class MissionDriver(Node):
    """Executes navigation missions received from command_manager."""

    STARTUP_LOCALIZATION_COMMAND = "__STARTUP_LOCALIZATION__"
    MANUAL_INITIAL_POSE_COMMAND = "__MANUAL_INITIAL_POSE__"

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
        self.declare_parameter_if_missing("home_spin_action", "/spin")
        self.declare_parameter_if_missing(
            "home_behavior_tree",
            os.path.join(get_package_share_directory("drive_manager"),
                         "behavior_trees", "navigate_home.xml"),
        )
        self.declare_parameter_if_missing("home_arrival_xy_tolerance_m", 0.15)
        self.declare_parameter_if_missing("home_alignment_yaw_tolerance_rad", 0.0524)
        self.declare_parameter_if_missing("home_alignment_max_attempts", 3)
        self.declare_parameter_if_missing("home_alignment_timeout_sec", 15.0)
        self.declare_parameter_if_missing("home_pose_max_age_sec", 0.5)
        self.declare_parameter_if_missing("ensure_nav2_active", True)
        self.declare_parameter_if_missing(
            "start_localization_on_startup",
            True,
        )
        self.declare_parameter_if_missing(
            "manual_initial_pose_starts_navigation",
            True,
        )
        self.declare_parameter_if_missing(
            "keep_localization_active_when_idle",
            True,
        )
        self.declare_parameter_if_missing("odom_topic", "/odom")
        self.declare_parameter_if_missing("scan_topic", "/scan")
        self.declare_parameter_if_missing("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter_if_missing("initial_pose_topic", "/initialpose")
        self.declare_parameter_if_missing("robot_pose_topic", "/robot_pose")
        self.declare_parameter_if_missing("robot_pose_status_topic", "/robot_pose_status")
        self.declare_parameter_if_missing("web_teleop_active_topic", "/web_teleop/active")
        self.declare_parameter_if_missing("capture_enabled", False)
        self.declare_parameter_if_missing("capture_required", True)
        self.declare_parameter_if_missing("capture_zone_names", ["A", "B"])
        self.declare_parameter_if_missing("capture_min_distance_m", 1.0)
        self.declare_parameter_if_missing("capture_trigger_min_linear_mps", 0.02)
        self.declare_parameter_if_missing("capture_trigger_max_angular_rps", 0.15)
        self.declare_parameter_if_missing("capture_goal_exclusion_radius_m", 0.30)
        self.declare_parameter_if_missing("capture_stop_settle_sec", 0.7)
        self.declare_parameter_if_missing("capture_stop_timeout_sec", 3.0)
        self.declare_parameter_if_missing("capture_stopped_linear_mps", 0.02)
        self.declare_parameter_if_missing("capture_stopped_angular_rps", 0.05)
        self.declare_parameter_if_missing("capture_pose_timeout_sec", 2.0)
        # RTSP capture can take up to about 30 seconds when a stream is unhealthy.
        self.declare_parameter_if_missing("capture_service_timeout_sec", 35.0)
        self.declare_parameter_if_missing("capture_finish_wait_timeout_sec", 40.0)
        self.declare_parameter_if_missing("capture_failure_stops_mission", False)
        self.declare_parameter_if_missing("capture_map_id", "map_0903")
        self.declare_parameter_if_missing("capture_zone_revision", "yaml_v1")
        self.declare_parameter_if_missing(
            "capture_run_start_service",
            "/camera/capture_run/start",
        )
        self.declare_parameter_if_missing(
            "capture_pair_service",
            "/camera/capture_pair",
        )
        self.declare_parameter_if_missing(
            "capture_run_finish_service",
            "/camera/capture_run/finish",
        )
        self.declare_parameter_if_missing(
            "capture_run_abort_service",
            "/camera/capture_run/abort",
        )
        self.declare_parameter_if_missing("odom_frame", "odom")
        self.declare_parameter_if_missing("base_frame", "base_link")
        self.declare_parameter_if_missing("map_frame", "map")
        self.declare_parameter_if_missing("robot_message_timeout_sec", 2.0)
        self.declare_parameter_if_missing("robot_ready_timeout_sec", 30.0)
        self.declare_parameter_if_missing("localization_timeout_sec", 20.0)
        self.declare_parameter_if_missing("initial_pose_publish_period_sec", 0.5)
        self.declare_parameter_if_missing("initial_pose_xy_stddev", 0.25)
        self.declare_parameter_if_missing("initial_pose_yaw_stddev", 0.2618)
        self.declare_parameter_if_missing("amcl_max_xy_covariance", 0.5)
        self.declare_parameter_if_missing("amcl_max_yaw_covariance", 0.5)
        self.declare_parameter_if_missing("amcl_stable_samples", 3)
        self.declare_parameter_if_missing("assume_docked_on_start", True)
        self.declare_parameter_if_missing("reset_nav2_after_docking", True)
        self.declare_parameter_if_missing("reset_nav2_on_robot_loss", True)
        self.declare_parameter_if_missing("navigate_to_home_to_patrol_pose", False)

        self.declare_parameter_if_missing(
            "departure_initial_pose",
            [-0.215, -0.045, 0.0],
        )
        self.declare_parameter_if_missing(
            "docked_pose",
            [-0.215, -0.045, math.pi],
        )

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
        self.declare_parameter_if_missing("capture_sync_enabled", False)
        self.declare_parameter_if_missing("capture_sync_local_directory", "~/capture")
        self.declare_parameter_if_missing("capture_sync_timeout_sec", 1800.0)
        self.declare_parameter_if_missing("capture_sync_attempts", 3)
        self.declare_parameter_if_missing("capture_sync_retry_delay_sec", 10.0)
        self.declare_parameter_if_missing("supervisor_service_timeout_sec", 30.0)
        self.declare_parameter_if_missing("costmap_service_timeout_sec", 5.0)

        mission_status_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.status_pub = self.create_publisher(
            String,
            self.get_parameter("mission_status_topic").value,
            mission_status_qos,
        )
        self.capture_sync_status_pub = self.create_publisher(
            String, "/capture_sync/status", mission_status_qos,
        )
        self.route_points_pub = self.create_publisher(
            String,
            self.get_parameter("route_points_topic").value,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        pose_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.robot_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("robot_pose_topic").value,
            pose_qos,
        )
        self.robot_pose_status_pub = self.create_publisher(
            String,
            self.get_parameter("robot_pose_status_topic").value,
            pose_qos,
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            self.get_parameter("initial_pose_topic").value,
            10,
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
        self.odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self.odom_callback,
            qos_profile_sensor_data,
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("amcl_pose_topic").value,
            self.amcl_pose_callback,
            10,
        )
        self.manual_initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("initial_pose_topic").value,
            self.manual_initial_pose_callback,
            10,
        )
        self.web_teleop_active_sub = self.create_subscription(
            Bool,
            self.get_parameter("web_teleop_active_topic").value,
            self.web_teleop_active_callback,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_action").value,
        )
        self.home_spin_client = ActionClient(
            self, Spin, self.get_parameter("home_spin_action").value,
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
            "reset_all": self.create_client(
                Trigger,
                "/nav2_supervisor/reset_all",
            ),
        }
        self.costmap_clients = [
            self.create_client(
                ClearEntireCostmap,
                "/local_costmap/clear_entirely_local_costmap",
            ),
            self.create_client(
                ClearEntireCostmap,
                "/global_costmap/clear_entirely_global_costmap",
            ),
        ]
        self.capture_clients = {
            "start": self.create_client(
                StartCaptureRun,
                self.get_parameter("capture_run_start_service").value,
            ),
            "capture": self.create_client(
                CapturePair,
                self.get_parameter("capture_pair_service").value,
            ),
            "finish": self.create_client(
                FinishCaptureRun,
                self.get_parameter("capture_run_finish_service").value,
            ),
            "abort": self.create_client(
                AbortCaptureRun,
                self.get_parameter("capture_run_abort_service").value,
            ),
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.command_queue = queue.Queue()
        self.shutdown_event = threading.Event()
        self.state_lock = threading.Lock()
        self.sensor_lock = threading.Lock()
        self.capture_lock = threading.Lock()
        self.active_goal_handle = None
        self.active_goal_label = None
        self.active_goal_distance_remaining = None
        self.mission_active = False
        self.manual_initial_pose_pending = False
        self.manual_initial_pose = None
        self.interrupt_reason = None
        self.estop_latched = False
        self.zero_until_time = 0.0
        self.last_feedback_time = 0.0
        self.last_odom_received = None
        self.last_odom_linear_speed = None
        self.last_odom_angular_speed = None
        self.last_scan_received = None
        self.last_scan_frame = None
        self.last_amcl_received = None
        self.last_amcl_pose = None
        self.amcl_sequence = 0
        self.pose_mode = "UNKNOWN"
        self.nav2_expected_active = False
        self.robot_was_ready = False
        self.robot_loss_handling = False
        self.web_teleop_active = False
        self.active_mission_command = None
        self.invalid_capture_zones = []
        self.capture_zones = self.get_capture_zones()
        self.capture_min_distance_m = max(
            0.01,
            float(self.get_parameter("capture_min_distance_m").value),
        )
        self.current_capture_zone = None
        self.last_capture_position = None
        self.capture_run_active = False
        self.capture_accepting_requests = False
        self.capture_in_progress = False
        self.capture_stop_requested = False
        self.capture_stop_active = False
        self.capture_stop_zone = None
        self.capture_run_id = ""
        self.capture_mission_id = ""
        self.capture_request_sequence = 0
        self.last_capture_directory = ""

        self.zero_timer = self.create_timer(0.1, self.zero_timer_callback)
        self.robot_health_timer = self.create_timer(0.5, self.robot_health_callback)
        self.capture_timer = self.create_timer(0.1, self.capture_timer_callback)
        route_points_period_sec = max(
            0.1,
            float(self.get_parameter("route_points_publish_period_sec").value),
        )
        self.route_points_timer = self.create_timer(
            route_points_period_sec,
            self.publish_route_points,
        )
        self.capture_sync = CaptureSync(self.report_capture_sync)
        self.worker = threading.Thread(target=self.worker_loop, daemon=True)
        self.worker.start()

        self.publish_route_points()
        if bool(self.get_parameter("assume_docked_on_start").value):
            docked_pose = self.get_pose3_parameter("docked_pose")
            if docked_pose is not None:
                self.publish_fixed_robot_pose(docked_pose, "DOCKED_ASSUMED")
        self.publish_status("IDLE_NAV2_INACTIVE")
        if (
            bool(self.get_parameter("ensure_nav2_active").value)
            and bool(self.get_parameter("start_localization_on_startup").value)
        ):
            self.command_queue.put(self.STARTUP_LOCALIZATION_COMMAND)
        self.get_logger().info(
            f"Mission driver ready on {self.get_parameter('mission_command_topic').value}"
        )
        if bool(self.get_parameter("capture_enabled").value):
            self.get_logger().info(
                "Zone capture enabled: "
                f"{len(self.capture_zones)} zone(s), "
                f"distance={self.capture_min_distance_m:.2f}m, "
                f"service={self.get_parameter('capture_pair_service').value}"
            )

    def declare_parameter_if_missing(self, name, default_value):
        if not self.has_parameter(name):
            self.declare_parameter(name, default_value)

    def odom_callback(self, msg):
        linear = msg.twist.twist.linear
        angular = msg.twist.twist.angular
        with self.sensor_lock:
            self.last_odom_received = time.monotonic()
            self.last_odom_linear_speed = math.hypot(linear.x, linear.y)
            self.last_odom_angular_speed = abs(angular.z)

    def scan_callback(self, msg):
        with self.sensor_lock:
            self.last_scan_received = time.monotonic()
            self.last_scan_frame = msg.header.frame_id

    def amcl_pose_callback(self, msg):
        with self.sensor_lock:
            self.last_amcl_received = time.monotonic()
            self.last_amcl_pose = msg
            self.amcl_sequence += 1
            pose_mode = self.pose_mode

        if pose_mode != "DOCKED":
            self.robot_pose_pub.publish(msg)
            self.publish_robot_pose_status("AMCL")

    def web_teleop_active_callback(self, msg):
        self.web_teleop_active = bool(msg.data)

    def manual_initial_pose_callback(self, msg):
        if not bool(
            self.get_parameter("manual_initial_pose_starts_navigation").value
        ):
            return

        position = msg.pose.pose.position
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        initial_pose = (float(position.x), float(position.y), float(yaw))
        if not all(math.isfinite(value) for value in initial_pose):
            self.get_logger().warn("Ignoring non-finite manual initial pose")
            return

        with self.state_lock:
            if (
                self.mission_active
                or self.manual_initial_pose_pending
                or not self.command_queue.empty()
            ):
                self.get_logger().warn(
                    "Ignoring manual initial pose while another operation "
                    "is pending"
                )
                return
            self.manual_initial_pose_pending = True
            self.manual_initial_pose = initial_pose

        self.get_logger().info(
            "Queued manual initial pose: "
            f"x={initial_pose[0]:.3f}, y={initial_pose[1]:.3f}, "
            f"yaw={initial_pose[2]:.3f}"
        )
        self.command_queue.put(self.MANUAL_INITIAL_POSE_COMMAND)

    def command_callback(self, msg):
        command = msg.data.strip().upper()

        if command == "RESET":
            self.estop_latched = False
            self.consume_interrupt_reason()
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

        if self.web_teleop_active:
            self.publish_status("MANUAL_CONTROL_ACTIVE")
            self.get_logger().warn(
                f"Ignoring {command}; release web teleop controls first"
            )
            return

        if command == "START" and (
            self.is_mission_active() or self.is_manual_initial_pose_pending()
        ):
            self.publish_status("BUSY")
            self.get_logger().warn("Ignoring START while a mission is active")
            return

        if command == "START" and not self.is_mission_active():
            self.consume_interrupt_reason()

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

            tracks_mission_state = command != self.STARTUP_LOCALIZATION_COMMAND
            try:
                if tracks_mission_state:
                    self.set_mission_active(True)
                    self.set_active_mission_command(command)
                if command == "START":
                    self.handle_start()
                elif command == "HOME":
                    self.handle_home()
                elif command == self.STARTUP_LOCALIZATION_COMMAND:
                    self.handle_startup_localization()
                elif command == self.MANUAL_INITIAL_POSE_COMMAND:
                    self.handle_manual_initial_pose()
            except Exception as exc:
                self.publish_status(f"ERROR {command}")
                self.get_logger().error(f"Failed to handle {command}: {exc}")
            finally:
                if command == "START" and self.is_capture_run_active():
                    self.abort_capture_run(self.get_capture_abort_reason())
                if command == self.MANUAL_INITIAL_POSE_COMMAND:
                    self.set_manual_initial_pose_pending(False)
                if tracks_mission_state:
                    self.set_active_mission_command(None)
                    self.set_mission_active(False)
                    self.reset_capture_tracking()
                self.command_queue.task_done()

    def handle_startup_localization(self):
        self.publish_status("STARTING_LOCALIZATION")
        if not self.call_supervisor("start_localization"):
            self.publish_status("ERROR startup_localization_failed")
            return

        with self.sensor_lock:
            self.nav2_expected_active = False
        self.publish_status("WAITING_FOR_INITIAL_POSE")

    def handle_manual_initial_pose(self):
        self.publish_status("MANUAL_INITIAL_POSE_RECEIVED")
        with self.state_lock:
            initial_pose = self.manual_initial_pose

        if initial_pose is None:
            self.publish_status("ERROR invalid_manual_initial_pose")
            return

        if not self.wait_for_robot_ready():
            self.publish_status("ERROR robot_not_ready")
            return

        if not self.prepare_navigation(initial_pose=initial_pose):
            self.publish_status("ERROR manual_nav2_not_ready")
            return

        self.publish_status("MANUAL_NAV2_READY")

    def handle_start(self):
        self.publish_status("START_MISSION")
        home_to_patrol_pose = self.get_pose3_parameter("home_to_patrol_pose")
        home_to_dock_pose = self.get_pose3_parameter("home_to_dock_pose")
        departure_initial_pose = self.get_pose3_parameter("departure_initial_pose")
        patrol_points = self.get_patrol_points()
        if (
            home_to_patrol_pose is None
            or home_to_dock_pose is None
            or departure_initial_pose is None
            or patrol_points is None
        ):
            self.publish_status("ERROR invalid_start_mission")
            return

        if not self.wait_for_robot_ready():
            self.publish_status("ERROR robot_not_ready")
            return

        # Nav2 must not publish velocity while the open-loop dock escape runs.
        if not self.pause_navigation():
            self.publish_status("ERROR nav2_pause_failed")
            return

        if not self.run_start_escape():
            self.publish_command_result("START", False)
            return

        if not self.prepare_navigation(
            initial_pose=departure_initial_pose,
            force_relocalize=True,
        ):
            self.publish_status("ERROR nav2_not_ready")
            return

        camera_started = self.start_capture_run()
        if (
            bool(self.get_parameter("capture_enabled").value)
            and not camera_started
            and bool(self.get_parameter("capture_required").value)
        ):
            self.publish_status("ERROR camera_run_start_failed")
            self.publish_command_result("START", False)
            return

        self.log_start_mission(home_to_patrol_pose, patrol_points, home_to_dock_pose)

        if bool(self.get_parameter("navigate_to_home_to_patrol_pose").value):
            if not self.navigate_or_abort(
                "START",
                "HOME_TO_PATROL",
                *home_to_patrol_pose,
            ):
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

        camera_ok = self.finish_capture_run()
        if not camera_ok:
            self.publish_status("ERROR camera_run_finish_failed")
            if self.is_capture_run_active():
                self.abort_capture_run("finish_failed")

        if not self.align_home_or_abort("START", home_to_dock_pose):
            return

        if not self.pause_navigation():
            self.publish_status("ERROR nav2_pause_before_docking_failed")
            return

        if not self.run_docking_step("START"):
            self.publish_command_result("START", False)
            return

        self.finalize_docked_state()
        camera_required = bool(self.get_parameter("capture_required").value)
        self.publish_command_result("START", not camera_required or camera_ok)

    def handle_home(self):
        self.publish_status("RETURNING_HOME")
        if not self.wait_for_robot_ready():
            self.publish_status("ERROR robot_not_ready")
            return

        if not self.prepare_navigation():
            self.publish_status("ERROR nav2_not_ready")
            return

        home_to_dock_pose = self.get_pose3_parameter("home_to_dock_pose")
        if home_to_dock_pose is None:
            self.publish_status("ERROR invalid_home_to_dock_pose")
            return

        if not self.navigate_or_abort("HOME", "HOME_TO_DOCK", *home_to_dock_pose):
            return

        if not self.align_home_or_abort("HOME", home_to_dock_pose):
            return

        if not self.pause_navigation():
            self.publish_status("ERROR nav2_pause_before_docking_failed")
            return

        docking_ok = self.run_docking_step("HOME")
        if docking_ok:
            self.finalize_docked_state()
        self.publish_command_result("HOME", docking_ok)

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

    def align_home_or_abort(self, command, target_pose):
        self.publish_status("HOME_ALIGNING")
        if self.align_home_heading(target_pose):
            return True
        self.publish_zero_velocity()
        self.publish_status("ERROR home_alignment_failed")
        self.publish_command_result(command, False)
        return False

    def get_home_pose_error(self, target_pose):
        """Use fresh map->base TF, never the fixed pose published for the web."""
        transform = self.tf_buffer.lookup_transform(
            str(self.get_parameter("map_frame").value),
            str(self.get_parameter("base_frame").value),
            Time(), timeout=Duration(seconds=0.2),
        )
        age = (self.get_clock().now().nanoseconds -
               Time.from_msg(transform.header.stamp).nanoseconds) / 1e9
        max_age = float(self.get_parameter("home_pose_max_age_sec").value)
        if not math.isfinite(max_age) or max_age <= 0 or not 0 <= age <= max_age:
            raise ValueError(f"HOME pose TF is stale or future-dated: age={age:.3f}s")
        position = transform.transform.translation
        yaw = quaternion_to_yaw(transform.transform.rotation)
        distance = math.hypot(target_pose[0] - position.x, target_pose[1] - position.y)
        yaw_error = math.atan2(math.sin(target_pose[2] - yaw),
                               math.cos(target_pose[2] - yaw))
        if not all(math.isfinite(value) for value in (distance, yaw_error)):
            raise ValueError("HOME pose error is not finite")
        return distance, yaw_error

    def align_home_heading(self, target_pose):
        xy_limit = float(self.get_parameter("home_arrival_xy_tolerance_m").value)
        yaw_limit = float(self.get_parameter("home_alignment_yaw_tolerance_rad").value)
        attempts = int(self.get_parameter("home_alignment_max_attempts").value)
        if (not all(math.isfinite(v) and v > 0 for v in (xy_limit, yaw_limit))
                or attempts < 1):
            self.get_logger().error("Invalid HOME alignment tolerances or attempts")
            return False
        for attempt in range(attempts + 1):
            if self.shutdown_event.is_set() or self.has_interrupt_reason():
                return False
            if not self.wait_until_robot_stopped():
                self.get_logger().error("HOME alignment could not confirm stopped odometry")
                return False
            try:
                distance, yaw_error = self.get_home_pose_error(target_pose)
            except (TransformException, ValueError) as exc:
                self.get_logger().error(f"HOME alignment pose unavailable: {exc}")
                return False
            self.get_logger().info(
                f"HOME alignment check {attempt}: xy_error={distance:.3f}m, "
                f"yaw_error={math.degrees(yaw_error):.2f}deg"
            )
            if distance > xy_limit:
                self.get_logger().error("HOME position outside docking handoff tolerance")
                return False
            if abs(yaw_error) <= yaw_limit:
                return not self.has_interrupt_reason() and not self.shutdown_event.is_set()
            if attempt == attempts or not self.spin_home_heading(yaw_error):
                return False
        return False

    def spin_home_heading(self, yaw_error):
        """Correct the shortest yaw error through Nav2's collision-checked Spin."""
        server_timeout = float(self.get_parameter("action_server_timeout").value)
        timeout = float(self.get_parameter("home_alignment_timeout_sec").value)
        if not math.isfinite(timeout) or timeout <= 0:
            self.get_logger().error("Invalid HOME alignment timeout")
            return False
        if not self.home_spin_client.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error("HOME alignment Spin action server unavailable")
            return False
        if self.has_interrupt_reason() or self.shutdown_event.is_set():
            return False
        goal = Spin.Goal()
        goal.target_yaw = float(yaw_error)
        goal.time_allowance = Duration(seconds=timeout).to_msg()
        send_future = self.home_spin_client.send_goal_async(goal)
        handle = self.wait_for_future(send_future, server_timeout)
        if handle is None:
            # A late acceptance must not start an untracked rotation.
            def cancel_late_goal(future):
                try:
                    late_handle = future.result()
                    if late_handle is not None and late_handle.accepted:
                        late_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().error(f"Failed to cancel late HOME spin: {exc}")
            send_future.add_done_callback(cancel_late_goal)
            self.get_logger().error("HOME alignment Spin goal send timed out")
            return False
        if not handle.accepted:
            self.get_logger().error("HOME alignment Spin goal rejected")
            return False
        with self.state_lock:
            self.active_goal_handle = handle
            self.active_goal_label = "HOME_ALIGN"
        response = None
        try:
            if self.has_interrupt_reason() or self.shutdown_event.is_set():
                return False
            response = self.wait_for_future(
                handle.get_result_async(), timeout + 2.0, abort_on_interrupt=True,
            )
            if response is None:
                self.get_logger().error("HOME alignment Spin interrupted or timed out")
                return False
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"HOME alignment Spin failed: status={response.status}, "
                    f"error_code={response.result.error_code}"
                )
                return False
            return True
        finally:
            try:
                if response is None:
                    self.wait_for_future(handle.cancel_goal_async(), server_timeout)
            finally:
                with self.state_lock:
                    if self.active_goal_handle == handle:
                        self.active_goal_handle = None
                        self.active_goal_label = None
                self.publish_zero_velocity()

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

    def get_capture_zones(self):
        zones = []
        invalid_zones = []
        capture_enabled = bool(self.get_parameter("capture_enabled").value)
        try:
            zone_names = normalize_capture_zone_names(
                list(self.get_parameter("capture_zone_names").value)
            )
        except (TypeError, ValueError) as exc:
            self.invalid_capture_zones = ["capture_zone_names"]
            self.get_logger().error(f"Invalid capture_zone_names: {exc}")
            return zones

        for zone_name in zone_names:
            parameter_name = f"capture_zones.{zone_name}"
            if not self.has_parameter(parameter_name):
                self.declare_parameter(parameter_name, [0.0, 0.0, 0.0, 0.0])
            coordinates = self.get_parameter(parameter_name).value
            try:
                zones.append(make_capture_zone(zone_name, coordinates))
            except ValueError as exc:
                invalid_zones.append(zone_name)
                self.get_logger().error(
                    f"Invalid {parameter_name}: {exc}; got {coordinates}"
                )

        self.invalid_capture_zones = invalid_zones
        if capture_enabled and not zones:
            self.get_logger().info(
                "No capture zones are active; this mission will run without photos"
            )
        return zones

    def start_capture_run(self):
        if not bool(self.get_parameter("capture_enabled").value):
            return True
        if self.is_capture_run_active() and not self.abort_capture_run(
            "stale_run_before_start"
        ):
            self.get_logger().error(
                "Cannot start a camera run while the previous run is unresolved"
            )
            return False
        if self.invalid_capture_zones:
            self.get_logger().error(
                "Camera run has invalid non-empty zones: "
                + ", ".join(self.invalid_capture_zones)
            )
            return False
        if not self.capture_zones:
            self.get_logger().info("Skipping camera run because no zones are active")
            return True

        request = StartCaptureRun.Request()
        request.mission_id = f"mission_{uuid.uuid4().hex}"
        request.map_id = str(self.get_parameter("capture_map_id").value)
        request.map_frame = str(self.get_parameter("map_frame").value)
        request.zone_revision = str(
            self.get_parameter("capture_zone_revision").value
        )
        request.zones = []
        for zone in self.capture_zones:
            zone_msg = CaptureZone()
            zone_msg.id = zone["name"]
            zone_msg.min_x = zone["min_x"]
            zone_msg.min_y = zone["min_y"]
            zone_msg.max_x = zone["max_x"]
            zone_msg.max_y = zone["max_y"]
            request.zones.append(zone_msg)

        response = self.call_capture_service("start", request)
        if response is None or not response.success or not response.run_id:
            message = "no response" if response is None else response.message
            self.get_logger().error(f"Camera run start failed: {message}")
            return False

        with self.capture_lock:
            self.capture_run_active = True
            self.capture_accepting_requests = True
            self.capture_in_progress = False
            self.capture_run_id = response.run_id
            self.capture_mission_id = request.mission_id
            self.capture_request_sequence = 0
            self.current_capture_zone = None
            self.last_capture_position = None
            self.capture_stop_requested = False
            self.capture_stop_active = False
            self.capture_stop_zone = None

        self.get_logger().info(
            f"Camera run started: mission_id={request.mission_id}, "
            f"run_id={response.run_id}"
        )
        return True

    def finish_capture_run(self):
        if not bool(self.get_parameter("capture_enabled").value):
            return True

        with self.capture_lock:
            if not self.capture_run_active:
                return True
            self.capture_accepting_requests = False
            run_id = self.capture_run_id
            mission_id = self.capture_mission_id

        if not self.wait_for_capture_idle():
            self.get_logger().error(
                f"Timed out waiting for the last capture in {run_id}"
            )
            return False

        request = FinishCaptureRun.Request()
        request.run_id = run_id
        request.mission_id = mission_id
        response = self.call_capture_service("finish", request)
        if (
            response is None
            or not response.success
            or not response.ready
            or response.run_id != run_id
        ):
            message = "no response" if response is None else response.message
            self.get_logger().error(f"Camera run finish failed: {message}")
            return False

        with self.capture_lock:
            self.last_capture_directory = response.directory
            self.clear_capture_run_state_locked()

        self.get_logger().info(
            f"Camera run ready: run_id={run_id}, directory={response.directory}"
        )
        return True

    def abort_capture_run(self, reason):
        with self.capture_lock:
            if not self.capture_run_active:
                return True
            self.capture_accepting_requests = False
            run_id = self.capture_run_id
            mission_id = self.capture_mission_id

        self.wait_for_capture_idle()
        request = AbortCaptureRun.Request()
        request.run_id = run_id
        request.mission_id = mission_id
        request.reason = str(reason)
        response = self.call_capture_service("abort", request)
        success = (
            response is not None
            and response.success
            and response.run_id == run_id
        )
        if success:
            self.get_logger().info(
                f"Camera run aborted: run_id={run_id}, reason={reason}"
            )
        else:
            message = "no response" if response is None else response.message
            self.get_logger().error(f"Camera run abort failed: {message}")

        if success:
            with self.capture_lock:
                self.clear_capture_run_state_locked()
        return success

    def call_capture_service(self, operation, request):
        timeout = float(self.get_parameter("capture_service_timeout_sec").value)
        return self.call_service(
            self.capture_clients[operation],
            request,
            timeout_sec=timeout,
        )

    def wait_for_capture_idle(self):
        timeout = max(
            0.0,
            float(self.get_parameter("capture_finish_wait_timeout_sec").value),
        )
        deadline = time.monotonic() + timeout
        while not self.shutdown_event.is_set():
            with self.capture_lock:
                if not self.capture_in_progress:
                    return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.05)
        return False

    def is_capture_run_active(self):
        with self.capture_lock:
            return self.capture_run_active

    def get_capture_abort_reason(self):
        with self.state_lock:
            return self.interrupt_reason or "mission_incomplete"

    def clear_capture_run_state_locked(self):
        self.capture_run_active = False
        self.capture_accepting_requests = False
        self.capture_in_progress = False
        self.capture_run_id = ""
        self.capture_mission_id = ""
        self.capture_request_sequence = 0
        self.current_capture_zone = None
        self.last_capture_position = None
        self.capture_stop_requested = False
        self.capture_stop_active = False
        self.capture_stop_zone = None

    def capture_timer_callback(self, now_monotonic=None):
        if not bool(self.get_parameter("capture_enabled").value):
            self.reset_capture_tracking()
            return

        with self.state_lock:
            capture_mission_active = (
                self.mission_active
                and self.active_mission_command == "START"
                and self.interrupt_reason is None
            )
            active_goal_label = self.active_goal_label
            goal_distance_remaining = self.active_goal_distance_remaining
        if not capture_mission_active:
            self.reset_capture_tracking()
            return
        if active_goal_label == "HOME_TO_DOCK":
            return
        goal_exclusion_radius = max(
            0.0,
            float(self.get_parameter("capture_goal_exclusion_radius_m").value),
        )
        if (
            goal_distance_remaining is not None
            and math.isfinite(goal_distance_remaining)
            and goal_distance_remaining <= goal_exclusion_radius
        ):
            return

        now_monotonic = time.monotonic() if now_monotonic is None else float(now_monotonic)
        with self.sensor_lock:
            nav2_expected_active = self.nav2_expected_active
            pose_mode = self.pose_mode
            amcl_received = self.last_amcl_received
            amcl_pose = self.last_amcl_pose
            odom_received = self.last_odom_received
            linear_speed = self.last_odom_linear_speed
            angular_speed = self.last_odom_angular_speed

        pose_timeout = max(
            0.0,
            float(self.get_parameter("capture_pose_timeout_sec").value),
        )
        if (
            not nav2_expected_active
            or pose_mode != "AMCL"
            or amcl_pose is None
            or amcl_received is None
            or now_monotonic - amcl_received > pose_timeout
        ):
            self.reset_capture_tracking()
            return

        odom_timeout = max(
            0.0,
            float(self.get_parameter("robot_message_timeout_sec").value),
        )
        if (
            odom_received is None
            or now_monotonic - odom_received > odom_timeout
            or not robot_motion_allows_capture(
                linear_speed,
                angular_speed,
                self.get_parameter("capture_trigger_min_linear_mps").value,
                self.get_parameter("capture_trigger_max_angular_rps").value,
            )
        ):
            return

        position = amcl_pose.pose.pose.position
        x = float(position.x)
        y = float(position.y)
        if not math.isfinite(x) or not math.isfinite(y):
            self.reset_capture_tracking()
            return

        zone = find_capture_zone(x, y, self.capture_zones)
        zone_name = None if zone is None else zone["name"]
        with self.capture_lock:
            previous_zone = self.current_capture_zone
            if zone_name != previous_zone:
                self.current_capture_zone = zone_name
                self.last_capture_position = None
        if zone_name != previous_zone:
            if previous_zone is not None:
                self.get_logger().info(
                    f"Robot left capture zone: {previous_zone}"
                )
            if zone_name is not None:
                self.get_logger().info(f"Robot entered capture zone: {zone_name}")

        if zone is None:
            return

        # A moving NavigateToPose goal is canceled in a controlled way. The
        # navigation worker captures while stationary and resends the same goal.
        if not self.has_active_goal():
            return

        with self.capture_lock:
            if (
                not self.capture_run_active
                or not self.capture_accepting_requests
                or self.capture_in_progress
            ):
                return
            if not capture_distance_reached(
                x,
                y,
                self.last_capture_position,
                self.capture_min_distance_m,
            ):
                return

            self.capture_in_progress = True
            self.capture_stop_requested = True
            self.capture_stop_zone = zone_name

        self.get_logger().info(
            f"Requesting stopped capture: zone={zone_name}, x={x:.3f}, y={y:.3f}"
        )
        if not self.cancel_active_goal(context="zone capture"):
            with self.capture_lock:
                self.capture_in_progress = False
                self.capture_stop_requested = False
                self.capture_stop_zone = None
            self.get_logger().warn("Could not pause the active goal for zone capture")

    def perform_stopped_capture(self):
        with self.capture_lock:
            if not self.capture_stop_requested:
                return False
            expected_zone = self.capture_stop_zone
            self.capture_stop_requested = False
            self.capture_stop_active = True

        try:
            self.publish_status(f"CAPTURE_STOPPING zone={expected_zone}")
            if not self.wait_until_robot_stopped():
                self.handle_capture_control_failure(
                    expected_zone,
                    "robot did not stop before the capture timeout",
                )
                return False

            with self.sensor_lock:
                amcl_pose = copy.deepcopy(self.last_amcl_pose)
                amcl_received = self.last_amcl_received

            now_monotonic = time.monotonic()
            pose_timeout = max(
                0.0,
                float(self.get_parameter("capture_pose_timeout_sec").value),
            )
            if (
                amcl_pose is None
                or amcl_received is None
                or now_monotonic - amcl_received > pose_timeout
            ):
                self.handle_capture_control_failure(
                    expected_zone,
                    "AMCL pose is unavailable after stopping",
                )
                return False

            position = amcl_pose.pose.pose.position
            x = float(position.x)
            y = float(position.y)
            zone = find_capture_zone(x, y, self.capture_zones)
            actual_zone = None if zone is None else zone["name"]
            if actual_zone != expected_zone:
                self.get_logger().warn(
                    "Skipping stopped capture because the robot left the zone while "
                    f"braking: expected={expected_zone}, actual={actual_zone}"
                )
                return False

            with self.capture_lock:
                if not self.capture_run_active or not self.capture_accepting_requests:
                    return False
                self.capture_request_sequence += 1
                request_id = (
                    f"{self.capture_mission_id}_capture_"
                    f"{self.capture_request_sequence:06d}"
                )
                run_id = self.capture_run_id
                mission_id = self.capture_mission_id
                # Count an attempted capture as this distance interval so a bad
                # camera does not repeatedly stop the robot at the same position.
                self.last_capture_position = (x, y)

            request = CapturePair.Request()
            request.run_id = run_id
            request.mission_id = mission_id
            request.request_id = request_id
            request.zone_id = expected_zone
            request.requested_at = self.get_clock().now().to_msg()
            request.robot_pose = amcl_pose
            self.publish_status(f"CAPTURING zone={expected_zone}")
            return self.execute_capture_request(request)
        finally:
            with self.capture_lock:
                self.capture_stop_active = False
                self.capture_stop_requested = False
                self.capture_stop_zone = None
                self.capture_in_progress = False

    def wait_until_robot_stopped(self):
        settle_sec = max(
            0.0,
            float(self.get_parameter("capture_stop_settle_sec").value),
        )
        timeout_sec = max(
            settle_sec,
            float(self.get_parameter("capture_stop_timeout_sec").value),
        )
        linear_limit = max(
            0.0,
            float(self.get_parameter("capture_stopped_linear_mps").value),
        )
        angular_limit = max(
            0.0,
            float(self.get_parameter("capture_stopped_angular_rps").value),
        )
        odom_timeout = max(
            0.0,
            float(self.get_parameter("robot_message_timeout_sec").value),
        )
        deadline = time.monotonic() + timeout_sec
        stopped_since = None

        while not self.shutdown_event.is_set() and time.monotonic() < deadline:
            if self.has_interrupt_reason():
                return False
            self.publish_zero_velocity()
            now = time.monotonic()
            with self.sensor_lock:
                odom_received = self.last_odom_received
                linear_speed = self.last_odom_linear_speed
                angular_speed = self.last_odom_angular_speed

            odom_fresh = (
                odom_received is not None and now - odom_received <= odom_timeout
            )
            stopped = (
                odom_fresh
                and linear_speed is not None
                and angular_speed is not None
                and linear_speed <= linear_limit
                and angular_speed <= angular_limit
            )
            if stopped:
                if stopped_since is None:
                    stopped_since = now
                if now - stopped_since >= settle_sec:
                    return True
            else:
                stopped_since = None
            time.sleep(0.05)
        return False

    def handle_capture_control_failure(self, zone_id, message):
        self.get_logger().error(f"Stopped capture failed: zone={zone_id}, error={message}")
        if not bool(
            self.get_parameter("capture_failure_stops_mission").value
        ):
            return
        with self.state_lock:
            if self.interrupt_reason is None:
                self.interrupt_reason = "CAMERA_CAPTURE_FAILED"
        self.publish_status("ERROR camera_capture_failed")

    def execute_capture_request(self, request):
        failure_message = None
        try:
            response = self.call_capture_service("capture", request)
            if response is None:
                failure_message = "no response"
            elif not response.success:
                failure_message = response.message or "capture failed"
            elif response.run_id != request.run_id:
                failure_message = "response run_id does not match request"
            elif response.request_id != request.request_id:
                failure_message = "response request_id does not match request"
            else:
                self.get_logger().info(
                    "Capture saved: "
                    f"run_id={response.run_id}, capture_id={response.capture_id}, "
                    f"request_id={response.request_id}, zone={request.zone_id}"
                )
        except Exception as exc:
            failure_message = str(exc)
        finally:
            try:
                if failure_message is not None:
                    self.handle_capture_failure(request, failure_message)
            finally:
                with self.capture_lock:
                    self.capture_in_progress = False
        return failure_message is None

    def handle_capture_failure(self, request, message):
        self.get_logger().error(
            f"Capture failed: request_id={request.request_id}, "
            f"zone={request.zone_id}, error={message}"
        )
        if not bool(
            self.get_parameter("capture_failure_stops_mission").value
        ):
            return

        with self.state_lock:
            should_stop = (
                self.mission_active
                and self.active_mission_command == "START"
                and self.interrupt_reason is None
            )
            if should_stop:
                self.interrupt_reason = "CAMERA_CAPTURE_FAILED"
        if should_stop:
            self.cancel_active_goal()
            self.publish_status("ERROR camera_capture_failed")

    def reset_capture_tracking(self):
        with self.capture_lock:
            self.current_capture_zone = None
            self.last_capture_position = None

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
        include_home_to_patrol = bool(
            self.get_parameter("navigate_to_home_to_patrol_pose").value
        )
        self.get_logger().info(
            "START sequence: ESCAPE -> LOCALIZE -> "
            + ("HOME_TO_PATROL -> " if include_home_to_patrol else "")
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
        departure_initial_pose = self.get_pose3_parameter("departure_initial_pose")
        docked_pose = self.get_pose3_parameter("docked_pose")
        patrol_points = self.get_patrol_points()
        if (
            home_to_patrol_pose is None
            or home_to_dock_pose is None
            or departure_initial_pose is None
            or docked_pose is None
            or patrol_points is None
        ):
            return None

        payload = {
            "frame_id": "map",
            "home_to_patrol_pose": self.pose3_to_dict(home_to_patrol_pose),
            "home_to_dock_pose": self.pose3_to_dict(home_to_dock_pose),
            "departure_initial_pose": self.pose3_to_dict(departure_initial_pose),
            "docked_pose": self.pose3_to_dict(docked_pose),
            "patrol_points": [
                {
                    "name": name,
                    "x": x,
                    "y": y,
                }
                for name, x, y in patrol_points
            ],
            "navigation_sequence": [],
        }

        if bool(self.get_parameter("navigate_to_home_to_patrol_pose").value):
            payload["navigation_sequence"].append(
                {
                    "name": "HOME_TO_PATROL",
                    "type": "home",
                    **self.pose3_to_dict(home_to_patrol_pose),
                }
            )

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
        succeeded = self.run_docking_command(docking_command)
        if succeeded:
            self.start_capture_sync()
        return succeeded

    def start_capture_sync(self):
        if not bool(self.get_parameter("capture_sync_enabled").value):
            return
        if self.shutdown_event.is_set():
            return
        try:
            destination = os.path.expanduser(str(
                self.get_parameter("capture_sync_local_directory").value
            ))
            command = build_sync_command(
                str(self.get_parameter("docking_ssh_user").value).strip(),
                str(self.get_parameter("docking_ssh_host").value).strip(),
                int(self.get_parameter("docking_ssh_port").value),
                str(self.get_parameter("docking_ssh_identity_file").value).strip(),
                str(self.get_parameter("docking_ssh_strict_host_key_checking").value),
                destination,
            )
            self.capture_sync.request(
                command, destination,
                float(self.get_parameter("capture_sync_timeout_sec").value),
                int(self.get_parameter("capture_sync_attempts").value),
                float(self.get_parameter("capture_sync_retry_delay_sec").value),
            )
        except (ValueError, OSError) as exc:
            self.report_capture_sync("SYNC_FAILED", str(exc))

    def report_capture_sync(self, status, detail):
        msg = String()
        msg.data = status
        self.capture_sync_status_pub.publish(msg)
        logger = self.get_logger()
        log = logger.error if status == "SYNC_FAILED" else logger.info
        log(f"Capture sync {status}: {detail}")

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
            process = subprocess.Popen(docking_command, start_new_session=True)
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

        self.signal_process_group(process, signal.SIGINT)
        try:
            process.wait(timeout=max(0.1, grace_sec))
        except subprocess.TimeoutExpired:
            self.signal_process_group(process, signal.SIGTERM)
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.signal_process_group(process, signal.SIGKILL)
                process.wait(timeout=1.0)

    def signal_process_group(self, process, signal_number):
        try:
            os.killpg(process.pid, signal_number)
        except ProcessLookupError:
            pass

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

    def prepare_navigation(self, initial_pose=None, force_relocalize=False):
        if not self.get_parameter("ensure_nav2_active").value:
            return True

        if not self.robot_is_ready():
            self.get_logger().error("Robot inputs became stale before Nav2 startup")
            return False

        if force_relocalize:
            self.publish_status("RESETTING_NAV2")
            if not self.reset_nav2():
                return False

        self.publish_status("STARTING_LOCALIZATION")
        if not self.call_supervisor("start_localization"):
            return False

        with self.sensor_lock:
            self.pose_mode = "LOCALIZING"

        if initial_pose is not None:
            self.publish_status("WAITING_FOR_LOCALIZATION")
            if not self.wait_for_localization(initial_pose):
                self.get_logger().error("AMCL did not converge after the initial pose")
                return False
        elif not self.wait_for_localization(None):
            self.get_logger().error("A current AMCL pose is not available")
            return False

        self.publish_status("STARTING_NAVIGATION")
        if not self.call_supervisor("start_navigation"):
            return False

        with self.sensor_lock:
            self.nav2_expected_active = True
            self.robot_was_ready = True

        if not self.clear_costmaps():
            self.get_logger().error("Nav2 started, but costmaps could not be cleared")
            self.pause_navigation()
            return False

        self.publish_status("NAV2_READY")
        return True

    def wait_for_robot_ready(self):
        timeout = float(self.get_parameter("robot_ready_timeout_sec").value)
        deadline = time.monotonic() + timeout
        self.publish_status("WAITING_FOR_ROBOT")

        while not self.shutdown_event.is_set() and time.monotonic() < deadline:
            if self.has_interrupt_reason():
                return False
            if self.robot_is_ready():
                with self.sensor_lock:
                    self.robot_was_ready = True
                self.get_logger().info("Robot odometry, scan, and TF are ready")
                return True
            time.sleep(0.2)

        self.get_logger().error("Timed out waiting for robot odometry, scan, and TF")
        return False

    def robot_is_ready(self):
        timeout = float(self.get_parameter("robot_message_timeout_sec").value)
        now = time.monotonic()
        with self.sensor_lock:
            odom_received = self.last_odom_received
            scan_received = self.last_scan_received
            scan_frame = self.last_scan_frame

        if odom_received is None or now - odom_received > timeout:
            return False
        if scan_received is None or now - scan_received > timeout:
            return False
        if not self.transform_available(
            self.get_parameter("odom_frame").value,
            self.get_parameter("base_frame").value,
        ):
            return False
        if scan_frame and not self.transform_available(
            self.get_parameter("base_frame").value,
            scan_frame,
        ):
            return False
        return True

    def transform_available(self, target_frame, source_frame):
        try:
            return self.tf_buffer.can_transform(
                str(target_frame),
                str(source_frame),
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return False

    def wait_for_localization(self, initial_pose):
        timeout = float(self.get_parameter("localization_timeout_sec").value)
        publish_period = max(
            0.1,
            float(self.get_parameter("initial_pose_publish_period_sec").value),
        )
        required_samples = max(
            1,
            int(self.get_parameter("amcl_stable_samples").value),
        )
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        last_sequence = -1
        stable_samples = 0
        started_at = time.monotonic()

        while not self.shutdown_event.is_set() and time.monotonic() < deadline:
            if self.has_interrupt_reason() or not self.robot_is_ready():
                return False

            now = time.monotonic()
            if initial_pose is not None and now >= next_publish:
                self.publish_initial_pose(initial_pose)
                next_publish = now + publish_period

            with self.sensor_lock:
                amcl_pose = self.last_amcl_pose
                amcl_received = self.last_amcl_received
                sequence = self.amcl_sequence

            is_new_sample = sequence != last_sequence
            is_fresh = amcl_received is not None and amcl_received >= started_at
            if is_new_sample:
                last_sequence = sequence
                if (
                    is_fresh
                    and self.amcl_covariance_is_acceptable(amcl_pose)
                    and self.transform_available(
                        self.get_parameter("map_frame").value,
                        self.get_parameter("base_frame").value,
                    )
                ):
                    stable_samples += 1
                    if stable_samples >= required_samples:
                        with self.sensor_lock:
                            self.pose_mode = "AMCL"
                        self.publish_robot_pose_status("AMCL")
                        return True
                else:
                    stable_samples = 0

            time.sleep(0.1)

        return False

    def publish_initial_pose(self, pose):
        msg = self.make_pose_with_covariance(pose)
        self.initial_pose_pub.publish(msg)

    def make_pose_with_covariance(self, pose):
        x, y, yaw = pose
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = str(self.get_parameter("map_frame").value)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        qz, qw = yaw_to_quaternion(float(yaw))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        xy_stddev = float(self.get_parameter("initial_pose_xy_stddev").value)
        yaw_stddev = float(self.get_parameter("initial_pose_yaw_stddev").value)
        msg.pose.covariance[0] = xy_stddev * xy_stddev
        msg.pose.covariance[7] = xy_stddev * xy_stddev
        msg.pose.covariance[35] = yaw_stddev * yaw_stddev
        return msg

    def amcl_covariance_is_acceptable(self, msg):
        if msg is None:
            return False
        covariance = msg.pose.covariance
        max_xy = float(self.get_parameter("amcl_max_xy_covariance").value)
        max_yaw = float(self.get_parameter("amcl_max_yaw_covariance").value)
        values = (covariance[0], covariance[7], covariance[35])
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            return False
        return max(covariance[0], covariance[7]) <= max_xy and covariance[35] <= max_yaw

    def clear_costmaps(self):
        timeout = float(self.get_parameter("costmap_service_timeout_sec").value)
        for client in self.costmap_clients:
            response = self.call_service(
                client,
                ClearEntireCostmap.Request(),
                timeout_sec=timeout,
            )
            if response is None:
                return False
        return True

    def call_supervisor(self, operation):
        client = self.supervisor_clients[operation]
        timeout = float(self.get_parameter("supervisor_service_timeout_sec").value)
        response = self.call_service(client, Trigger.Request(), timeout_sec=timeout)
        if response is None:
            self.get_logger().error(f"Nav2 supervisor {operation} is unavailable")
            return False
        if not response.success:
            self.get_logger().error(
                f"Nav2 supervisor {operation} failed: {response.message}"
            )
            return False
        return True

    def pause_navigation(self):
        if not self.get_parameter("ensure_nav2_active").value:
            return True
        success = self.call_supervisor("pause_navigation")
        if success:
            with self.sensor_lock:
                self.nav2_expected_active = False
        return success

    def reset_nav2(self):
        if not self.get_parameter("ensure_nav2_active").value:
            return True
        success = self.call_supervisor("reset_all")
        if success:
            with self.sensor_lock:
                self.nav2_expected_active = False
        return success

    def finalize_docked_state(self):
        with self.sensor_lock:
            self.pose_mode = "DOCKED"
            self.nav2_expected_active = False

        if bool(self.get_parameter("reset_nav2_after_docking").value):
            if not self.reset_nav2():
                self.get_logger().error(
                    "Docking succeeded, but Nav2 could not be reset"
                )
            else:
                self.restore_idle_localization("docking")

        docked_pose = self.get_pose3_parameter("docked_pose")
        if docked_pose is not None:
            self.publish_fixed_robot_pose(docked_pose, "DOCKED")
        self.publish_status("DOCKED_NAV2_INACTIVE")

    def publish_fixed_robot_pose(self, pose, source):
        with self.sensor_lock:
            self.pose_mode = "DOCKED" if source.startswith("DOCKED") else source
        self.robot_pose_pub.publish(self.make_pose_with_covariance(pose))
        self.publish_robot_pose_status(source)

    def publish_robot_pose_status(self, status):
        msg = String()
        msg.data = status
        self.robot_pose_status_pub.publish(msg)

    def robot_health_callback(self):
        with self.sensor_lock:
            nav2_expected_active = self.nav2_expected_active
            robot_was_ready = self.robot_was_ready
            handling = self.robot_loss_handling

        if not nav2_expected_active or not robot_was_ready or handling:
            return
        if self.robot_is_ready():
            return

        with self.sensor_lock:
            self.robot_loss_handling = True
        self.set_interrupt_reason("ROBOT_LOST")
        self.cancel_active_goal()
        self.zero_until_time = time.time() + float(
            self.get_parameter("stop_zero_seconds").value
        )
        self.publish_zero_velocity()
        self.publish_status("ROBOT_LOST_NAV2_STOPPING")
        threading.Thread(
            target=self.handle_robot_loss,
            daemon=True,
        ).start()

    def handle_robot_loss(self):
        try:
            if bool(self.get_parameter("reset_nav2_on_robot_loss").value):
                if self.reset_nav2():
                    self.restore_idle_localization("robot loss")
            else:
                self.pause_navigation()
        finally:
            with self.sensor_lock:
                self.nav2_expected_active = False
                self.robot_loss_handling = False
            self.publish_status("WAITING_FOR_ROBOT")

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

        while not self.shutdown_event.is_set():
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.make_pose_stamped(x, y, yaw)
            if label == "HOME_TO_DOCK":
                goal_msg.behavior_tree = str(self.get_parameter("home_behavior_tree").value)
                if not goal_msg.behavior_tree or not os.path.isfile(goal_msg.behavior_tree):
                    self.get_logger().error("HOME behavior tree file is unavailable")
                    return False

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
                self.active_goal_distance_remaining = None

            result_future = goal_handle.get_result_async()
            result_response = self.wait_for_future(
                result_future,
                None,
                abort_on_interrupt=True,
            )

            with self.state_lock:
                if self.active_goal_handle == goal_handle:
                    self.active_goal_handle = None
                    self.active_goal_label = None
                    self.active_goal_distance_remaining = None

            if result_response is None:
                self.discard_capture_stop_request()
                if self.has_interrupt_reason():
                    self.get_logger().warn(
                        f"{label} interrupted while waiting for result"
                    )
                else:
                    self.get_logger().error(f"{label} result unavailable")
                return False

            status = result_response.status
            with self.capture_lock:
                capture_stop_requested = self.capture_stop_requested

            if (
                capture_stop_requested
                and status
                in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED)
                and not self.has_interrupt_reason()
            ):
                self.perform_stopped_capture()
                if self.has_interrupt_reason() or self.shutdown_event.is_set():
                    return False
                if status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info(f"{label} succeeded")
                    return True
                self.publish_status(f"NAVIGATING {label}")
                self.get_logger().info(
                    f"Resuming {label} after stopped zone capture"
                )
                continue

            self.discard_capture_stop_request()
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

        return False

    def navigation_feedback_callback(self, feedback_msg):
        distance = float(feedback_msg.feedback.distance_remaining)
        with self.state_lock:
            self.active_goal_distance_remaining = distance
        now = time.time()
        if now - self.last_feedback_time < 1.0:
            return
        self.get_logger().info(f"Navigation remaining distance: {distance:.2f} m")
        self.last_feedback_time = now

    def stop_robot(self, reason):
        self.set_interrupt_reason(reason)
        self.cancel_active_goal()
        stop_seconds = self.get_parameter("stop_zero_seconds").value
        self.zero_until_time = time.time() + float(stop_seconds)
        self.publish_zero_velocity()
        if reason == "STOP":
            status = "STOPPED"
        elif reason == "ESTOP":
            status = "ESTOP_LATCHED"
        else:
            status = reason
        self.publish_status(status)
        self.get_logger().warn(f"{reason}: active goal canceled and zero velocity sent")
        with self.sensor_lock:
            nav2_expected_active = self.nav2_expected_active
        if nav2_expected_active and reason in ("STOP", "ESTOP"):
            threading.Thread(target=self.pause_navigation, daemon=True).start()

    def cancel_active_goal(self, context=None):
        with self.state_lock:
            goal_handle = self.active_goal_handle
            label = self.active_goal_label

        if goal_handle is None:
            return False

        suffix = "" if context is None else f" ({context})"
        self.get_logger().warn(f"Canceling active goal: {label}{suffix}")
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception as exc:
            self.get_logger().error(f"Failed to request goal cancellation: {exc}")
            return False
        cancel_future.add_done_callback(self.cancel_done_callback)
        return True

    def discard_capture_stop_request(self):
        with self.capture_lock:
            if self.capture_stop_active:
                return
            self.capture_stop_requested = False
            self.capture_stop_zone = None
            self.capture_in_progress = False

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

    def is_manual_initial_pose_pending(self):
        with self.state_lock:
            return self.manual_initial_pose_pending

    def set_mission_active(self, active):
        with self.state_lock:
            self.mission_active = active

    def set_active_mission_command(self, command):
        with self.state_lock:
            self.active_mission_command = command

    def set_manual_initial_pose_pending(self, pending):
        with self.state_lock:
            self.manual_initial_pose_pending = pending
            if not pending:
                self.manual_initial_pose = None

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
                command = self.command_queue.get_nowait()
                if command == self.MANUAL_INITIAL_POSE_COMMAND:
                    self.set_manual_initial_pose_pending(False)
                self.command_queue.task_done()
            except queue.Empty:
                break

    def restore_idle_localization(self, context):
        if not bool(
            self.get_parameter("keep_localization_active_when_idle").value
        ):
            return True
        if self.call_supervisor("start_localization"):
            self.get_logger().info(
                f"Localization restored after {context}; "
                "waiting for initial pose"
            )
            return True

        self.get_logger().error(
            f"Failed to restore localization after {context}"
        )
        return False

    def zero_timer_callback(self):
        with self.capture_lock:
            capture_stop_active = self.capture_stop_active
        if time.time() < self.zero_until_time or capture_stop_active:
            self.publish_zero_velocity()

    def publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)
        self.get_logger().info(f"Status: {status}")

    def wait_for_future(self, future, timeout_sec, abort_on_interrupt=False):
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
                if abort_on_interrupt and self.has_interrupt_reason():
                    return None
        else:
            deadline = time.monotonic() + float(timeout_sec)
            while not event.is_set() and time.monotonic() < deadline:
                event.wait(timeout=min(0.2, max(0.0, deadline - time.monotonic())))
                if abort_on_interrupt and self.has_interrupt_reason():
                    return None

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

    def destroy_node(self):
        self.shutdown_event.set()
        self.capture_sync.close()
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
