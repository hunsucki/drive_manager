import math
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from drive_manager.mission_driver import (
    calc_yaw_from_to,
    capture_distance_reached,
    find_capture_zone,
    make_capture_zone,
    MissionDriver,
    normalize_capture_zone_names,
    point_is_in_capture_zone,
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from drive_manager.web_teleop import clamp_twist

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from inspection_interfaces.srv import (
    AbortCaptureRun,
    FinishCaptureRun,
    StartCaptureRun,
)


class MissionHelperTests(unittest.TestCase):
    def test_rectangle_capture_zone_includes_boundary(self):
        zone = make_capture_zone("A", [-1.0, -2.0, 3.0, 4.0])

        self.assertTrue(point_is_in_capture_zone(0.0, 0.0, zone))
        self.assertTrue(point_is_in_capture_zone(-1.0, 4.0, zone))
        self.assertFalse(point_is_in_capture_zone(3.01, 0.0, zone))

    def test_rectangle_capture_zone_normalizes_reverse_drag(self):
        zone = make_capture_zone("A", [3.0, 4.0, -1.0, -2.0])

        self.assertTrue(point_is_in_capture_zone(0.0, 0.0, zone))
        self.assertEqual(zone["min_x"], -1.0)
        self.assertEqual(zone["max_y"], 4.0)

    def test_overlapping_capture_zones_use_configured_priority(self):
        first = make_capture_zone("A", [0.0, 0.0, 2.0, 2.0])
        second = make_capture_zone("B", [0.0, 0.0, 3.0, 3.0])

        self.assertEqual(
            find_capture_zone(1.0, 1.0, [first, second])["name"],
            "A",
        )

    def test_invalid_capture_zone_is_rejected(self):
        with self.assertRaises(ValueError):
            make_capture_zone("too_short", [0.0, 1.0])
        with self.assertRaises(ValueError):
            make_capture_zone(
                "zero_area",
                [0.0, 0.0, 0.0, 1.0],
            )
        with self.assertRaises(ValueError):
            make_capture_zone(
                "three_points",
                [0.0, 0.0, 2.0, 0.0, 1.0, 2.0],
            )

    def test_capture_distance_is_immediate_then_uses_map_displacement(self):
        self.assertTrue(capture_distance_reached(1.0, 1.0, None, 0.5))
        self.assertFalse(capture_distance_reached(1.3, 1.4, (1.0, 1.0), 0.51))
        self.assertTrue(capture_distance_reached(1.3, 1.4, (1.0, 1.0), 0.5))

    def test_capture_zone_names_allow_any_ordered_count(self):
        self.assertEqual(
            normalize_capture_zone_names(["greenhouse_1", "north-bed", "A"]),
            ["greenhouse_1", "north-bed", "A"],
        )

    def test_capture_zone_names_reject_duplicates_and_parameter_separators(self):
        with self.assertRaises(ValueError):
            normalize_capture_zone_names(["A", "A"])
        with self.assertRaises(ValueError):
            normalize_capture_zone_names(["building.first_floor"])

    def test_capture_zones_are_loaded_from_dynamic_name_list(self):
        driver = MissionDriver.__new__(MissionDriver)
        values = {
            "capture_enabled": True,
            "capture_zone_names": ["greenhouse_1", "north-bed", "inspection_3"],
            "capture_zones.greenhouse_1": [0.0, 0.0, 1.0, 1.0],
            "capture_zones.north-bed": [1.0, 0.0, 2.0, 1.0],
            "capture_zones.inspection_3": [2.0, 0.0, 3.0, 1.0],
        }
        driver.get_parameter = Mock(
            side_effect=lambda name: SimpleNamespace(value=values[name])
        )
        driver.has_parameter = Mock(return_value=True)
        driver.declare_parameter = Mock()
        driver.get_logger = Mock(return_value=Mock())

        zones = driver.get_capture_zones()

        self.assertEqual(
            [zone["name"] for zone in zones],
            ["greenhouse_1", "north-bed", "inspection_3"],
        )
        self.assertEqual(driver.invalid_capture_zones, [])
        driver.declare_parameter.assert_not_called()

    def make_capture_driver(self, command="START", x=1.0, y=1.0):
        driver = MissionDriver.__new__(MissionDriver)
        driver.state_lock = threading.Lock()
        driver.sensor_lock = threading.Lock()
        driver.capture_lock = threading.Lock()
        driver.mission_active = True
        driver.active_mission_command = command
        driver.interrupt_reason = None
        driver.nav2_expected_active = True
        driver.pose_mode = "AMCL"
        driver.last_amcl_received = 10.0
        driver.last_amcl_pose = PoseWithCovarianceStamped()
        driver.last_amcl_pose.pose.pose.position.x = x
        driver.last_amcl_pose.pose.pose.position.y = y
        driver.last_odom_received = 10.0
        driver.last_odom_linear_speed = 0.10
        driver.last_odom_angular_speed = 0.0
        driver.capture_zones = [
            make_capture_zone("A", [0.0, 0.0, 2.0, 2.0])
        ]
        driver.capture_min_distance_m = 0.5
        driver.current_capture_zone = None
        driver.last_capture_position = None
        driver.capture_run_active = True
        driver.capture_accepting_requests = True
        driver.capture_in_progress = False
        driver.capture_stop_requested = False
        driver.capture_stop_active = False
        driver.capture_stop_zone = None
        driver.capture_run_id = "run_1"
        driver.capture_mission_id = "mission_test"
        driver.capture_request_sequence = 0
        driver.active_goal_handle = Mock()
        driver.active_goal_label = "PATROL_1"
        driver.active_goal_distance_remaining = None
        parameter_values = {
            "capture_enabled": True,
            "capture_pose_timeout_sec": 2.0,
            "robot_message_timeout_sec": 2.0,
            "capture_trigger_min_linear_mps": 0.02,
            "capture_trigger_max_angular_rps": 0.15,
            "capture_goal_exclusion_radius_m": 0.30,
        }
        driver.get_parameter = Mock(
            side_effect=lambda name: SimpleNamespace(value=parameter_values[name])
        )
        driver.get_logger = Mock(return_value=Mock())
        now = SimpleNamespace(
            nanoseconds=123,
            to_msg=Mock(return_value=SimpleNamespace(sec=1, nanosec=2)),
        )
        driver.get_clock = Mock(return_value=SimpleNamespace(now=Mock(return_value=now)))
        driver.cancel_active_goal = Mock(return_value=True)
        return driver

    def test_capture_timer_requests_stop_on_entry_then_after_distance(self):
        driver = self.make_capture_driver()

        driver.capture_timer_callback(now_monotonic=10.0)
        self.assertTrue(driver.capture_stop_requested)
        self.assertEqual(driver.capture_stop_zone, "A")
        driver.last_capture_position = (1.0, 1.0)
        driver.capture_in_progress = False
        driver.capture_stop_requested = False
        driver.capture_stop_zone = None
        driver.last_amcl_pose.pose.pose.position.x = 1.49
        driver.capture_timer_callback(now_monotonic=10.1)
        driver.last_amcl_pose.pose.pose.position.x = 1.5
        driver.capture_timer_callback(now_monotonic=10.2)

        self.assertEqual(driver.cancel_active_goal.call_count, 2)
        driver.cancel_active_goal.assert_called_with(context="zone capture")

    def test_capture_timer_reentry_triggers_immediate_capture(self):
        driver = self.make_capture_driver()
        driver.capture_timer_callback(now_monotonic=10.0)
        driver.last_capture_position = (1.0, 1.0)
        driver.capture_in_progress = False
        driver.capture_stop_requested = False
        driver.capture_stop_zone = None
        driver.last_amcl_pose.pose.pose.position.x = 3.0
        driver.capture_timer_callback(now_monotonic=10.2)
        driver.last_amcl_pose.pose.pose.position.x = 1.0
        driver.capture_timer_callback(now_monotonic=10.3)

        self.assertEqual(driver.cancel_active_goal.call_count, 2)

    def test_capture_timer_rejects_home_and_stale_pose(self):
        home_driver = self.make_capture_driver(command="HOME")
        home_driver.capture_timer_callback(now_monotonic=10.0)
        home_driver.cancel_active_goal.assert_not_called()

        stale_driver = self.make_capture_driver()
        stale_driver.capture_timer_callback(now_monotonic=12.1)
        stale_driver.cancel_active_goal.assert_not_called()

    def test_capture_timer_rejects_home_to_dock_and_waypoint_yaw_alignment(self):
        return_driver = self.make_capture_driver()
        return_driver.active_goal_label = "HOME_TO_DOCK"
        return_driver.capture_timer_callback(now_monotonic=10.0)
        return_driver.cancel_active_goal.assert_not_called()

        rotating_driver = self.make_capture_driver()
        rotating_driver.last_odom_linear_speed = 0.0
        rotating_driver.last_odom_angular_speed = 0.5
        rotating_driver.capture_timer_callback(now_monotonic=10.0)
        rotating_driver.cancel_active_goal.assert_not_called()

    def test_capture_timer_rejects_final_waypoint_alignment_radius(self):
        driver = self.make_capture_driver()
        driver.active_goal_distance_remaining = 0.1

        driver.capture_timer_callback(now_monotonic=10.0)

        driver.cancel_active_goal.assert_not_called()

    def test_stopped_capture_uses_pose_after_robot_has_settled(self):
        driver = self.make_capture_driver(x=1.0, y=1.0)
        driver.capture_in_progress = True
        driver.capture_stop_requested = True
        driver.capture_stop_zone = "A"
        driver.last_amcl_received = time.monotonic()
        driver.wait_until_robot_stopped = Mock(return_value=True)
        driver.execute_capture_request = Mock(return_value=True)
        driver.publish_status = Mock()

        self.assertTrue(driver.perform_stopped_capture())

        request = driver.execute_capture_request.call_args.args[0]
        self.assertEqual(request.run_id, "run_1")
        self.assertEqual(request.mission_id, "mission_test")
        self.assertEqual(request.request_id, "mission_test_capture_000001")
        self.assertEqual(request.zone_id, "A")
        self.assertAlmostEqual(request.robot_pose.pose.pose.position.x, 1.0)
        self.assertEqual(driver.last_capture_position, (1.0, 1.0))
        self.assertFalse(driver.capture_in_progress)
        self.assertFalse(driver.capture_stop_active)

    def test_wait_until_robot_stopped_accepts_fresh_stationary_odometry(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.shutdown_event = threading.Event()
        driver.sensor_lock = threading.Lock()
        driver.last_odom_received = time.monotonic()
        driver.last_odom_linear_speed = 0.0
        driver.last_odom_angular_speed = 0.0
        driver.has_interrupt_reason = Mock(return_value=False)
        driver.publish_zero_velocity = Mock()
        values = {
            "capture_stop_settle_sec": 0.0,
            "capture_stop_timeout_sec": 0.1,
            "capture_stopped_linear_mps": 0.02,
            "capture_stopped_angular_rps": 0.05,
            "robot_message_timeout_sec": 2.0,
        }
        driver.get_parameter = Mock(
            side_effect=lambda name: SimpleNamespace(value=values[name])
        )

        self.assertTrue(driver.wait_until_robot_stopped())
        driver.publish_zero_velocity.assert_called()

    def test_navigation_resends_same_goal_after_stopped_capture(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.shutdown_event = threading.Event()
        driver.state_lock = threading.Lock()
        driver.capture_lock = threading.Lock()
        driver.active_goal_handle = None
        driver.active_goal_label = None
        driver.active_goal_distance_remaining = None
        driver.capture_stop_requested = True
        driver.capture_stop_active = False
        driver.capture_stop_zone = "A"
        driver.capture_in_progress = True
        driver.get_parameter = Mock(return_value=SimpleNamespace(value=1.0))
        driver.get_logger = Mock(return_value=Mock())
        driver.make_pose_stamped = Mock(return_value=object())
        driver.navigation_feedback_callback = Mock()
        driver.publish_status = Mock()
        driver.has_interrupt_reason = Mock(return_value=False)

        first_goal = Mock(accepted=True)
        second_goal = Mock(accepted=True)
        first_goal.get_result_async.return_value = object()
        second_goal.get_result_async.return_value = object()
        driver.nav_client = Mock()
        driver.nav_client.wait_for_server.return_value = True
        driver.nav_client.send_goal_async.side_effect = [object(), object()]
        driver.wait_for_future = Mock(
            side_effect=[
                first_goal,
                SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=Mock()),
                second_goal,
                SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=Mock()),
            ]
        )

        def complete_capture():
            driver.capture_stop_requested = False
            driver.capture_in_progress = False
            return True

        driver.perform_stopped_capture = Mock(side_effect=complete_capture)

        self.assertTrue(driver.navigate_to_named_pose("PATROL_1", 2.0, 3.0, 0.0))
        self.assertEqual(driver.nav_client.send_goal_async.call_count, 2)
        driver.perform_stopped_capture.assert_called_once_with()

    def make_capture_run_driver(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.capture_lock = threading.Lock()
        zone_names = ("A", "B")
        driver.capture_zones = [
            make_capture_zone(name, [index, 0.0, index + 1.0, 1.0])
            for index, name in enumerate(zone_names)
        ]
        driver.invalid_capture_zones = []
        driver.capture_run_active = False
        driver.capture_accepting_requests = False
        driver.capture_in_progress = False
        driver.capture_run_id = ""
        driver.capture_mission_id = ""
        driver.capture_request_sequence = 0
        driver.current_capture_zone = None
        driver.last_capture_position = None
        driver.capture_stop_requested = False
        driver.capture_stop_active = False
        driver.capture_stop_zone = None
        driver.last_capture_directory = ""
        parameter_values = {
            "capture_enabled": True,
            "capture_map_id": "map_0903",
            "map_frame": "map",
            "capture_zone_revision": "zones_test",
        }
        driver.get_parameter = Mock(
            side_effect=lambda name: SimpleNamespace(value=parameter_values[name])
        )
        now = SimpleNamespace(nanoseconds=123456789)
        driver.get_clock = Mock(return_value=SimpleNamespace(now=Mock(return_value=now)))
        driver.get_logger = Mock(return_value=Mock())
        driver.call_capture_service = Mock()
        return driver

    def test_start_capture_run_sends_zone_snapshot_and_stores_run_id(self):
        driver = self.make_capture_run_driver()
        response = StartCaptureRun.Response()
        response.success = True
        response.run_id = "run_7"
        driver.call_capture_service.return_value = response

        self.assertTrue(driver.start_capture_run())

        operation, request = driver.call_capture_service.call_args.args
        self.assertEqual(operation, "start")
        self.assertTrue(request.mission_id.startswith("mission_"))
        self.assertGreater(len(request.mission_id), len("mission_"))
        self.assertEqual(request.map_id, "map_0903")
        self.assertEqual([zone.id for zone in request.zones], ["A", "B"])
        self.assertTrue(driver.capture_run_active)
        self.assertTrue(driver.capture_accepting_requests)
        self.assertEqual(driver.capture_run_id, "run_7")

    def test_start_capture_run_accepts_only_configured_zone_subset(self):
        driver = self.make_capture_run_driver()
        driver.capture_zones = [
            make_capture_zone("greenhouse_1", [0.0, 0.0, 1.0, 1.0]),
            make_capture_zone("north-bed", [1.0, 0.0, 2.0, 1.0]),
            make_capture_zone("inspection_3", [2.0, 0.0, 3.0, 1.0]),
        ]
        response = StartCaptureRun.Response()
        response.success = True
        response.run_id = "run_8"
        driver.call_capture_service.return_value = response

        self.assertTrue(driver.start_capture_run())

        _, request = driver.call_capture_service.call_args.args
        self.assertEqual(
            [zone.id for zone in request.zones],
            ["greenhouse_1", "north-bed", "inspection_3"],
        )

    def test_start_capture_run_rejects_invalid_non_empty_zone(self):
        driver = self.make_capture_run_driver()
        driver.capture_zones = driver.capture_zones[:2]
        driver.invalid_capture_zones = ["C"]

        self.assertFalse(driver.start_capture_run())
        driver.call_capture_service.assert_not_called()

    def test_start_capture_run_skips_service_when_zone_list_is_empty(self):
        driver = self.make_capture_run_driver()
        driver.capture_zones = []

        self.assertTrue(driver.start_capture_run())
        driver.call_capture_service.assert_not_called()
        self.assertFalse(driver.capture_run_active)

    def test_finish_capture_run_requires_ready_response(self):
        driver = self.make_capture_run_driver()
        driver.capture_run_active = True
        driver.capture_accepting_requests = True
        driver.capture_run_id = "run_7"
        driver.capture_mission_id = "mission_123"
        driver.wait_for_capture_idle = Mock(return_value=True)
        response = FinishCaptureRun.Response()
        response.success = True
        response.ready = True
        response.run_id = "run_7"
        response.directory = "/capture/20260903/run_7"
        driver.call_capture_service.return_value = response

        self.assertTrue(driver.finish_capture_run())
        self.assertFalse(driver.capture_run_active)
        self.assertEqual(
            driver.last_capture_directory,
            "/capture/20260903/run_7",
        )

    def test_abort_capture_run_closes_local_state(self):
        driver = self.make_capture_run_driver()
        driver.capture_run_active = True
        driver.capture_accepting_requests = True
        driver.capture_run_id = "run_7"
        driver.capture_mission_id = "mission_123"
        driver.wait_for_capture_idle = Mock(return_value=True)
        response = AbortCaptureRun.Response()
        response.success = True
        response.run_id = "run_7"
        driver.call_capture_service.return_value = response

        self.assertTrue(driver.abort_capture_run("STOP"))
        self.assertFalse(driver.capture_run_active)
        operation, request = driver.call_capture_service.call_args.args
        self.assertEqual(operation, "abort")
        self.assertEqual(request.reason, "STOP")

    def test_failed_abort_keeps_run_identity_for_retry(self):
        driver = self.make_capture_run_driver()
        driver.capture_run_active = True
        driver.capture_accepting_requests = True
        driver.capture_run_id = "run_7"
        driver.capture_mission_id = "mission_123"
        driver.wait_for_capture_idle = Mock(return_value=True)
        driver.call_capture_service.return_value = None

        self.assertFalse(driver.abort_capture_run("STOP"))
        self.assertTrue(driver.capture_run_active)
        self.assertFalse(driver.capture_accepting_requests)
        self.assertEqual(driver.capture_run_id, "run_7")

    def test_calc_yaw_points_toward_next_target(self):
        self.assertAlmostEqual(calc_yaw_from_to(0.0, 0.0, 1.0, 0.0), 0.0)
        self.assertAlmostEqual(
            calc_yaw_from_to(0.0, 0.0, 0.0, -1.0),
            -math.pi / 2.0,
        )

    def test_yaw_to_quaternion_for_half_turn(self):
        qz, qw = yaw_to_quaternion(math.pi)
        self.assertAlmostEqual(qz, 1.0)
        self.assertAlmostEqual(qw, 0.0, places=12)

    def test_quaternion_to_yaw_round_trip(self):
        expected_yaw = -2.392
        qz, qw = yaw_to_quaternion(expected_yaw)
        quaternion = SimpleNamespace(x=0.0, y=0.0, z=qz, w=qw)

        self.assertAlmostEqual(quaternion_to_yaw(quaternion), expected_yaw)

    def test_web_twist_is_clamped_and_non_drive_axes_are_removed(self):
        command = Twist()
        command.linear.x = 2.0
        command.linear.y = 1.0
        command.angular.x = 1.0
        command.angular.z = -3.0

        output = clamp_twist(command, 0.08, 0.25)

        self.assertAlmostEqual(output.linear.x, 0.08)
        self.assertAlmostEqual(output.linear.y, 0.0)
        self.assertAlmostEqual(output.angular.x, 0.0)
        self.assertAlmostEqual(output.angular.z, -0.25)

    def test_web_twist_rejects_non_finite_values(self):
        command = Twist()
        command.linear.x = math.nan
        command.angular.z = math.inf

        output = clamp_twist(command, 0.08, 0.25)

        self.assertAlmostEqual(output.linear.x, 0.0)
        self.assertAlmostEqual(output.angular.z, 0.0)

    def make_manual_pose_driver(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.state_lock = threading.Lock()
        driver.mission_active = False
        driver.manual_initial_pose_pending = False
        driver.manual_initial_pose = None
        driver.command_queue = queue.Queue()
        driver.get_logger = Mock(return_value=Mock())
        driver.get_parameter = Mock(
            return_value=SimpleNamespace(value=True),
        )
        return driver

    def test_manual_initial_pose_queues_navigation_preparation(self):
        driver = self.make_manual_pose_driver()
        pose = PoseWithCovarianceStamped()
        pose.pose.pose.position.x = 1.25
        pose.pose.pose.position.y = -0.75
        qz, qw = yaw_to_quaternion(-1.2)
        pose.pose.pose.orientation.z = qz
        pose.pose.pose.orientation.w = qw

        driver.manual_initial_pose_callback(pose)

        self.assertTrue(driver.manual_initial_pose_pending)
        self.assertAlmostEqual(driver.manual_initial_pose[0], 1.25)
        self.assertAlmostEqual(driver.manual_initial_pose[1], -0.75)
        self.assertAlmostEqual(driver.manual_initial_pose[2], -1.2)
        self.assertEqual(
            driver.command_queue.get_nowait(),
            MissionDriver.MANUAL_INITIAL_POSE_COMMAND,
        )

    def test_manual_initial_pose_is_ignored_during_a_mission(self):
        driver = self.make_manual_pose_driver()
        driver.mission_active = True

        driver.manual_initial_pose_callback(PoseWithCovarianceStamped())

        self.assertFalse(driver.manual_initial_pose_pending)
        self.assertIsNone(driver.manual_initial_pose)
        self.assertTrue(driver.command_queue.empty())

    def test_manual_initial_pose_prepares_nav2_without_sending_a_goal(self):
        driver = self.make_manual_pose_driver()
        driver.publish_status = Mock()
        driver.wait_for_robot_ready = Mock(return_value=True)
        driver.prepare_navigation = Mock(return_value=True)
        driver.manual_initial_pose = (1.25, -0.75, -1.2)

        driver.handle_manual_initial_pose()

        driver.wait_for_robot_ready.assert_called_once_with()
        driver.prepare_navigation.assert_called_once_with(
            initial_pose=(1.25, -0.75, -1.2)
        )
        driver.publish_status.assert_any_call("MANUAL_NAV2_READY")

    def test_prepare_navigation_republishes_manual_initial_pose(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.sensor_lock = threading.Lock()
        driver.pose_mode = "UNKNOWN"
        driver.nav2_expected_active = False
        driver.robot_was_ready = False
        driver.get_parameter = Mock(
            return_value=SimpleNamespace(value=True),
        )
        driver.robot_is_ready = Mock(return_value=True)
        driver.publish_status = Mock()
        driver.call_supervisor = Mock(return_value=True)
        driver.wait_for_localization = Mock(return_value=True)
        driver.clear_costmaps = Mock(return_value=True)

        initial_pose = (1.25, -0.75, -1.2)
        result = driver.prepare_navigation(initial_pose=initial_pose)

        self.assertTrue(result)
        driver.wait_for_localization.assert_called_once_with(initial_pose)
        self.assertEqual(driver.pose_mode, "LOCALIZING")
        self.assertTrue(driver.nav2_expected_active)

    def make_command_driver(self, web_teleop_active):
        driver = MissionDriver.__new__(MissionDriver)
        driver.state_lock = threading.Lock()
        driver.mission_active = False
        driver.manual_initial_pose_pending = False
        driver.interrupt_reason = None
        driver.estop_latched = False
        driver.web_teleop_active = web_teleop_active
        driver.command_queue = queue.Queue()
        driver.publish_status = Mock()
        driver.get_logger = Mock(return_value=Mock())
        return driver

    def test_force_active_rejects_start_and_home(self):
        for command in ("START", "HOME"):
            with self.subTest(command=command):
                driver = self.make_command_driver(web_teleop_active=True)
                message = SimpleNamespace(data=command)

                driver.command_callback(message)

                self.assertTrue(driver.command_queue.empty())
                driver.publish_status.assert_called_once_with(
                    "MANUAL_CONTROL_ACTIVE"
                )

    def test_safe_release_keeps_existing_start_queue_behavior(self):
        driver = self.make_command_driver(web_teleop_active=False)

        driver.command_callback(SimpleNamespace(data="START"))

        self.assertEqual(driver.command_queue.get_nowait(), "START")

    def test_home_always_navigates_to_staging_pose_then_docks(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.publish_status = Mock()
        driver.wait_for_robot_ready = Mock(return_value=True)
        driver.prepare_navigation = Mock(return_value=True)
        driver.get_pose3_parameter = Mock(
            return_value=(0.0129, 5.443, 1.5708)
        )
        driver.navigate_or_abort = Mock(return_value=True)
        driver.align_home_or_abort = Mock(return_value=True)
        driver.pause_navigation = Mock(return_value=True)
        driver.run_docking_step = Mock(return_value=True)
        driver.finalize_docked_state = Mock()
        driver.publish_command_result = Mock()

        driver.handle_home()

        driver.navigate_or_abort.assert_called_once_with(
            "HOME",
            "HOME_TO_DOCK",
            0.0129,
            5.443,
            1.5708,
        )
        driver.pause_navigation.assert_called_once_with()
        driver.align_home_or_abort.assert_called_once_with(
            "HOME", (0.0129, 5.443, 1.5708)
        )
        driver.run_docking_step.assert_called_once_with("HOME")
        driver.finalize_docked_state.assert_called_once_with()
        driver.publish_command_result.assert_called_once_with("HOME", True)


if __name__ == "__main__":
    unittest.main()
