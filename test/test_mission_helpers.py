import math
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from action_msgs.msg import GoalStatus
from drive_manager.mission_driver import (
    calc_yaw_from_to,
    capture_distance_reached,
    find_capture_zone,
    make_capture_zone,
    map_id_from_yaml_path,
    MissionDriver,
    normalize_capture_zone_names,
    point_is_in_capture_zone,
    quaternion_to_yaw,
    sensor_stamp_is_fresh,
    yaw_to_quaternion,
)
from drive_manager.web_teleop import clamp_twist

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from inspection_interfaces.srv import (
    AbortCaptureRun,
    CapturePair,
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

    def test_sensor_stamp_rejects_old_and_future_data(self):
        stamp = SimpleNamespace(sec=100, nanosec=0)
        self.assertTrue(sensor_stamp_is_fresh(stamp, 101.5, 2.0))
        self.assertFalse(sensor_stamp_is_fresh(stamp, 103.0, 2.0))
        self.assertFalse(sensor_stamp_is_fresh(stamp, 99.0, 2.0))

    def test_localization_waits_through_one_stale_sensor_sample(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.shutdown_event = threading.Event()
        driver.sensor_lock = threading.Lock()
        driver.robot_is_ready = Mock(side_effect=[False, True])
        driver.has_interrupt_reason = Mock(return_value=False)
        driver.publish_zero_velocity = Mock()
        driver.publish_initial_pose = Mock()
        driver.publish_robot_pose_status = Mock()
        driver.amcl_covariance_is_acceptable = Mock(return_value=True)
        driver.transform_available = Mock(return_value=True)
        driver.last_amcl_pose = PoseWithCovarianceStamped()
        driver.last_amcl_received = 10.0
        driver.amcl_sequence = 1
        values = {
            "localization_timeout_sec": 20.0,
            "initial_pose_publish_period_sec": 0.5,
            "amcl_stable_samples": 1,
            "map_frame": "map",
            "base_frame": "base_link",
        }
        driver.get_parameter = Mock(
            side_effect=lambda name: SimpleNamespace(value=values[name])
        )

        with patch("drive_manager.mission_driver.time.monotonic", return_value=10.0), \
             patch("drive_manager.mission_driver.time.sleep"):
            self.assertTrue(driver.wait_for_localization((0.0, 0.0, 0.0)))

        driver.publish_zero_velocity.assert_called_once()
        driver.publish_initial_pose.assert_called_once()

    def test_robot_loss_requires_sustained_stale_inputs(self):
        driver = self.make_capture_driver()
        driver.capture_stop_active = False
        driver.robot_was_ready = True
        driver.robot_loss_handling = False
        driver.robot_unready_since = None
        driver.robot_is_ready = Mock(side_effect=[False, False, True, False, False])
        driver.describe_navigation_inputs = Mock(return_value="scan age=2.1s")
        driver.start_robot_loss_handling = Mock()
        driver.get_parameter = Mock(
            return_value=SimpleNamespace(value=3.0)
        )

        with patch("drive_manager.mission_driver.time.monotonic",
                   side_effect=[10.0, 11.0, 12.0, 16.0]):
            for _ in range(4):
                driver.robot_health_callback()
            driver.start_robot_loss_handling.assert_not_called()
            driver.robot_health_callback()

        driver.start_robot_loss_handling.assert_called_once()

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
        driver.capture_consecutive_failures = 0
        driver.active_goal_handle = Mock()
        driver.active_goal_label = "PATROL_1"
        driver.active_goal_distance_remaining = None
        parameter_values = {
            "capture_enabled": True,
            "action_server_timeout": 1.0,
            "stop_zero_seconds": 1.5,
            "capture_pose_timeout_sec": 2.0,
            "capture_resume_timeout_sec": 10.0,
            "robot_message_timeout_sec": 2.0,
            "capture_trigger_min_linear_mps": 0.02,
            "capture_trigger_max_angular_rps": 0.15,
            "capture_goal_exclusion_radius_m": 0.30,
            "capture_failure_stops_mission": False,
            "capture_max_consecutive_failures": 1,
            "map_frame": "map",
            "base_frame": "base_link",
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
        driver.shutdown_event = threading.Event()
        driver.amcl_sequence = 0

        def publish_stationary_amcl():
            with driver.sensor_lock:
                driver.amcl_sequence += 1
                driver.last_amcl_received = time.monotonic()
            return True

        driver.request_stationary_amcl_update = Mock(side_effect=publish_stationary_amcl)
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

    def test_stale_pose_during_capture_preserves_distance_and_zone(self):
        driver = self.make_capture_driver()
        driver.current_capture_zone = "A"
        driver.last_capture_position = (1.0, 1.0)
        driver.capture_in_progress = True
        driver.last_amcl_received = 10.0

        driver.capture_timer_callback(now_monotonic=12.1)

        self.assertEqual(driver.current_capture_zone, "A")
        self.assertEqual(driver.last_capture_position, (1.0, 1.0))
        driver.capture_in_progress = False
        driver.last_amcl_received = 12.2
        driver.last_odom_received = 12.2
        driver.last_amcl_pose.pose.pose.position.x = 1.25
        driver.capture_timer_callback(now_monotonic=12.3)
        driver.cancel_active_goal.assert_not_called()

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
        driver.wait_for_capture_resume_ready = Mock(return_value=True)
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
        driver.wait_for_capture_resume_ready.assert_called_once_with()

    def test_capture_resume_accepts_stationary_pose_with_fresh_map_tf(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.last_amcl_received = time.monotonic() - 10.0
        driver.has_interrupt_reason = Mock(return_value=False)
        driver.publish_zero_velocity = Mock()
        driver.robot_is_ready = Mock(return_value=True)
        driver.amcl_covariance_is_acceptable = Mock(return_value=True)
        driver.transform_is_fresh = Mock(return_value=True)

        self.assertTrue(driver.wait_for_capture_resume_ready())
        driver.publish_zero_velocity.assert_called_once_with()
        driver.transform_is_fresh.assert_called_once_with("map", "base_link", 2.0)

    def make_stale_stopped_capture_driver(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.capture_stop_requested = True
        driver.capture_stop_zone = "A"
        driver.capture_in_progress = True
        driver.last_amcl_received = time.monotonic() - 3.0
        driver.wait_until_robot_stopped = Mock(return_value=True)
        driver.execute_capture_request = Mock()
        driver.wait_for_capture_resume_ready = Mock(return_value=True)
        driver.publish_status = Mock()
        driver.publish_zero_velocity = Mock()
        # AMCL does not answer the stationary update request.
        driver.request_stationary_amcl_update = Mock(return_value=False)
        original_get_parameter = driver.get_parameter
        driver.get_parameter = Mock(side_effect=lambda name: (
            SimpleNamespace(value=0.2) if name == "capture_pose_timeout_sec"
            else original_get_parameter(name)
        ))
        return driver

    def test_stopped_capture_refreshes_stale_pose_with_stationary_amcl_update(self):
        driver = self.make_stale_stopped_capture_driver()
        old_pose = driver.last_amcl_pose

        def publish_stopped_pose():
            pose = PoseWithCovarianceStamped()
            pose.pose.pose.position.x = 1.2
            pose.pose.pose.position.y = 0.9
            with driver.sensor_lock:
                driver.last_amcl_pose = pose
                driver.amcl_sequence += 1
                driver.last_amcl_received = time.monotonic()
            return True

        driver.request_stationary_amcl_update = Mock(side_effect=publish_stopped_pose)
        driver.execute_capture_request = Mock(return_value=True)

        self.assertTrue(driver.perform_stopped_capture())

        driver.request_stationary_amcl_update.assert_called_once_with()
        request = driver.execute_capture_request.call_args.args[0]
        self.assertAlmostEqual(request.robot_pose.pose.pose.position.x, 1.2)
        self.assertAlmostEqual(request.robot_pose.pose.pose.position.y, 0.9)
        self.assertIsNot(request.robot_pose, old_pose)
        self.assertEqual(driver.last_capture_position, (1.2, 0.9))
        self.assertEqual(driver.capture_consecutive_failures, 0)
        self.assertTrue(driver.capture_accepting_requests)

    def test_stopped_capture_falls_back_to_fresh_pose_without_amcl_update(self):
        driver = self.make_stale_stopped_capture_driver()
        driver.last_amcl_received = time.monotonic()
        driver.execute_capture_request = Mock(return_value=True)

        self.assertTrue(driver.perform_stopped_capture())

        self.assertGreaterEqual(driver.request_stationary_amcl_update.call_count, 1)
        request = driver.execute_capture_request.call_args.args[0]
        self.assertAlmostEqual(request.robot_pose.pose.pose.position.x, 1.0)
        self.assertTrue(driver.capture_accepting_requests)

    def test_stopped_capture_pose_wait_stops_on_interrupt(self):
        driver = self.make_stale_stopped_capture_driver()
        driver.request_stationary_amcl_update = Mock(
            side_effect=lambda: driver.set_interrupt_reason("STOP")
        )

        self.assertFalse(driver.perform_stopped_capture())

        driver.execute_capture_request.assert_not_called()
        driver.wait_for_capture_resume_ready.assert_not_called()
        self.assertEqual(driver.capture_consecutive_failures, 0)
        self.assertFalse(driver.capture_stop_active)

    def test_optional_stale_capture_pose_resumes_same_navigation_goal(self):
        driver = self.make_stale_stopped_capture_driver()
        driver.nav_client = Mock()
        driver.nav_client.wait_for_server.return_value = True
        driver.check_navigation_plan = Mock(return_value=True)
        driver.make_pose_stamped = Mock(return_value=PoseStamped())
        driver.navigation_feedback_callback = Mock()
        driver.wait_for_future = Mock(side_effect=[
            Mock(accepted=True),
            SimpleNamespace(status=GoalStatus.STATUS_CANCELED, result=Mock()),
            Mock(accepted=True),
            SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=Mock()),
        ])

        self.assertTrue(driver.navigate_to_named_pose("PATROL_2", 4.666, -0.023, 3.0))

        self.assertEqual(driver.nav_client.send_goal_async.call_count, 2)
        for call in driver.make_pose_stamped.call_args_list:
            self.assertEqual(call.args, (4.666, -0.023, 3.0))
        driver.execute_capture_request.assert_not_called()
        driver.wait_for_capture_resume_ready.assert_called_once_with()
        self.assertIsNone(driver.interrupt_reason)
        self.assertFalse(driver.capture_accepting_requests)
        self.assertFalse(driver.capture_in_progress)
        self.assertFalse(driver.capture_stop_active)
        driver.publish_status.assert_any_call("NAVIGATING PATROL_2")

    def test_optional_missing_capture_pose_still_waits_for_sensors(self):
        driver = self.make_stale_stopped_capture_driver()
        driver.last_amcl_pose = None
        driver.last_amcl_received = None
        driver.wait_for_capture_resume_ready.return_value = False
        driver.robot_is_ready = Mock(return_value=False)
        driver.start_robot_loss_handling = Mock(
            side_effect=lambda: driver.set_interrupt_reason("ROBOT_LOST")
        )

        self.assertFalse(driver.perform_stopped_capture())

        driver.execute_capture_request.assert_not_called()
        driver.start_robot_loss_handling.assert_called_once_with()
        self.assertEqual(driver.interrupt_reason, "ROBOT_LOST")
        self.assertFalse(driver.capture_stop_active)

    def test_required_capture_pose_failure_still_aborts(self):
        driver = self.make_stale_stopped_capture_driver()
        original_get_parameter = driver.get_parameter
        driver.get_parameter = Mock(side_effect=lambda name: (
            SimpleNamespace(value=True) if name == "capture_failure_stops_mission"
            else original_get_parameter(name)
        ))

        self.assertFalse(driver.perform_stopped_capture())

        self.assertEqual(driver.interrupt_reason, "CAPTURE_CONTROL_FAILED")
        driver.execute_capture_request.assert_not_called()
        driver.wait_for_capture_resume_ready.assert_not_called()
        driver.publish_status.assert_any_call("ERROR capture_control_failed")

    def test_capture_service_exceptions_are_reported_as_failed_responses(self):
        driver = self.make_capture_driver()
        driver.get_parameter = Mock(return_value=SimpleNamespace(value=0.1))
        driver.capture_clients = {op: Mock() for op in ("start", "capture", "finish", "abort")}
        driver.call_service = Mock(side_effect=RuntimeError("camera disconnected"))
        for operation in driver.capture_clients:
            with self.subTest(operation=operation):
                self.assertIsNone(driver.call_capture_service(operation, object()))
        self.assertIsNone(driver.interrupt_reason)

    def test_capture_resume_refreshes_stationary_amcl_covariance(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.publish_zero_velocity = Mock()
        driver.robot_is_ready = Mock(return_value=True)
        driver.amcl_covariance_is_acceptable = Mock(side_effect=[False, True])
        driver.transform_is_fresh = Mock(return_value=True)
        driver.request_stationary_amcl_update = Mock(return_value=True)

        with patch("drive_manager.mission_driver.time.sleep"):
            self.assertTrue(driver.wait_for_capture_resume_ready())

        driver.request_stationary_amcl_update.assert_called_once_with()
        self.assertIsNone(driver.interrupt_reason)

    def test_capture_resume_refreshes_stale_map_tf(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.publish_zero_velocity = Mock()
        driver.robot_is_ready = Mock(return_value=True)
        driver.amcl_covariance_is_acceptable = Mock(return_value=True)
        driver.transform_is_fresh = Mock(side_effect=[False, True])
        driver.request_stationary_amcl_update = Mock(return_value=True)

        with patch("drive_manager.mission_driver.time.sleep"):
            self.assertTrue(driver.wait_for_capture_resume_ready())

        driver.request_stationary_amcl_update.assert_called_once_with()

    def test_capture_resume_timeout_reports_the_failing_condition(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.publish_zero_velocity = Mock()
        driver.robot_is_ready = Mock(return_value=True)
        driver.amcl_covariance_is_acceptable = Mock(return_value=False)
        driver.describe_navigation_inputs = Mock(return_value="amcl_covariance=(0.8, 0.6, 0.7)")
        driver.request_stationary_amcl_update = Mock()
        with patch("drive_manager.mission_driver.time.monotonic", side_effect=[0.0, 11.0]):
            self.assertFalse(driver.wait_for_capture_resume_ready())

        error = driver.get_logger().error.call_args.args[0]
        self.assertIn("covariance is not acceptable", error)
        self.assertIn("amcl_covariance=(0.8, 0.6, 0.7)", error)
        driver.request_stationary_amcl_update.assert_not_called()

    def test_capture_resume_localization_failure_preserves_amcl(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.capture_stop_requested = True
        driver.capture_stop_zone = "A"
        driver.last_amcl_received = time.monotonic()
        driver.wait_until_robot_stopped = Mock(return_value=True)
        driver.execute_capture_request = Mock(return_value=False)
        driver.wait_for_capture_resume_ready = Mock(return_value=False)
        driver.robot_is_ready = Mock(return_value=True)
        driver.publish_status = Mock()
        driver.handle_capture_control_failure = Mock()
        driver.pause_navigation = Mock(return_value=True)
        driver.start_robot_loss_handling = Mock()

        self.assertFalse(driver.perform_stopped_capture())

        driver.pause_navigation.assert_called_once_with()
        driver.start_robot_loss_handling.assert_not_called()
        driver.handle_capture_control_failure.assert_called_once()
        self.assertFalse(driver.capture_stop_active)

    def test_capture_resume_sensor_failure_still_stops_robot(self):
        driver = self.make_capture_driver()
        driver.shutdown_event = threading.Event()
        driver.capture_stop_requested = True
        driver.capture_stop_zone = "A"
        driver.last_amcl_received = time.monotonic()
        driver.wait_until_robot_stopped = Mock(return_value=True)
        driver.execute_capture_request = Mock(return_value=False)
        driver.wait_for_capture_resume_ready = Mock(return_value=False)
        driver.robot_is_ready = Mock(return_value=False)
        driver.publish_status = Mock()
        driver.pause_navigation = Mock()
        driver.start_robot_loss_handling = Mock()

        self.assertFalse(driver.perform_stopped_capture())

        driver.start_robot_loss_handling.assert_called_once_with()
        driver.pause_navigation.assert_not_called()

    def test_optional_camera_failure_does_not_stop_again_in_next_zone(self):
        driver = self.make_capture_driver()
        request = SimpleNamespace(request_id="request_1", zone_id="A")

        driver.handle_capture_failure(request, "capture pair incomplete")
        driver.capture_timer_callback(now_monotonic=10.0)

        self.assertFalse(driver.capture_accepting_requests)
        self.assertIsNone(driver.interrupt_reason)
        driver.cancel_active_goal.assert_not_called()

    def test_successful_capture_resets_consecutive_camera_failures(self):
        driver = self.make_capture_driver()
        driver.capture_consecutive_failures = 1
        request = SimpleNamespace(run_id="run_1", request_id="request_2", zone_id="A")
        response = CapturePair.Response()
        response.success = True
        response.run_id = request.run_id
        response.request_id = request.request_id
        driver.call_capture_service = Mock(return_value=response)

        self.assertTrue(driver.execute_capture_request(request))
        self.assertEqual(driver.capture_consecutive_failures, 0)

    def test_one_camera_keeps_saving_across_capture_requests(self):
        for side in ("left", "right"):
            with self.subTest(saved_side=side):
                driver = self.make_capture_driver()
                driver.capture_consecutive_failures = 1
                for index in range(2):
                    request = SimpleNamespace(
                        run_id="run_1", request_id=f"request_{index}", zone_id="A"
                    )
                    response = CapturePair.Response()
                    response.success = False
                    response.message = "capture pair incomplete"
                    response.run_id = request.run_id
                    response.request_id = request.request_id
                    setattr(response, f"{side}_file", f"{side}/{index}.jpg")
                    other_side = "right" if side == "left" else "left"
                    setattr(response, f"{other_side}_error", "cannot open RTSP stream")
                    driver.call_capture_service = Mock(return_value=response)

                    self.assertTrue(driver.execute_capture_request(request))
                    self.assertTrue(driver.capture_accepting_requests)
                    self.assertEqual(driver.capture_consecutive_failures, 0)
                    self.assertIsNone(driver.interrupt_reason)
                    self.assertFalse(driver.capture_in_progress)

                driver.capture_timer_callback(now_monotonic=10.1)
                driver.cancel_active_goal.assert_called_once_with(context="zone capture")
                self.assertTrue(driver.capture_stop_requested)

    def test_partial_capture_with_wrong_identity_is_rejected(self):
        for field in ("run_id", "request_id"):
            with self.subTest(mismatched_field=field):
                driver = self.make_capture_driver()
                request = SimpleNamespace(run_id="run_1", request_id="request_1", zone_id="A")
                response = CapturePair.Response()
                response.run_id = request.run_id
                response.request_id = request.request_id
                response.left_file = "left/1.jpg"
                setattr(response, field, "unrelated")
                driver.call_capture_service = Mock(return_value=response)

                self.assertFalse(driver.execute_capture_request(request))
                self.assertFalse(driver.capture_accepting_requests)
                self.assertEqual(driver.capture_consecutive_failures, 1)

    def test_both_cameras_failing_still_disables_remaining_optional_captures(self):
        driver = self.make_capture_driver()
        request = SimpleNamespace(run_id="run_1", request_id="request_1", zone_id="A")
        response = CapturePair.Response()
        response.run_id = request.run_id
        response.request_id = request.request_id
        response.left_error = "left disconnected"
        response.right_error = "right disconnected"
        driver.call_capture_service = Mock(return_value=response)

        self.assertFalse(driver.execute_capture_request(request))

        self.assertFalse(driver.capture_accepting_requests)
        self.assertEqual(driver.capture_consecutive_failures, 1)
        self.assertIsNone(driver.interrupt_reason)
        self.assertFalse(driver.capture_in_progress)

    def test_robot_health_does_not_reset_nav2_during_stopped_capture(self):
        driver = self.make_capture_driver()
        driver.capture_stop_active = True
        driver.robot_is_ready = Mock(return_value=False)
        driver.start_robot_loss_handling = Mock()

        driver.robot_health_callback()

        driver.start_robot_loss_handling.assert_not_called()

    def test_failed_stop_aborts_mission_even_when_camera_failures_are_optional(self):
        driver = self.make_capture_driver()
        driver.publish_zero_velocity = Mock()
        driver.publish_status = Mock()
        driver.get_parameter = Mock(side_effect=lambda name: SimpleNamespace(value={
            "stop_zero_seconds": 2.0,
            "capture_failure_stops_mission": False,
        }[name]))

        driver.handle_capture_control_failure("A", "odometry stayed in motion")

        self.assertEqual(driver.interrupt_reason, "CAPTURE_CONTROL_FAILED")
        driver.publish_zero_velocity.assert_called_once_with()
        driver.publish_status.assert_called_once_with("ERROR capture_control_failed")

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

    def test_navigation_resends_same_goal_after_optional_capture_failure(self):
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
        driver.check_navigation_plan = Mock(return_value=True)
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
            return False

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
        driver.capture_consecutive_failures = 1
        driver.current_capture_zone = None
        driver.last_capture_position = None
        driver.capture_stop_requested = False
        driver.capture_stop_active = False
        driver.capture_stop_zone = None
        driver.last_capture_directory = ""
        parameter_values = {
            "capture_enabled": True,
            "navigation_map_yaml": "/root/map_0903.yaml",
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

    def test_capture_map_id_follows_navigation_map_file(self):
        self.assertEqual(map_id_from_yaml_path("~/map_1031.yaml"), "map_1031")
        self.assertEqual(map_id_from_yaml_path("/maps/map_1005-2.yml"), "map_1005-2")
        self.assertEqual(map_id_from_yaml_path(""), "")

    def test_capture_map_id_uses_map_server_when_navigation_map_is_unset(self):
        driver = self.make_capture_run_driver()
        driver.get_parameter = Mock(return_value=SimpleNamespace(value=""))
        driver.get_loaded_map_yaml = Mock(return_value="/root/map_1031.yaml")
        self.assertEqual(driver.get_capture_map_id(), "map_1031")

        driver.get_loaded_map_yaml = Mock(return_value=None)
        self.assertEqual(driver.get_capture_map_id(), "unknown_map")

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
        self.assertEqual(driver.capture_consecutive_failures, 0)

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
        driver.get_node_names_and_namespaces = Mock(return_value=[])
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
        driver.navigation_map_matches = Mock(return_value=True)
        driver.wait_for_localization = Mock(return_value=True)
        driver.clear_costmaps = Mock(return_value=True)

        initial_pose = (1.25, -0.75, -1.2)
        result = driver.prepare_navigation(initial_pose=initial_pose)

        self.assertTrue(result)
        driver.wait_for_localization.assert_called_once_with(initial_pose)
        self.assertEqual(driver.pose_mode, "LOCALIZING")
        self.assertTrue(driver.nav2_expected_active)

    def test_navigation_rejects_a_different_map_before_localizing(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.get_node_names_and_namespaces = Mock(return_value=[])
        driver.get_parameter = Mock(return_value=SimpleNamespace(value=True))
        driver.robot_is_ready = Mock(return_value=True)
        driver.publish_status = Mock()
        driver.call_supervisor = Mock(return_value=True)
        driver.navigation_map_matches = Mock(return_value=False)
        driver.wait_for_localization = Mock()

        self.assertFalse(driver.prepare_navigation(initial_pose=(0.0, 0.0, 0.0)))

        driver.wait_for_localization.assert_not_called()
        driver.call_supervisor.assert_called_once_with("start_localization")

    def test_map_server_must_match_the_mission_map(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.get_parameter = Mock(return_value=SimpleNamespace(value="~/map_1005-2.yaml"))
        driver.get_logger = Mock(return_value=Mock())
        driver.map_parameters_client = Mock()
        driver.call_service = Mock(return_value=SimpleNamespace(
            values=[SimpleNamespace(string_value="/root/map_1005-1.yaml")]
        ))

        self.assertFalse(driver.navigation_map_matches())

        self.assertIn("Navigation map mismatch", driver.get_logger().error.call_args.args[0])
        driver.call_service.return_value.values[0].string_value = "/root/map_1005-2.yaml"
        self.assertTrue(driver.navigation_map_matches())

    def test_missing_map_server_response_blocks_navigation(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.get_parameter = Mock(return_value=SimpleNamespace(value="/root/map_1005-2.yaml"))
        driver.get_logger = Mock(return_value=Mock())
        driver.map_parameters_client = Mock()
        driver.call_service = Mock(return_value=None)

        self.assertFalse(driver.navigation_map_matches())

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
