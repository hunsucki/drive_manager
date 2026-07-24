import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from drive_manager.web_teleop import WebTeleop

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class WebTeleopStateTests(unittest.TestCase):
    def setUp(self):
        self.rclpy_ok = patch(
            "drive_manager.web_teleop.rclpy.ok",
            return_value=True,
        )
        self.rclpy_ok.start()

    def tearDown(self):
        self.rclpy_ok.stop()

    def make_teleop(self):
        teleop = WebTeleop.__new__(WebTeleop)
        teleop.lock = threading.Lock()
        teleop.transition_call_lock = threading.Lock()
        teleop.state_publish_lock = threading.Lock()
        teleop.shutdown_event = threading.Event()
        teleop.latest_commands = {
            teleop.SAFE: (Twist(), 0.0),
            teleop.FORCE: (Twist(), 0.0),
        }
        teleop.mode = teleop.DISABLED
        teleop.transitioning = False
        teleop.transition_target = None
        teleop.transition_generation = 0
        teleop.last_mission_status = "IDLE"
        teleop.stop_until = 0.0
        teleop.current_status = None
        teleop.current_active = None
        teleop.last_state_publish_time = 0.0
        teleop.safe_pub = RecordingPublisher()
        teleop.force_pub = RecordingPublisher()
        teleop.status_pub = RecordingPublisher()
        teleop.active_pub = RecordingPublisher()
        teleop.get_logger = Mock(return_value=Mock())

        parameters = {
            "command_timeout_sec": 0.35,
            "stop_publish_duration_sec": 0.0,
            "state_publish_period_sec": 1.0,
            "safe_max_linear_x": 0.15,
            "safe_max_angular_z": 0.40,
            "force_max_linear_x": 0.08,
            "force_max_angular_z": 0.25,
        }
        teleop.get_parameter = Mock(
            side_effect=lambda name: SimpleNamespace(value=parameters[name]),
        )
        teleop.cancel_navigation_goal = Mock(return_value=(True, "cancelled"))
        teleop.call_supervisor = Mock(return_value=(True, "ok"))
        teleop.start_transition = (
            lambda desired_mode, token: teleop.transition_mode(
                desired_mode,
                token,
            )
        )
        teleop.publish_state(teleop.DISABLED, active=False, force=True)
        return teleop

    def request_mode(self, teleop, mode):
        message = String()
        message.data = mode
        teleop.mode_request_callback(message)

    def make_twist(self, linear_x=0.0, angular_z=0.0):
        command = Twist()
        command.linear.x = linear_x
        command.angular.z = angular_z
        return command

    def assert_last_force_output(self, teleop, linear_x, angular_z=0.0):
        command = teleop.force_pub.messages[-1]
        self.assertAlmostEqual(command.linear.x, linear_x)
        self.assertAlmostEqual(command.angular.z, angular_z)

    def test_force_request_latches_force_and_active_true(self):
        teleop = self.make_teleop()

        self.request_mode(teleop, "FORCE")

        self.assertEqual(teleop.mode, teleop.FORCE)
        self.assertEqual(teleop.current_status, "FORCE")
        self.assertTrue(teleop.current_active)
        teleop.call_supervisor.assert_called_once_with("pause_navigation")

    def test_zero_force_twist_stops_motion_without_disarming(self):
        teleop = self.make_teleop()
        self.request_mode(teleop, "FORCE")
        teleop.force_callback(self.make_twist(linear_x=0.05))
        teleop.timer_callback()
        self.assert_last_force_output(teleop, 0.05)

        teleop.force_callback(self.make_twist())

        self.assert_last_force_output(teleop, 0.0)
        self.assertEqual(teleop.mode, teleop.FORCE)
        self.assertEqual(teleop.current_status, "FORCE")
        self.assertTrue(teleop.current_active)

    def test_watchdog_stops_force_but_next_force_twist_is_accepted(self):
        teleop = self.make_teleop()
        self.request_mode(teleop, "FORCE")
        teleop.latest_commands[teleop.FORCE] = (
            self.make_twist(linear_x=0.06),
            time.monotonic() - 1.0,
        )

        teleop.timer_callback()

        self.assert_last_force_output(teleop, 0.0)
        self.assertEqual(teleop.mode, teleop.FORCE)
        self.assertTrue(teleop.current_active)

        teleop.force_callback(self.make_twist(linear_x=-0.04))
        teleop.timer_callback()
        self.assert_last_force_output(teleop, -0.04)

    def test_force_output_is_clamped_on_server(self):
        teleop = self.make_teleop()
        self.request_mode(teleop, "FORCE")

        teleop.force_callback(self.make_twist(linear_x=2.0, angular_z=-3.0))
        teleop.timer_callback()

        self.assert_last_force_output(teleop, 0.08, -0.25)

    def test_safe_request_stops_and_returns_to_safe_path(self):
        teleop = self.make_teleop()
        self.request_mode(teleop, "FORCE")
        teleop.force_callback(self.make_twist(linear_x=0.05))
        teleop.timer_callback()

        self.request_mode(teleop, "SAFE")

        self.assert_last_force_output(teleop, 0.0)
        self.assertEqual(teleop.mode, teleop.SAFE)
        self.assertEqual(teleop.current_status, "SAFE")
        self.assertFalse(teleop.current_active)
        operations = [call.args[0] for call in teleop.call_supervisor.call_args_list]
        self.assertEqual(
            operations,
            ["pause_navigation", "start_localization", "start_navigation"],
        )

    def test_repeated_force_and_safe_requests_are_idempotent(self):
        teleop = self.make_teleop()

        self.request_mode(teleop, "FORCE")
        self.request_mode(teleop, "FORCE")
        self.request_mode(teleop, "SAFE")
        self.request_mode(teleop, "SAFE")

        operations = [call.args[0] for call in teleop.call_supervisor.call_args_list]
        self.assertEqual(operations.count("pause_navigation"), 1)
        self.assertEqual(operations.count("start_localization"), 1)
        self.assertEqual(operations.count("start_navigation"), 1)
        self.assertEqual(teleop.mode, teleop.SAFE)
        self.assertFalse(teleop.current_active)

    def test_estop_and_stop_force_disabled(self):
        for command in ("ESTOP", "STOP"):
            with self.subTest(command=command):
                teleop = self.make_teleop()
                self.request_mode(teleop, "FORCE")
                message = String()
                message.data = command

                teleop.safety_command_callback(message)

                self.assertEqual(teleop.mode, teleop.DISABLED)
                self.assertFalse(teleop.current_active)
                self.assert_last_force_output(teleop, 0.0)

    def test_docking_and_drive_errors_force_disabled(self):
        callbacks_and_statuses = (
            ("mission_status_callback", "DOCKING"),
            ("mission_status_callback", "ERROR navigation_failed"),
            ("drive_status_callback", "ERROR command_manager_failed"),
            ("supervisor_status_callback", "ERROR"),
        )
        for callback_name, status in callbacks_and_statuses:
            with self.subTest(callback=callback_name, status=status):
                teleop = self.make_teleop()
                self.request_mode(teleop, "FORCE")
                message = String()
                message.data = status

                getattr(teleop, callback_name)(message)

                self.assertEqual(teleop.mode, teleop.DISABLED)
                self.assertFalse(teleop.current_active)
                self.assert_last_force_output(teleop, 0.0)

    def test_force_request_is_rejected_in_docking_state(self):
        teleop = self.make_teleop()
        teleop.last_mission_status = "DOCKING"

        self.request_mode(teleop, "FORCE")

        self.assertEqual(teleop.mode, teleop.DISABLED)
        self.assertFalse(teleop.current_active)
        teleop.call_supervisor.assert_not_called()


if __name__ == "__main__":
    unittest.main()
