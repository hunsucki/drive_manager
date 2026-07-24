import math
import queue
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from drive_manager.mission_driver import (
    calc_yaw_from_to,
    MissionDriver,
    yaw_to_quaternion,
)
from drive_manager.web_teleop import clamp_twist

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist


class MissionHelperTests(unittest.TestCase):
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

        driver.manual_initial_pose_callback(pose)

        self.assertTrue(driver.manual_initial_pose_pending)
        self.assertEqual(
            driver.command_queue.get_nowait(),
            MissionDriver.MANUAL_INITIAL_POSE_COMMAND,
        )

    def test_manual_initial_pose_is_ignored_during_a_mission(self):
        driver = self.make_manual_pose_driver()
        driver.mission_active = True

        driver.manual_initial_pose_callback(PoseWithCovarianceStamped())

        self.assertFalse(driver.manual_initial_pose_pending)
        self.assertTrue(driver.command_queue.empty())

    def test_manual_initial_pose_prepares_nav2_without_sending_a_goal(self):
        driver = self.make_manual_pose_driver()
        driver.publish_status = Mock()
        driver.wait_for_robot_ready = Mock(return_value=True)
        driver.prepare_navigation = Mock(return_value=True)

        driver.handle_manual_initial_pose()

        driver.wait_for_robot_ready.assert_called_once_with()
        driver.prepare_navigation.assert_called_once_with()
        driver.publish_status.assert_any_call("MANUAL_NAV2_READY")

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


if __name__ == "__main__":
    unittest.main()
