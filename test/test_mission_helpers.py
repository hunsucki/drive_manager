import math
import unittest

from drive_manager.mission_driver import calc_yaw_from_to, yaw_to_quaternion
from drive_manager.web_teleop import clamp_twist
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


if __name__ == "__main__":
    unittest.main()
