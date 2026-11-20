from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
from nav2_msgs.action import ComputePathToPose
from rclpy.task import Future

from drive_manager.mission_driver import MissionDriver


class NavigationPlanTests(unittest.TestCase):
    def make_driver(self):
        node = MissionDriver.__new__(MissionDriver)
        node.get_parameter = Mock(return_value=SimpleNamespace(value=1.0))
        node.plan_client = Mock()
        node.plan_client.wait_for_server.return_value = True
        node.make_pose_stamped = Mock(return_value=ComputePathToPose.Goal().goal)
        node.has_interrupt_reason = Mock(return_value=False)
        node.get_logger = Mock(return_value=Mock())
        node.publish_status = Mock()
        node.log_blocked_plan_costmap = Mock()
        return node

    def test_no_path_is_rejected_with_diagnostics(self):
        node = self.make_driver()
        handle = Mock(accepted=True)
        result = SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED,
            result=SimpleNamespace(error_code=208, error_msg='No valid path',
                                   path=SimpleNamespace(poses=[])),
        )
        node.wait_for_future = Mock(side_effect=[handle, result])
        self.assertFalse(node.check_navigation_plan('PATROL_1', 4.116, -.132, 0.0))
        node.log_blocked_plan_costmap.assert_called_once_with(4.116, -.132)
        node.publish_status.assert_called_with('ERROR navigation_no_valid_path PATROL_1')

    def test_success_requires_a_nonempty_path(self):
        for poses, expected in (([], False), ([object()], True)):
            node = self.make_driver()
            handle = Mock(accepted=True)
            result = SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=SimpleNamespace(error_code=0, error_msg='',
                                       path=SimpleNamespace(poses=poses)),
            )
            node.wait_for_future = Mock(side_effect=[handle, result])
            self.assertEqual(node.check_navigation_plan('PATROL_1', 1., 2., 0.), expected)

    def test_failed_preflight_never_sends_a_navigation_goal(self):
        node = self.make_driver()
        node.nav_client = Mock()
        node.shutdown_event = threading.Event()
        node.check_navigation_plan = Mock(return_value=False)
        node.publish_zero_velocity = Mock()
        self.assertFalse(node.navigate_to_named_pose('PATROL_1', 1., 2., 0.))
        node.nav_client.send_goal_async.assert_not_called()
        node.publish_zero_velocity.assert_called_once()

    def test_timed_out_planning_result_is_cancelled(self):
        node = self.make_driver()
        handle = Mock(accepted=True)
        node.wait_for_future = Mock(side_effect=[handle, None])
        self.assertFalse(node.check_navigation_plan('PATROL_1', 1., 2., 0.))
        handle.cancel_goal_async.assert_called_once()

    def test_late_planning_acceptance_is_cancelled(self):
        node = self.make_driver()
        future = Future()
        node.plan_client.send_goal_async.return_value = future
        node.wait_for_future = Mock(return_value=None)
        self.assertFalse(node.check_navigation_plan('PATROL_1', 1., 2., 0.))
        handle = Mock(accepted=True)
        future.set_result(handle)
        handle.cancel_goal_async.assert_called_once()

    def test_zero_feedback_does_not_claim_zero_remaining_distance(self):
        node = self.make_driver()
        node.state_lock = threading.Lock()
        node.last_feedback_time = 0.
        node.navigation_feedback_callback(SimpleNamespace(
            feedback=SimpleNamespace(distance_remaining=0.)
        ))
        self.assertIsNone(node.active_goal_distance_remaining)
        self.assertNotIn('remaining distance: 0.00', node.get_logger().info.call_args.args[0])

    def test_failed_navigation_pauses_without_resetting_localization(self):
        node = self.make_driver()
        node.navigate_to_named_pose = Mock(return_value=False)
        node.pause_navigation = Mock(return_value=True)
        node.reset_nav2 = Mock()
        node.publish_command_result = Mock()
        self.assertFalse(node.navigate_or_abort('START', 'PATROL_1', 1., 2., 0.))
        node.pause_navigation.assert_called_once()
        node.reset_nav2.assert_not_called()

    def make_start_driver(self, pose_mode):
        node = self.make_driver()
        node.sensor_lock = threading.Lock()
        node.pose_mode = pose_mode
        node.pending_departure_initial_pose = None
        node.navigation_stack_is_unique = Mock(return_value=True)
        node.get_pose3_parameter = Mock(return_value=(0., 0., 0.))
        node.get_patrol_points = Mock(return_value=[('point_1', 4.116, -.132)])
        values = {'ensure_nav2_active': True, 'capture_enabled': False,
                  'navigate_to_home_to_patrol_pose': False}
        node.get_parameter = Mock(side_effect=lambda name: SimpleNamespace(value=values[name]))
        node.wait_for_robot_ready = Mock(return_value=True)
        node.navigation_map_matches = Mock(return_value=True)
        node.pause_navigation = Mock(return_value=True)
        node.reset_nav2 = Mock(return_value=True)
        node.call_supervisor = Mock(return_value=True)
        node.run_start_escape = Mock(return_value=True)
        node.prepare_navigation = Mock(return_value=True)
        node.start_capture_run = Mock(return_value=True)
        node.log_start_mission = Mock()
        node.navigate_or_abort = Mock(return_value=False)
        node.publish_command_result = Mock()
        return node

    def test_start_outside_dock_preserves_localization_and_skips_escape(self):
        node = self.make_start_driver('AMCL')
        node.handle_start()
        node.run_start_escape.assert_not_called()
        node.reset_nav2.assert_not_called()
        node.prepare_navigation.assert_called_once_with(initial_pose=None, force_relocalize=False)

    def test_failed_first_start_retries_localization_without_another_escape(self):
        node = self.make_start_driver('DOCKED')
        node.prepare_navigation.return_value = False
        node.handle_start()
        self.assertEqual(node.pose_mode, 'LOCALIZING')
        node.handle_start()
        node.run_start_escape.assert_called_once()
        self.assertEqual(node.prepare_navigation.call_count, 2)
        node.prepare_navigation.assert_called_with(initial_pose=(0., 0., 0.), force_relocalize=False)

    def test_interrupted_escape_is_not_repeated_or_seeded_with_full_escape_pose(self):
        node = self.make_start_driver('DOCKED')
        node.run_start_escape.return_value = False
        node.handle_start()
        self.assertEqual(node.pose_mode, 'LOCALIZING')
        self.assertIsNone(node.pending_departure_initial_pose)
        node.prepare_navigation.return_value = False
        node.handle_start()
        node.run_start_escape.assert_called_once()
        node.prepare_navigation.assert_called_once_with(initial_pose=None, force_relocalize=False)

    def test_sensor_loss_pauses_navigation_and_preserves_localization(self):
        node = self.make_driver()
        node.get_parameter.return_value = SimpleNamespace(value=False)
        node.sensor_lock = threading.Lock()
        node.nav2_expected_active = True
        node.robot_loss_handling = True
        node.pause_navigation = Mock(return_value=True)
        node.reset_nav2 = Mock()
        node.handle_robot_loss()
        node.pause_navigation.assert_called_once()
        node.reset_nav2.assert_not_called()
        self.assertFalse(node.nav2_expected_active)
