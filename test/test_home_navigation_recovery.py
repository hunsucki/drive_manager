"""Recover aborted HOME TF goals without skipping arrival or docking checks."""
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import Mock, call

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
import pytest
from tf2_ros import TransformException

from drive_manager.mission_driver import MissionDriver
from drive_manager.web_teleop import mission_status_blocks_force, mission_status_forces_safe


@pytest.fixture
def driver():
    node = MissionDriver.__new__(MissionDriver)
    values = {
        'action_server_timeout': 1.0,
        'home_behavior_tree': str(Path(__file__).resolve().parents[1] /
                                  'behavior_trees/navigate_home.xml'),
        'home_navigation_tf_retries': 2,
        'home_navigation_recovery_timeout_sec': 1.0,
        'home_arrival_xy_tolerance_m': 0.15,
        'home_alignment_yaw_tolerance_rad': 0.0524,
        'home_alignment_max_attempts': 3,
        'home_pose_max_age_sec': 0.5,
        'home_pose_wait_timeout_sec': 5.0,
        'robot_loss_grace_sec': 3.0,
    }
    node.parameter_values = values
    node.get_parameter = Mock(side_effect=lambda name: SimpleNamespace(value=values[name]))
    node.get_logger = Mock(return_value=Mock())
    node.shutdown_event = threading.Event()
    node.state_lock = threading.Lock()
    node.capture_lock = threading.Lock()
    node.sensor_lock = threading.Lock()
    node.home_navigation_recovery_active = False
    node.capture_stop_requested = False
    node.last_amcl_pose = object()
    node.has_interrupt_reason = Mock(return_value=False)
    node.publish_zero_velocity = Mock()
    node.publish_status = Mock()
    node.publish_command_result = Mock()
    node.nav_client = Mock()
    node.nav_client.wait_for_server.return_value = True
    node.check_navigation_plan = Mock(return_value=True)
    node.make_pose_stamped = Mock(return_value=PoseStamped())
    node.discard_capture_stop_request = Mock()
    node.navigation_feedback_callback = Mock()
    node.pause_navigation = Mock(return_value=True)
    node.prepare_navigation = Mock(return_value=True)
    node.robot_is_ready = Mock(return_value=True)
    node.amcl_covariance_is_acceptable = Mock(return_value=True)
    node.get_home_pose_error = Mock(return_value=(0.3, 0.01))
    node.request_stationary_amcl_update = Mock()
    node.describe_navigation_inputs = Mock(return_value='scan stale')
    node.wait_until_robot_stopped = Mock(return_value=True)
    node.wait_for_robot_ready = Mock(return_value=True)
    node.get_pose3_parameter = Mock(return_value=(0.01, -0.162, 0.0))
    node.spin_home_heading = Mock(return_value=True)
    node.run_docking_step = Mock(return_value=True)
    node.finalize_docked_state = Mock()
    node.reset_nav2 = Mock()
    return node


def navigation_results(node, codes, status=GoalStatus.STATUS_ABORTED):
    responses = []
    for code in codes:
        handle = Mock(accepted=True)
        result = SimpleNamespace(error_code=code, error_msg='')
        response = SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED if code == 0 else status,
            result=result,
        )
        responses.extend([handle, response])
    node.wait_for_future = Mock(side_effect=responses)


@pytest.fixture
def recovery_clock(monkeypatch, driver):
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr('drive_manager.mission_driver.time.monotonic', lambda: clock.now)
    def advance(delay):
        clock.now += delay
        return driver.shutdown_event.is_set()
    driver.shutdown_event.wait = Mock(side_effect=advance)
    return clock


def test_home_tf_abort_recovers_same_goal_then_aligns_before_docking(driver):
    navigation_results(driver, [FollowPath.Result.TF_ERROR, 0])
    # A fresh pose 16cm away permits a retry, not a premature docking handoff.
    driver.get_home_pose_error.side_effect = [(0.16, 0.01), (0.16, 0.01), (0.11, 0.01)]
    driver.handle_home()
    assert driver.nav_client.send_goal_async.call_count == 2
    assert driver.check_navigation_plan.call_args_list == [
        call('HOME_TO_DOCK', 0.01, -0.162, 0.0),
        call('HOME_TO_DOCK', 0.01, -0.162, 0.0),
    ]
    assert driver.pause_navigation.call_count == 2
    assert driver.prepare_navigation.call_args_list == [
        call(), call(initial_pose=None, force_relocalize=False),
    ]
    driver.reset_nav2.assert_not_called()
    driver.run_docking_step.assert_called_once_with('HOME')
    driver.publish_command_result.assert_called_once_with('HOME', True)
    assert not driver.home_navigation_recovery_active


def test_arrived_home_tf_abort_keeps_nav2_paused_and_hands_off_without_new_goal(driver):
    navigation_results(driver, [FollowPath.Result.TF_ERROR])
    driver.get_home_pose_error.return_value = (0.111, 0.01)
    driver.handle_home()
    assert driver.nav_client.send_goal_async.call_count == 1
    driver.prepare_navigation.assert_called_once_with()
    driver.spin_home_heading.assert_not_called()
    driver.run_docking_step.assert_called_once_with('HOME')
    driver.publish_command_result.assert_called_once_with('HOME', True)
    assert not driver.home_navigation_recovery_active


def test_start_final_return_can_hand_off_after_confirmed_arrival_without_home_retry(driver):
    driver.parameter_values.update({
        'ensure_nav2_active': True,
        'capture_enabled': False,
        'capture_required': False,
        'navigate_to_home_to_patrol_pose': False,
    })
    driver.pose_mode = 'AMCL'
    driver.pending_departure_initial_pose = None
    driver.navigation_stack_is_unique = Mock(return_value=True)
    driver.navigation_map_matches = Mock(return_value=True)
    driver.get_patrol_points = Mock(return_value=[('point_1', 1.0, 2.0)])
    driver.start_capture_run = Mock(return_value=True)
    driver.finish_capture_run = Mock(return_value=True)
    driver.log_start_mission = Mock()
    driver.get_home_pose_error.return_value = (0.111, 0.01)
    navigation_results(driver, [0, FollowPath.Result.TF_ERROR])
    driver.handle_start()
    assert driver.nav_client.send_goal_async.call_count == 2  # Patrol and first HOME only.
    driver.prepare_navigation.assert_called_once_with(initial_pose=None, force_relocalize=False)
    driver.run_docking_step.assert_called_once_with('START')
    driver.publish_command_result.assert_called_once_with('START', True)


def test_repeated_tf_abort_exhausts_two_retries_without_docking(driver):
    navigation_results(driver, [FollowPath.Result.TF_ERROR] * 3)
    driver.handle_home()
    assert driver.nav_client.send_goal_async.call_count == 3
    driver.run_docking_step.assert_not_called()
    driver.publish_command_result.assert_called_once_with('HOME', False)
    assert not driver.home_navigation_recovery_active


@pytest.mark.parametrize('label,code,status', [
    ('PATROL_1', FollowPath.Result.TF_ERROR, GoalStatus.STATUS_ABORTED),
    ('HOME_TO_DOCK', FollowPath.Result.NO_VALID_CONTROL, GoalStatus.STATUS_ABORTED),
    ('HOME_TO_DOCK', FollowPath.Result.TF_ERROR, GoalStatus.STATUS_CANCELED),
])
def test_other_failures_and_cancellations_are_not_retried(driver, label, code, status):
    navigation_results(driver, [code], status=status)
    assert not driver.navigate_to_named_pose(label, 0.01, -0.162, 0.0)
    assert driver.nav_client.send_goal_async.call_count == 1
    driver.prepare_navigation.assert_not_called()


def test_retry_can_be_disabled(driver):
    driver.parameter_values['home_navigation_tf_retries'] = 0
    navigation_results(driver, [FollowPath.Result.TF_ERROR])
    assert not driver.navigate_to_named_pose('HOME_TO_DOCK', 0.01, -0.162, 0.0)
    driver.prepare_navigation.assert_not_called()


@pytest.mark.parametrize('problem', ['sensors', 'covariance', 'tf'])
def test_unrecovered_inputs_timeout_stopped_without_resending(driver, recovery_clock, problem):
    navigation_results(driver, [FollowPath.Result.TF_ERROR])
    if problem == 'sensors':
        driver.robot_is_ready.return_value = False
    elif problem == 'covariance':
        driver.amcl_covariance_is_acceptable.return_value = False
    else:
        driver.get_home_pose_error.side_effect = TransformException('stale TF')
    driver.handle_home()
    assert recovery_clock.now == pytest.approx(1.0)
    assert driver.nav_client.send_goal_async.call_count == 1
    driver.prepare_navigation.assert_called_once_with()
    driver.run_docking_step.assert_not_called()
    assert driver.publish_zero_velocity.call_count > 1
    assert not driver.home_navigation_recovery_active


def test_temporarily_stale_tf_is_rechecked_before_retry(driver, recovery_clock):
    driver.get_home_pose_error.side_effect = [ValueError('stale TF'), (0.16, 0.01), (0.16, 0.01)]
    assert driver.recover_home_navigation_after_tf_error((0.01, -0.162, 0.0)) == 'retry'
    assert driver.get_home_pose_error.call_count == 3
    driver.request_stationary_amcl_update.assert_called_once_with()
    driver.prepare_navigation.assert_called_once_with(initial_pose=None, force_relocalize=False)


@pytest.mark.parametrize('reason', ['stop', 'shutdown'])
def test_stop_or_shutdown_during_recovery_blocks_new_goal(driver, recovery_clock, reason):
    navigation_results(driver, [FollowPath.Result.TF_ERROR])
    driver.robot_is_ready.return_value = False
    def stop_wait(delay):
        recovery_clock.now += delay
        if reason == 'stop':
            driver.has_interrupt_reason.return_value = True
        else:
            driver.shutdown_event.set()
        return driver.shutdown_event.is_set()
    driver.shutdown_event.wait.side_effect = stop_wait
    driver.handle_home()
    assert driver.nav_client.send_goal_async.call_count == 1
    driver.run_docking_step.assert_not_called()
    assert not driver.home_navigation_recovery_active


@pytest.mark.parametrize('phase', ['pause', 'stopped', 'prepare'])
def test_recovery_stage_failure_never_resends_or_docks(driver, phase):
    navigation_results(driver, [FollowPath.Result.TF_ERROR])
    if phase == 'pause':
        driver.pause_navigation.return_value = False
    elif phase == 'stopped':
        driver.wait_until_robot_stopped.return_value = False
    else:
        driver.prepare_navigation.return_value = False
    assert not driver.navigate_to_named_pose('HOME_TO_DOCK', 0.01, -0.162, 0.0)
    assert driver.nav_client.send_goal_async.call_count == 1
    driver.run_docking_step.assert_not_called()
    assert not driver.home_navigation_recovery_active


def test_late_fresh_pose_cannot_escape_recovery_deadline(driver, recovery_clock):
    def late_pose(target_pose):
        recovery_clock.now += 1.1
        return (0.1, 0.01)
    driver.get_home_pose_error.side_effect = late_pose
    assert not driver.recover_home_navigation_after_tf_error((0.01, -0.162, 0.0))
    driver.prepare_navigation.assert_not_called()


def test_recovery_flag_is_released_even_if_pause_raises(driver):
    driver.pause_navigation.side_effect = RuntimeError('service failed')
    with pytest.raises(RuntimeError):
        driver.recover_home_navigation_after_tf_error((0.01, -0.162, 0.0))
    assert not driver.home_navigation_recovery_active


def test_health_grace_does_not_race_controlled_stopped_recovery(driver, monkeypatch):
    driver.capture_stop_active = False
    driver.nav2_expected_active = True
    driver.robot_was_ready = True
    driver.robot_loss_handling = False
    driver.robot_unready_since = 0.0
    driver.home_navigation_recovery_active = True
    driver.robot_is_ready.return_value = False
    driver.start_robot_loss_handling = Mock()
    driver.robot_health_callback()
    assert driver.robot_unready_since is None
    driver.start_robot_loss_handling.assert_not_called()
    driver.home_navigation_recovery_active = False
    monkeypatch.setattr('drive_manager.mission_driver.time.monotonic', Mock(side_effect=[30., 34.]))
    driver.robot_health_callback()
    driver.robot_health_callback()
    driver.start_robot_loss_handling.assert_called_once_with()


def test_recovery_disarms_web_teleop_and_blocks_force():
    assert mission_status_forces_safe('HOME_NAVIGATION_RECOVERING')
    assert mission_status_blocks_force('HOME_NAVIGATION_RECOVERING')
