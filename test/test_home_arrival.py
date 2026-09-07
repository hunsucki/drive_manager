"""HOME navigation selection, alignment and failed-handoff regression tests."""
import math
from pathlib import Path
import threading
from types import SimpleNamespace
from unittest.mock import Mock
import xml.etree.ElementTree as ET

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped
from rclpy.task import Future
from rclpy.time import Time
import pytest
import yaml

from drive_manager.mission_driver import MissionDriver, yaw_to_quaternion
from drive_manager.web_teleop import mission_status_blocks_force


ROOT = Path(__file__).resolve().parents[1]


def make_driver():
    driver = MissionDriver.__new__(MissionDriver)
    values = {
        'home_arrival_xy_tolerance_m': 0.15,
        'home_alignment_yaw_tolerance_rad': 0.0524,
        'home_alignment_max_attempts': 3,
        'home_alignment_timeout_sec': 15.0,
        'home_pose_max_age_sec': 0.5,
        'action_server_timeout': 1.0,
        'map_frame': 'map',
        'base_frame': 'base_link',
        'home_behavior_tree': str(ROOT / 'behavior_trees/navigate_home.xml'),
    }
    driver.get_parameter = Mock(side_effect=lambda name: SimpleNamespace(value=values[name]))
    driver.get_logger = Mock(return_value=Mock())
    driver.state_lock = threading.Lock()
    driver.capture_lock = threading.Lock()
    driver.shutdown_event = threading.Event()
    driver.has_interrupt_reason = Mock(return_value=False)
    driver.wait_until_robot_stopped = Mock(return_value=True)
    driver.publish_zero_velocity = Mock()
    driver.publish_status = Mock()
    driver.publish_command_result = Mock()
    driver.spin_home_heading = Mock(return_value=True)
    return driver


def test_alignment_corrects_heading_then_checks_again():
    driver = make_driver()
    driver.get_home_pose_error = Mock(side_effect=[(0.10, 0.09), (0.11, -0.02)])
    assert driver.align_home_heading((0.0, 0.0, math.pi / 2))
    driver.spin_home_heading.assert_called_once_with(0.09)
    assert driver.wait_until_robot_stopped.call_count == 2


def test_already_aligned_does_not_rotate():
    driver = make_driver()
    driver.get_home_pose_error = Mock(return_value=(0.10, 0.01))
    assert driver.align_home_heading((0.0, 0.0, 0.0))
    driver.spin_home_heading.assert_not_called()


@pytest.mark.parametrize('error', [(0.16, 0.01), ValueError('stale TF')])
def test_invalid_position_blocks_rotation_and_handoff(error):
    driver = make_driver()
    driver.get_home_pose_error = Mock()
    if isinstance(error, Exception):
        driver.get_home_pose_error.side_effect = error
    else:
        driver.get_home_pose_error.return_value = error
    assert not driver.align_home_or_abort('HOME', (0.0, 0.0, 0.0))
    driver.spin_home_heading.assert_not_called()
    driver.publish_command_result.assert_called_once_with('HOME', False)


def test_position_drift_after_rotation_is_rejected():
    driver = make_driver()
    driver.get_home_pose_error = Mock(side_effect=[(0.10, 0.09), (0.18, 0.01)])
    assert not driver.align_home_heading((0.0, 0.0, 0.0))


def test_alignment_has_bounded_attempts():
    driver = make_driver()
    driver.get_home_pose_error = Mock(return_value=(0.10, 0.09))
    assert not driver.align_home_heading((0.0, 0.0, 0.0))
    assert driver.spin_home_heading.call_count == 3


@pytest.mark.parametrize('reason', ['stop', 'moving', 'spin_failed'])
def test_alignment_failures_stop_handoff(reason):
    driver = make_driver()
    driver.get_home_pose_error = Mock(return_value=(0.10, 0.09))
    if reason == 'stop':
        driver.has_interrupt_reason.return_value = True
    elif reason == 'moving':
        driver.wait_until_robot_stopped.return_value = False
    else:
        driver.spin_home_heading.return_value = False
    assert not driver.align_home_heading((0.0, 0.0, 0.0))


def make_tf_driver(stamp=10.0, now=10.1, yaw=0.0):
    driver = make_driver()
    transform = TransformStamped()
    transform.header.stamp = Time(seconds=stamp).to_msg()
    transform.transform.rotation.z, transform.transform.rotation.w = yaw_to_quaternion(yaw)
    driver.tf_buffer = Mock()
    driver.tf_buffer.lookup_transform.return_value = transform
    driver.get_clock = Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=now)))
    return driver


@pytest.mark.parametrize('stamp', [9.0, 11.0])
def test_stale_or_future_tf_rejected(stamp):
    driver = make_tf_driver(stamp=stamp)
    with pytest.raises(ValueError):
        driver.get_home_pose_error((0.0, 0.0, 0.0))


def test_yaw_correction_crosses_pi_by_shortest_rotation():
    driver = make_tf_driver(yaw=math.radians(179))
    distance, error = driver.get_home_pose_error((0.0, 0.0, math.radians(-179)))
    assert distance == 0.0
    assert math.degrees(error) == pytest.approx(2.0)


def test_home_failure_never_pauses_then_docks():
    driver = make_driver()
    driver.wait_for_robot_ready = Mock(return_value=True)
    driver.prepare_navigation = Mock(return_value=True)
    driver.get_pose3_parameter = Mock(return_value=(0.0, 0.0, 0.0))
    driver.navigate_or_abort = Mock(return_value=True)
    driver.align_home_or_abort = Mock(return_value=False)
    driver.pause_navigation = Mock()
    driver.run_docking_step = Mock()
    driver.handle_home()
    driver.pause_navigation.assert_not_called()
    driver.run_docking_step.assert_not_called()


def test_home_alignment_blocks_force_teleop():
    assert mission_status_blocks_force('HOME_ALIGNING')


@pytest.mark.parametrize('status', [GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, None])
def test_spin_result_and_timeout_cancel(status):
    driver = make_driver()
    handle = Mock(accepted=True)
    driver.home_spin_client = Mock()
    driver.home_spin_client.wait_for_server.return_value = True
    result = None if status is None else SimpleNamespace(status=status, result=Mock(error_code=703))
    driver.wait_for_future = Mock(side_effect=[handle, result, Mock()])
    assert MissionDriver.spin_home_heading(driver, -0.09) == (status == GoalStatus.STATUS_SUCCEEDED)
    assert driver.active_goal_handle is None
    if status is None:
        handle.cancel_goal_async.assert_called_once_with()
    driver.publish_zero_velocity.assert_called_once_with()
    goal = driver.home_spin_client.send_goal_async.call_args.args[0]
    assert goal.target_yaw == pytest.approx(-0.09)
    assert goal.time_allowance.sec == 15


def test_late_spin_acceptance_is_cancelled():
    driver = make_driver()
    future = Future()
    driver.home_spin_client = Mock()
    driver.home_spin_client.send_goal_async.return_value = future
    driver.wait_for_future = Mock(return_value=None)
    assert not MissionDriver.spin_home_heading(driver, 0.09)
    late_handle = Mock(accepted=True)
    future.set_result(late_handle)
    late_handle.cancel_goal_async.assert_called_once_with()


@pytest.mark.parametrize('label', ['HOME_TO_DOCK', 'PATROL_1'])
def test_only_home_navigation_uses_strict_tree(label):
    driver = make_driver()
    driver.capture_stop_requested = False
    driver.discard_capture_stop_request = Mock()
    driver.navigation_feedback_callback = Mock()
    driver.nav_client = Mock()
    handle = Mock(accepted=True)
    driver.wait_for_future = Mock(side_effect=[handle, SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=Mock())])
    driver.get_clock = Mock(return_value=SimpleNamespace(now=lambda: Time(seconds=10)))
    assert driver.navigate_to_named_pose(label, 1.0, 2.0, 1.57)
    goal = driver.nav_client.send_goal_async.call_args.args[0]
    if label == 'HOME_TO_DOCK':
        assert goal.behavior_tree.endswith('/navigate_home.xml')
    else:
        assert goal.behavior_tree == ''


def test_behavior_tree_plugin_ids_match_config_and_preserve_patrol():
    config = yaml.safe_load((ROOT / 'param/stella.yaml').read_text())
    controller = config['controller_server']['ros__parameters']
    assert controller['precise_goal_checker']['xy_goal_tolerance'] == 0.3
    assert controller['precise_goal_checker']['yaw_goal_tolerance'] == 0.3
    assert controller['FollowPath']['trans_stopped_velocity'] == 0.25
    assert controller['HomeFollowPath']['xy_goal_tolerance'] == controller['home_goal_checker']['xy_goal_tolerance'] == 0.15
    for filename in ('navigate_to_pose.xml', 'navigate_through_poses.xml', 'navigate_home.xml'):
        tree = ET.parse(ROOT / 'behavior_trees' / filename)
        follow = tree.find('.//FollowPath')
        assert follow.attrib['goal_checker_id'] in controller['goal_checker_plugins']
        assert follow.attrib['progress_checker_id'] in controller['progress_checker_plugins']
        if filename == 'navigate_home.xml':
            assert follow.attrib['controller_id'] == 'HomeFollowPath'
            assert follow.attrib['goal_checker_id'] == 'home_goal_checker'
        else:
            assert follow.attrib['goal_checker_id'] == 'precise_goal_checker'


@pytest.mark.parametrize('command', ['HOME', 'START'])
@pytest.mark.parametrize('alignment_ok', [True, False])
def test_both_return_paths_align_before_pausing_and_docking(command, alignment_ok):
    driver = make_driver()
    driver.get_parameter = Mock(return_value=SimpleNamespace(value=False))
    driver.get_pose3_parameter = Mock(return_value=(0.0, 0.0, 0.0))
    driver.get_patrol_points = Mock(return_value=[('point_1', 1.0, 1.0)])
    driver.wait_for_robot_ready = Mock(return_value=True)
    driver.prepare_navigation = Mock(return_value=True)
    driver.run_start_escape = Mock(return_value=True)
    driver.start_capture_run = Mock(return_value=True)
    driver.log_start_mission = Mock()
    driver.finalize_docked_state = Mock()
    events = []
    driver.navigate_or_abort = Mock(side_effect=lambda *args: events.append('navigate') or True)
    driver.finish_capture_run = Mock(side_effect=lambda: events.append('finish_capture') or True)
    driver.align_home_or_abort = Mock(side_effect=lambda *args: events.append('align') or alignment_ok)
    driver.pause_navigation = Mock(side_effect=lambda: events.append('pause') or True)
    driver.run_docking_step = Mock(side_effect=lambda *args: events.append('dock') or True)
    if command == 'HOME':
        driver.handle_home()
        expected = ['navigate', 'align']
    else:
        driver.handle_start()
        expected = ['pause', 'navigate', 'navigate', 'finish_capture', 'align']
    if alignment_ok:
        expected += ['pause', 'dock']
    else:
        driver.run_docking_step.assert_not_called()
    assert events == expected


def test_stop_after_spin_acceptance_cancels_before_waiting_for_motion():
    driver = make_driver()
    driver.home_spin_client = Mock()
    handle = Mock(accepted=True)
    driver.wait_for_future = Mock(side_effect=[handle, Mock()])
    driver.has_interrupt_reason = Mock(side_effect=[False, True])
    assert not MissionDriver.spin_home_heading(driver, 0.09)
    handle.get_result_async.assert_not_called()
    handle.cancel_goal_async.assert_called_once_with()
    assert driver.active_goal_handle is None
