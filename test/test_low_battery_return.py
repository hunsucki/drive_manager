"""Low-battery HOME handoff, sensor validation and mission lifecycle regressions."""
import math
import queue
import threading
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from drive_manager.mission_driver import MissionDriver


@pytest.fixture
def driver():
    node = MissionDriver.__new__(MissionDriver)
    node.parameter_values = {
        'low_battery_return_enabled': True,
        'low_battery_threshold': 0.20,
        'battery_message_timeout_sec': 5.0,
    }
    node.get_parameter = Mock(side_effect=lambda name: SimpleNamespace(
        value=node.parameter_values[name],
    ))
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=100_000_000_000),
    ))
    node.get_logger = Mock(return_value=Mock())
    node.state_lock = threading.Lock()
    node.shutdown_event = threading.Event()
    node.command_queue = queue.Queue()
    node.mission_active = True
    node.active_mission_command = 'START'
    node.start_returning_home = False
    node.interrupt_reason = None
    node.estop_latched = False
    node.web_teleop_active = False
    node.cancel_active_goal = Mock()
    node.publish_status = Mock()
    return node


def battery(percentage=0.20, stamp=100, present=True):
    message = BatteryState()
    message.header.stamp.sec = stamp
    message.percentage = percentage
    message.present = present
    return message


@pytest.mark.parametrize('percentage', [0.0, 0.10, 0.199, 0.20])
def test_low_battery_cancels_start_and_queues_home(driver, percentage):
    # Exercise serialization: exactly 20% rounds UP in BatteryState's float32.
    from rclpy.serialization import deserialize_message, serialize_message
    message = deserialize_message(serialize_message(battery(percentage)), BatteryState)
    driver.battery_callback(message)
    driver.cancel_active_goal.assert_called_once_with()
    driver.publish_status.assert_called_once_with('RETURNING_HOME_REQUESTED')
    assert driver.interrupt_reason == 'HOME'
    assert driver.command_queue.get_nowait() == 'HOME'
    assert driver.command_queue.empty()


def test_repeated_samples_do_not_restart_home_even_after_interrupt_is_consumed(driver):
    driver.battery_callback(battery())
    driver.consume_interrupt_reason()
    for percentage in [0.19, 0.21, 0.18]:
        driver.battery_callback(battery(percentage))
    assert driver.command_queue.qsize() == 1
    driver.cancel_active_goal.assert_called_once_with()


@pytest.mark.parametrize('percentage', [0.20001, 0.21, 1.0, -0.01, 20.0,
                                       math.nan, math.inf, -math.inf])
def test_high_or_invalid_percentages_do_not_request_home(driver, percentage):
    driver.battery_callback(battery(percentage))
    assert driver.command_queue.empty()
    driver.cancel_active_goal.assert_not_called()


@pytest.mark.parametrize('stamp,present', [(94, True), (101, True), (0, True),
                                         (100, False)])
def test_old_future_unstamped_or_absent_battery_is_ignored(driver, stamp, present):
    driver.battery_callback(battery(stamp=stamp, present=present))
    assert driver.command_queue.empty()


@pytest.mark.parametrize('state', [
    {'mission_active': False, 'active_mission_command': None},
    {'active_mission_command': 'HOME'},
    {'active_mission_command': MissionDriver.MANUAL_INITIAL_POSE_COMMAND},
    {'start_returning_home': True},
    {'interrupt_reason': 'HOME'},
    {'interrupt_reason': 'STOP'},
    {'interrupt_reason': 'ESTOP'},
    {'estop_latched': True},
    {'web_teleop_active': True},
])
def test_other_missions_return_and_operator_interrupts_are_respected(driver, state):
    for name, value in state.items():
        setattr(driver, name, value)
    driver.battery_callback(battery())
    assert driver.command_queue.empty()
    driver.cancel_active_goal.assert_not_called()
    if 'interrupt_reason' in state:
        assert driver.interrupt_reason == state['interrupt_reason']


def test_shutdown_does_not_request_home(driver):
    driver.shutdown_event.set()
    driver.battery_callback(battery())
    assert driver.command_queue.empty()


@pytest.mark.parametrize('parameters', [
    {'low_battery_return_enabled': False},
    {'low_battery_threshold': 0.10},
    {'low_battery_threshold': math.nan},
    {'low_battery_threshold': 20.0},
    {'battery_message_timeout_sec': -1.0},
])
def test_feature_switch_threshold_and_invalid_configuration(driver, parameters):
    driver.parameter_values.update(parameters)
    driver.battery_callback(battery())
    assert driver.command_queue.empty()


def test_next_start_rearms_low_battery_return(driver):
    driver.battery_callback(battery())
    driver.command_queue.get_nowait()
    driver.consume_interrupt_reason()
    driver.set_active_mission_command(None)
    driver.set_active_mission_command('START')
    driver.battery_callback(battery())
    assert driver.command_queue.get_nowait() == 'HOME'
    assert driver.cancel_active_goal.call_count == 2


def test_normal_start_return_alignment_and_docking_are_not_restarted(driver):
    driver.parameter_values.update({
        'ensure_nav2_active': False,
        'capture_enabled': False,
        'capture_required': False,
        'navigate_to_home_to_patrol_pose': False,
    })
    driver.sensor_lock = threading.Lock()
    driver.pose_mode = 'AMCL'
    driver.pending_departure_initial_pose = None
    driver.navigation_stack_is_unique = Mock(return_value=True)
    driver.get_pose3_parameter = Mock(return_value=(0.01, -0.162, 0.0))
    driver.get_patrol_points = Mock(return_value=[('point_1', 1.0, 2.0)])
    driver.wait_for_robot_ready = Mock(return_value=True)
    driver.pause_navigation = Mock(return_value=True)
    driver.prepare_navigation = Mock(return_value=True)
    driver.start_capture_run = Mock(return_value=True)
    driver.finish_capture_run = Mock(return_value=True)
    driver.log_start_mission = Mock()
    driver.finalize_docked_state = Mock()

    def low_battery(*args):
        driver.battery_callback(battery())
        return True

    def navigate(command, label, *pose):
        if label == 'HOME_TO_DOCK':
            low_battery()
        return True

    driver.navigate_or_abort = Mock(side_effect=navigate)
    driver.align_home_or_abort = Mock(side_effect=low_battery)
    driver.run_docking_step = Mock(side_effect=low_battery)
    driver.handle_start()
    assert driver.command_queue.empty()
    driver.cancel_active_goal.assert_not_called()
    driver.run_docking_step.assert_called_once_with('START')
    driver.publish_status.assert_called_with('SUCCEEDED START')


@pytest.mark.parametrize('start_reports_result', [False, True])
def test_worker_handoff_cleans_capture_then_navigates_and_docks(driver, start_reports_result):
    events = []
    driver.command_queue.put('START')
    driver.mission_active = False
    driver.active_mission_command = None

    def start():
        driver.battery_callback(battery())
        if start_reports_result:
            driver.publish_command_result('START', False)

    def status(value):
        if value == 'SUCCEEDED HOME':
            driver.shutdown_event.set()

    driver.handle_start = Mock(side_effect=start)
    driver.publish_status.side_effect = status
    driver.is_capture_run_active = Mock(return_value=True)
    driver.get_capture_abort_reason = Mock(return_value='HOME')
    driver.abort_capture_run = Mock(side_effect=lambda reason: events.append('abort_capture'))
    driver.reset_capture_tracking = Mock()
    driver.wait_for_robot_ready = Mock(side_effect=lambda: not driver.has_interrupt_reason())
    driver.prepare_navigation = Mock(return_value=True)
    driver.get_pose3_parameter = Mock(return_value=(0.01, -0.162, 0.0))
    driver.navigate_or_abort = Mock(side_effect=lambda *args: events.append('navigate') or True)
    driver.align_home_or_abort = Mock(side_effect=lambda *args: events.append('align') or True)
    driver.pause_navigation = Mock(side_effect=lambda: events.append('pause') or True)
    driver.run_docking_step = Mock(side_effect=lambda command: events.append('dock') or True)
    driver.finalize_docked_state = Mock(side_effect=lambda: events.append('finalize'))

    # Use the actual worker on a bounded thread so a broken handoff cannot hang pytest.
    worker = threading.Thread(target=driver.worker_loop, daemon=True)
    worker.start()
    worker.join(timeout=2.0)
    completed = driver.shutdown_event.is_set()
    driver.shutdown_event.set()
    worker.join(timeout=1.0)

    assert completed
    assert not worker.is_alive()
    assert events == ['abort_capture', 'navigate', 'align', 'pause', 'dock', 'finalize']
    driver.navigate_or_abort.assert_called_once_with('HOME', 'HOME_TO_DOCK', 0.01, -0.162, 0.0)
    assert driver.interrupt_reason is None
    assert not driver.mission_active


@pytest.mark.parametrize('reason', ['STOP', 'ESTOP'])
def test_queued_home_does_not_clear_operator_stop(driver, reason):
    driver.command_queue.put('HOME')
    driver.interrupt_reason = reason
    driver.is_capture_run_active = Mock(return_value=False)
    driver.reset_capture_tracking = Mock()

    def home():
        assert driver.interrupt_reason == reason
        driver.shutdown_event.set()

    driver.handle_home = Mock(side_effect=home)
    driver.worker_loop()
    driver.handle_home.assert_called_once_with()


def test_stop_after_automatic_home_cancels_queued_return(driver):
    driver.battery_callback(battery())
    driver.stop_robot = Mock(side_effect=driver.set_interrupt_reason)
    driver.command_callback(String(data='STOP'))
    assert driver.command_queue.empty()
    assert driver.interrupt_reason == 'STOP'
    driver.battery_callback(battery())
    assert driver.command_queue.empty()
