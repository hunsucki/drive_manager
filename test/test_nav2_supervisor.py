from types import SimpleNamespace
import threading
import unittest
from unittest.mock import Mock

from nav2_msgs.srv import ManageLifecycleNodes

from drive_manager.nav2_supervisor import Nav2Supervisor
from drive_manager.mission_driver import MissionDriver


class Nav2SupervisorTests(unittest.TestCase):
    def make_supervisor(self):
        node = Nav2Supervisor.__new__(Nav2Supervisor)
        node.stack_is_unique = Mock(return_value=True)
        node.manager_states = {"localization": "unconfigured"}
        node.last_node_states = {}
        node.manager_clients = {"localization": Mock()}
        node.manager_clients["localization"].wait_for_service.return_value = True
        node.get_parameter = Mock(return_value=SimpleNamespace(value=20.0))
        node.get_logger = Mock(return_value=Mock())
        node.publish_status = Mock()
        node.manager_is_active = Mock(return_value=False)
        node.send_manager_command = Mock(return_value=False)
        return node

    def test_failed_reset_cannot_succeed_using_cached_or_inactive_flag(self):
        node = self.make_supervisor()
        node.get_manager_state = Mock(return_value="active")
        node.get_manager_activity = Mock(return_value=False)
        self.assertFalse(node.reset_manager("localization"))
        self.assertEqual(node.manager_states["localization"], "unknown")

    def test_successful_reset_must_verify_every_node_unconfigured(self):
        node = self.make_supervisor()
        node.get_manager_state = Mock(side_effect=["active", "inconsistent"])
        node.send_manager_command.return_value = True
        self.assertFalse(node.reset_manager("localization"))

    def test_verified_reset_succeeds(self):
        node = self.make_supervisor()
        node.get_manager_state = Mock(side_effect=["active", "unconfigured"])
        node.send_manager_command.return_value = True
        self.assertTrue(node.reset_manager("localization"))

    def test_failed_resume_and_cleanup_never_issue_startup(self):
        node = self.make_supervisor()
        node.get_manager_state = Mock(return_value="inactive")
        self.assertFalse(node.ensure_manager_active("localization"))
        commands = [call.args[1] for call in node.send_manager_command.call_args_list]
        self.assertEqual(commands, [
            ManageLifecycleNodes.Request.RESUME, ManageLifecycleNodes.Request.CLEANUP
        ])
        self.assertNotIn(ManageLifecycleNodes.Request.STARTUP, commands)

    def test_mixed_states_are_reported_and_not_blindly_reset(self):
        node = self.make_supervisor()
        node.get_manager_state = Mock(return_value="inconsistent")
        self.assertFalse(node.ensure_manager_active("localization"))
        node.send_manager_command.assert_not_called()

    def test_manager_state_checks_amcl_as_well_as_map_server(self):
        node = self.make_supervisor()
        node.state_clients = {"localization": {
            "map_server": Mock(), "amcl": Mock()
        }}
        node.last_node_states = {}
        node.call_service = Mock(side_effect=[
            SimpleNamespace(current_state=SimpleNamespace(id=3)),
            SimpleNamespace(current_state=SimpleNamespace(id=2)),
        ])
        self.assertEqual(node.get_manager_state("localization"), "inconsistent")
        self.assertEqual(node.last_node_states["localization"], {
            "map_server": "active", "amcl": "inactive"
        })

    def test_duplicate_graph_prevents_lifecycle_commands(self):
        node = self.make_supervisor()
        node.stack_is_unique.return_value = False
        self.assertFalse(node.ensure_manager_active("localization"))
        node.send_manager_command.assert_not_called()

    def test_duplicate_graph_prevents_start_escape(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.navigation_stack_is_unique = Mock(return_value=False)
        driver.publish_status = Mock()
        driver.publish_command_result = Mock()
        driver.run_start_escape = Mock()
        driver.handle_start()
        driver.run_start_escape.assert_not_called()
        driver.publish_status.assert_called_with("ERROR duplicate_navigation_nodes")

    def test_lifecycle_failure_prevents_start_escape(self):
        driver = MissionDriver.__new__(MissionDriver)
        driver.sensor_lock = threading.Lock()
        driver.pose_mode = 'DOCKED'
        driver.navigation_stack_is_unique = Mock(return_value=True)
        driver.get_pose3_parameter = Mock(return_value=(0.0, 0.0, 0.0))
        driver.get_patrol_points = Mock(return_value=[("A", 1.0, 1.0)])
        driver.get_parameter = Mock(return_value=SimpleNamespace(value=True))
        driver.wait_for_robot_ready = Mock(return_value=True)
        driver.navigation_map_matches = Mock(return_value=True)
        driver.pause_navigation = Mock(return_value=True)
        driver.reset_nav2 = Mock(return_value=False)
        driver.publish_status = Mock()
        driver.publish_command_result = Mock()
        driver.run_start_escape = Mock()
        driver.handle_start()
        driver.run_start_escape.assert_not_called()
        driver.publish_status.assert_called_with("ERROR nav2_reset_failed")
