import subprocess
import sys
import tempfile
import unittest

from drive_manager.launch_guard import LaunchInstanceLock
from drive_manager.stack_health import duplicate_stack_nodes


class LaunchGuardTests(unittest.TestCase):
    def test_second_process_is_blocked_and_can_launch_after_owner_exits(self):
        with tempfile.TemporaryDirectory() as directory:
            first = LaunchInstanceLock(0, directory)
            first.acquire()
            try:
                script = (
                    "from drive_manager.launch_guard import LaunchInstanceLock; "
                    f"LaunchInstanceLock(0, {directory!r}).acquire()"
                )
                result = subprocess.run(
                    [sys.executable, "-c", script], capture_output=True, text=True
                )
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("already running", result.stderr)
                self.assertIn("PID", result.stderr)
                other_domain = LaunchInstanceLock(1, directory)
                other_domain.acquire()
                other_domain.release()
            finally:
                first.release()
            result = subprocess.run(
                [sys.executable, "-c", script], capture_output=True, text=True
            )
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_graph_detects_duplicates_in_the_same_namespace(self):
        self.assertEqual(duplicate_stack_nodes([
            ("amcl", "/"), ("amcl", "/"), ("amcl", "/other_robot"),
            ("unrelated", "/"), ("unrelated", "/"),
        ]), ["/amcl (2)"])

    def test_graph_allows_distinct_robot_namespaces(self):
        self.assertEqual(duplicate_stack_nodes([
            ("amcl", "/"), ("amcl", "/other_robot"),
        ]), [])
