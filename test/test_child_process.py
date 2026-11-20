import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


def process_is_running(pid):
    try:
        return 'State:\tZ' not in Path(f'/proc/{pid}/status').read_text()
    except FileNotFoundError:
        return False


class ChildProcessTests(unittest.TestCase):
    def test_parent_exit_stops_executed_child(self):
        with tempfile.TemporaryDirectory() as directory:
            pid_file = Path(directory) / 'child.pid'
            script = '''
import os, subprocess, sys, time
from pathlib import Path
child = subprocess.Popen([
    sys.executable, '-m', 'drive_manager.child_process', str(os.getpid()),
    '/bin/sleep', '30'
], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
deadline = time.monotonic() + 5
while time.monotonic() < deadline:
    if Path(f'/proc/{child.pid}/cmdline').read_bytes().startswith(b'/bin/sleep\\0'):
        Path(sys.argv[1]).write_text(str(child.pid))
        break
    if child.poll() is not None:
        raise SystemExit('child failed to exec')
    time.sleep(.02)
else:
    child.kill()
    raise SystemExit('child did not exec')
# Exit without terminating the child or waiting for it.
'''
            parent = subprocess.run(
                [sys.executable, '-c', script, str(pid_file)],
                capture_output=True, text=True, timeout=8,
            )
            self.assertEqual(parent.returncode, 0, parent.stderr)
            pid = int(pid_file.read_text())
            deadline = time.monotonic() + 3
            while process_is_running(pid) and time.monotonic() < deadline:
                time.sleep(.02)
            try:
                self.assertFalse(process_is_running(pid))
            finally:
                if process_is_running(pid):
                    os.kill(pid, 9)

    def test_already_exited_parent_cannot_start_a_node(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / 'started'
            result = subprocess.run([
                sys.executable, '-m', 'drive_manager.child_process', '1',
                sys.executable, '-c',
                'from pathlib import Path; import sys; Path(sys.argv[1]).touch()',
                str(marker),
            ], capture_output=True, text=True, timeout=5)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('refusing to start an orphan node', result.stderr)
            self.assertFalse(marker.exists())
