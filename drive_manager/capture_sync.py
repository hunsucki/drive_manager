"""Pull the robot's entire capture directory without blocking ROS callbacks."""

import os
from pathlib import Path
import queue
import shlex
import signal
import subprocess
import tempfile
import threading
import time


def build_sync_command(user, host, port, identity_file, host_key_checking, destination):
    if not user or not host or user.startswith('-') or host.startswith('-'):
        raise ValueError('Capture sync requires a valid docking SSH user and host')
    ssh = [
        'ssh', '-p', str(port), '-o', 'BatchMode=yes',
        '-o', 'ConnectTimeout=10', '-o', 'ServerAliveInterval=15',
        '-o', 'ServerAliveCountMax=3',
        '-o', f'StrictHostKeyChecking={host_key_checking}',
    ]
    if identity_file:
        ssh.extend(['-i', os.path.expanduser(identity_file)])
    # A relative remote path is resolved in the SSH user's home directory.
    # Keep the trailing slash: copy contents, not capture/capture.
    return [
        'rsync', '-rt', '--partial-dir=.rsync-partial', '--timeout=60',
        '--stats', '-e', shlex.join(ssh), '--',
        f'{user}@{host}:capture/', str(Path(destination).expanduser()) + '/',
    ]


class CaptureSync:
    """Serialize transfers and coalesce repeated requests into one pending pass."""

    def __init__(self, report):
        self.report = report
        self.pending = queue.Queue(maxsize=1)
        self.stopping = threading.Event()
        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()

    def request(self, command, destination, timeout, attempts, retry_delay):
        if self.stopping.is_set():
            return
        job = (list(command), destination, max(1.0, timeout),
               max(1, attempts), max(0.0, retry_delay))
        try:
            self.pending.put_nowait(job)
        except queue.Full:
            # The queued pass also scans the whole tree, including newer files.
            pass

    def close(self):
        self.stopping.set()
        self.worker.join()

    def _worker(self):
        while not self.stopping.is_set():
            try:
                job = self.pending.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                self._sync(*job)
            except Exception as exc:
                if not self.stopping.is_set():
                    self.report('SYNC_FAILED', str(exc))
            finally:
                self.pending.task_done()

    def _sync(self, command, destination, timeout, attempts, retry_delay):
        Path(destination).expanduser().mkdir(parents=True, exist_ok=True)
        for attempt in range(1, attempts + 1):
            if self.stopping.is_set():
                return
            self.report('SYNCING', f'attempt={attempt}/{attempts}, destination={destination}')
            try:
                success, detail = self._run(command, timeout)
            except OSError as exc:
                success, detail = False, str(exc)
            if self.stopping.is_set():
                return
            if success:
                self.report('SYNC_SUCCEEDED', detail)
                return
            if attempt == attempts:
                self.report('SYNC_FAILED', detail)
            else:
                self.report('SYNC_RETRYING', detail)
                if self.stopping.wait(retry_delay):
                    return

    def _run(self, command, timeout):
        # A file avoids pipe-buffer deadlocks and unbounded RAM for rsync output.
        with tempfile.TemporaryFile() as output:
            process = subprocess.Popen(
                command, stdin=subprocess.DEVNULL, stdout=output,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            deadline = time.monotonic() + timeout
            timed_out = False
            try:
                while process.poll() is None:
                    if self.stopping.wait(0.2):
                        break
                    if time.monotonic() >= deadline:
                        timed_out = True
                        break
            finally:
                if process.poll() is None:
                    self._terminate(process)
            output.seek(max(0, output.seek(0, os.SEEK_END) - 4096))
            detail = output.read().decode('utf-8', errors='replace').strip()
            prefix = 'timeout' if timed_out else f'exit={process.returncode}'
            return process.returncode == 0 and not timed_out, f'{prefix}: {detail}'

    @staticmethod
    def _terminate(process):
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            process.wait()
