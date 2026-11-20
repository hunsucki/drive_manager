"""Keep one drive_manager launch per user and ROS domain on this host."""

import atexit
import fcntl
import os
from pathlib import Path
import shlex
import sys
import tempfile


class LaunchInstanceLock:
    def __init__(self, domain_id, directory=None):
        domain_id = int(domain_id)
        self.path = Path(directory or tempfile.gettempdir()) / (
            f"drive_manager-{os.getuid()}-domain-{domain_id}.lock"
        )
        self.fd = None

    def acquire(self):
        fd = os.open(self.path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            owner = os.pread(fd, 128, 0).decode("utf-8", errors="replace").strip()
            os.close(fd)
            raise RuntimeError(
                f"drive_manager is already running ({owner or 'another launch'}). "
                "Use the existing launch, or stop it and wait for its nodes to exit "
                "before launching again. No additional nodes were started."
            ) from None
        self.fd = fd
        os.ftruncate(fd, 0)
        os.write(fd, f"PID {os.getpid()}".encode("utf-8"))

    def release(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        # Keep the file: removing it would allow two processes to lock different
        # inodes at the same path during shutdown/relaunch.


def acquire_launch_lock(context):
    lock = LaunchInstanceLock(context.environment.get("ROS_DOMAIN_ID", "0"))
    lock.acquire()
    # Retain the lock until launch exits, including all child shutdown handling.
    # Releasing in OnShutdown would permit another launch while old nodes live.
    atexit.register(lock.release)
    context.extend_locals({"drive_manager_instance_lock": lock})
    prefix = shlex.join([
        sys.executable, "-m", "drive_manager.child_process", str(os.getpid())
    ])
    existing_prefix = context.launch_configurations.get("launch-prefix", "")
    context.launch_configurations["launch-prefix"] = (
        prefix + (" " + existing_prefix if existing_prefix else "")
    )
    return []
