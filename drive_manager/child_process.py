"""Execute a launch child with a Linux parent-death signal."""

import ctypes
import os
import signal
import sys


def main():
    if len(sys.argv) < 3:
        raise SystemExit("Usage: child_process LAUNCH_PID EXECUTABLE [ARG ...]")
    launch_pid = int(sys.argv[1])
    libc = ctypes.CDLL(None, use_errno=True)
    # PR_SET_PDEATHSIG survives exec. It also covers children spawned while
    # launch is shutting down, before ProcessStarted can be handled.
    if libc.prctl(1, signal.SIGTERM, 0, 0, 0) != 0:
        errno = ctypes.get_errno()
        raise OSError(errno, os.strerror(errno))
    # Close the race in which launch exited before prctl registered the signal.
    if os.getppid() != launch_pid:
        raise SystemExit("Launch parent exited; refusing to start an orphan node")
    os.execvp(sys.argv[2], sys.argv[2:])


if __name__ == "__main__":
    main()
