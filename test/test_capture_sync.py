"""Directory transfer, retry, cancellation and docking trigger regression tests."""

from pathlib import Path
import shlex
import shutil
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from drive_manager.capture_sync import CaptureSync, build_sync_command
from drive_manager.mission_driver import MissionDriver


def test_command_pulls_entire_home_directory_without_deleting_local_files(tmp_path):
    destination = tmp_path / 'capture with spaces'
    command = build_sync_command('user', '192.168.0.15', 2222, '/key with spaces',
                                 'yes', destination)
    assert command[-2:] == ['user@192.168.0.15:capture/', str(destination) + '/']
    assert '--delete' not in command
    assert '--remove-source-files' not in command
    assert not any(arg.startswith('--include') or arg.startswith('--exclude') for arg in command)
    ssh = shlex.split(command[command.index('-e') + 1])
    assert ssh[ssh.index('-i') + 1] == '/key with spaces'
    assert ssh[ssh.index('-p') + 1] == '2222'


def wait_for(predicate):
    deadline = time.monotonic() + 8
    while not predicate():
        assert time.monotonic() < deadline, 'worker did not finish'
        time.sleep(0.02)


@pytest.mark.skipif(shutil.which('rsync') is None, reason='requires rsync')
def test_real_rsync_copies_whole_tree_and_preserves_server_only_files(tmp_path):
    source = tmp_path / 'robot'
    destination = tmp_path / 'server'
    (source / 'day/run/left').mkdir(parents=True)
    (source / 'day/run/left/photo.jpg').write_bytes(b'image')
    (source / 'day/run/metadata.yaml').write_text('status: completed\n')
    (source / '.hidden').write_text('hidden')
    destination.mkdir()
    (destination / 'server-only.jpg').write_bytes(b'keep')
    events = []
    sync = CaptureSync(lambda status, detail: events.append((status, detail)))
    command = ['rsync', '-rt', '--partial-dir=.rsync-partial',
               str(source) + '/', str(destination) + '/']
    try:
        sync.request(command, destination, 10, 1, 0)
        wait_for(lambda: any(status == 'SYNC_SUCCEEDED' for status, _ in events))
        assert (destination / 'day/run/left/photo.jpg').read_bytes() == b'image'
        assert (destination / 'day/run/metadata.yaml').exists()
        assert (destination / '.hidden').read_text() == 'hidden'
        assert (destination / 'server-only.jpg').read_bytes() == b'keep'
        (source / 'day/run/left/photo.jpg').write_bytes(b'updated image')
        events.clear()
        sync.request(command, destination, 10, 1, 0)
        wait_for(lambda: any(status == 'SYNC_SUCCEEDED' for status, _ in events))
        assert (destination / 'day/run/left/photo.jpg').read_bytes() == b'updated image'
        assert (source / 'day/run/left/photo.jpg').exists()
        assert not (destination / 'robot').exists()
    finally:
        sync.close()


def test_failed_transfer_retries_then_reports_failure(tmp_path):
    events = []
    sync = CaptureSync(lambda status, detail: events.append(status))
    try:
        sync.request([sys.executable, '-c', 'raise SystemExit(23)'], tmp_path, 10, 3, 0)
        wait_for(lambda: 'SYNC_FAILED' in events)
        assert events == ['SYNCING', 'SYNC_RETRYING', 'SYNCING',
                          'SYNC_RETRYING', 'SYNCING', 'SYNC_FAILED']
    finally:
        sync.close()


def test_missing_jetson_mount_fails_without_creating_container_directory(tmp_path):
    destination = tmp_path / 'jetson_capture'
    events = []
    sync = CaptureSync(lambda status, detail: events.append((status, detail)))
    sync._run = Mock()
    try:
        sync.request(['rsync'], destination, 10, 3, 0, require_mount=True)
        wait_for(lambda: bool(events))
        assert events[0][0] == 'SYNC_FAILED'
        assert str(destination) in events[0][1]
        assert not destination.exists()
        sync._run.assert_not_called()
    finally:
        sync.close()


def test_jetson_mount_check_allows_transfer_when_present(tmp_path, monkeypatch):
    destination = tmp_path / 'jetson_capture'
    destination.mkdir()
    monkeypatch.setattr('drive_manager.capture_sync.os.path.ismount',
                        lambda path: Path(path) == destination)
    events = []
    sync = CaptureSync(lambda status, detail: events.append((status, detail)))
    sync._run = Mock(return_value=(True, 'ok'))
    try:
        sync.request(['rsync'], destination, 10, 1, 0, require_mount=True)
        wait_for(lambda: any(status == 'SYNC_SUCCEEDED' for status, _ in events))
        sync._run.assert_called_once()
    finally:
        sync.close()


def test_timeout_and_shutdown_terminate_transfer(tmp_path):
    events = []
    sync = CaptureSync(lambda status, detail: events.append((status, detail)))
    command = [sys.executable, '-c', 'import time; time.sleep(60)']
    try:
        sync.request(command, tmp_path, 1, 1, 0)
        wait_for(lambda: any(status == 'SYNC_FAILED' for status, _ in events))
        assert 'timeout' in events[-1][1]
        events.clear()
        sync.request(command, tmp_path, 60, 1, 0)
        wait_for(lambda: bool(events))
    finally:
        start = time.monotonic()
        sync.close()
        assert time.monotonic() - start < 5
        assert not sync.worker.is_alive()


def test_requests_are_serialized_with_one_pending_pass(tmp_path):
    entered = threading.Event()
    release = threading.Event()
    calls = []
    sync = CaptureSync(lambda *args: None)

    def run(*args):
        calls.append(args)
        entered.set()
        assert release.wait(5)
        return True, 'ok'

    sync._run = run
    try:
        sync.request(['unused'], tmp_path, 10, 1, 0)
        assert entered.wait(2)
        for _ in range(10):
            sync.request(['unused'], tmp_path, 10, 1, 0)
        assert len(calls) == 1
        assert sync.pending.qsize() == 1
        release.set()
        wait_for(lambda: len(calls) == 2 and sync.pending.unfinished_tasks == 0)
    finally:
        release.set()
        sync.close()


@pytest.mark.parametrize('mission', ['START', 'HOME'])
@pytest.mark.parametrize('command,result,expected', [([], True, False),
                                                  (['dock'], False, False),
                                                  (['dock'], True, True)])
def test_only_successful_actual_docking_triggers_sync(mission, command, result, expected):
    driver = MissionDriver.__new__(MissionDriver)
    driver.get_docking_command = Mock(return_value=command)
    driver.run_docking_command = Mock(return_value=result)
    driver.publish_status = Mock()
    driver.get_logger = Mock(return_value=Mock())
    driver.start_capture_sync = Mock()
    driver.run_docking_step(mission)
    assert driver.start_capture_sync.called == expected


@pytest.mark.parametrize('enabled,stopping', [(False, False), (True, True)])
def test_disabled_or_shutting_down_driver_does_not_sync(enabled, stopping):
    driver = MissionDriver.__new__(MissionDriver)
    driver.get_parameter = Mock(return_value=SimpleNamespace(value=enabled))
    driver.shutdown_event = threading.Event()
    if stopping:
        driver.shutdown_event.set()
    driver.capture_sync = Mock()
    driver.start_capture_sync()
    driver.capture_sync.request.assert_not_called()


def test_driver_queues_jetson_bind_mount_destination():
    driver = MissionDriver.__new__(MissionDriver)
    values = {
        'capture_sync_enabled': True,
        'capture_sync_local_directory': '/mnt/jetson_capture',
        'capture_sync_require_mount': True,
        'capture_sync_timeout_sec': 1800.0,
        'capture_sync_attempts': 3,
        'capture_sync_retry_delay_sec': 10.0,
        'docking_ssh_user': 'user',
        'docking_ssh_host': '192.168.0.15',
        'docking_ssh_port': 22,
        'docking_ssh_identity_file': '/root/.ssh/id_ed25519_drive_manager',
        'docking_ssh_strict_host_key_checking': 'accept-new',
    }
    driver.get_parameter = Mock(side_effect=lambda name: SimpleNamespace(value=values[name]))
    driver.shutdown_event = threading.Event()
    driver.capture_sync = Mock()
    driver.start_capture_sync()
    args = driver.capture_sync.request.call_args.args
    assert args[0][-1] == '/mnt/jetson_capture/'
    assert args[1:] == ('/mnt/jetson_capture', 1800.0, 3, 10.0, True)
