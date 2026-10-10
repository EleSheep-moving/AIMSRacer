"""The CLI waits for concurrent NDT activation before publishing initial pose."""
import hashlib
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from lifecycle_msgs.msg import State


@pytest.fixture
def initializer(monkeypatch):
    scripts = Path(__file__).parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location('initializer_lifecycle_cli',
                                                scripts / 'relocalize_known_map.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_cli(module, monkeypatch, tmp_path, states):
    map_file = tmp_path / 'map.pcd'
    map_file.write_bytes(b'fixture-map')
    wall = [0.]
    published = []
    calls = []
    remaining = iter(states)
    last = [states[-1]]

    def get_state(request):
        last[0] = next(remaining, last[0])
        calls.append(last[0])
        return SimpleNamespace(done=lambda: True,
            result=lambda: SimpleNamespace(current_state=SimpleNamespace(id=last[0])))

    node = SimpleNamespace(
        map_sha256=hashlib.sha256(map_file.read_bytes()).hexdigest(),
        status={'epoch': 'old'}, status_received=0.,
        lifecycle=SimpleNamespace(service_is_ready=lambda: True, call_async=get_state),
        publisher=SimpleNamespace(get_subscription_count=lambda: 1, publish=published.append),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=1_000_000_000, to_msg=lambda: Time(sec=1, nanosec=0))),
        destroy_node=lambda: None)
    monkeypatch.setattr(module, 'MapInitializer', lambda target: node)
    monkeypatch.setattr(module.time, 'monotonic', lambda: wall[0])
    monkeypatch.setattr(module.rclpy, 'spin_once',
                        lambda node, timeout_sec: wall.__setitem__(0, wall[0] + timeout_sec))
    monkeypatch.setattr(module.rclpy, 'init', lambda: None)
    monkeypatch.setattr(module.rclpy, 'shutdown', lambda: None)
    # Trusted epoch validation is separately covered by localization_policy;
    # this fixture isolates lifecycle scheduling before the pose publication.
    monkeypatch.setattr(module, 'initialization_complete', lambda *args, **kwargs: True)
    monkeypatch.setattr(sys, 'argv', ['relocalize_known_map.py', str(map_file),
                                    '--pose-frame', 'base_link', '--timeout', '1.'])
    return module.main(), published, calls, wall[0]


def test_transient_unconfigured_and_inactive_wait_for_active(initializer, monkeypatch, tmp_path):
    result, published, calls, _ = run_cli(initializer, monkeypatch, tmp_path,
        [State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE])
    assert result == 0
    assert calls == [State.PRIMARY_STATE_UNCONFIGURED, State.PRIMARY_STATE_INACTIVE, State.PRIMARY_STATE_ACTIVE]
    assert len(published) == 1


def test_lifecycle_deadline_does_not_publish_pose(initializer, monkeypatch, tmp_path):
    result, published, calls, wall = run_cli(initializer, monkeypatch, tmp_path,
                                            [State.PRIMARY_STATE_INACTIVE])
    assert result == 1
    assert published == []
    assert calls and wall >= 1.


def test_already_active_manual_cli_publishes_once(initializer, monkeypatch, tmp_path):
    result, published, calls, _ = run_cli(initializer, monkeypatch, tmp_path,
                                         [State.PRIMARY_STATE_ACTIVE])
    assert result == 0
    assert calls == [State.PRIMARY_STATE_ACTIVE]
    assert len(published) == 1
