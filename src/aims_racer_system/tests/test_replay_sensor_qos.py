"""The raw IMU replay writer must retain propagation data across receiver pauses."""
import hashlib
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('sensor_replay', ROOT / 'replay/fastlio_ndt_replay.py')
replay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(replay)


def test_default_player_command_uses_only_deep_reliable_raw_imu_profile():
    profile = ROOT / 'params/replay_sensor_qos.yaml'
    command = replay.playback_command(Path('/recorded bag'), 5., profile)
    selected = Path(command[command.index('--qos-profile-overrides-path') + 1])
    assert selected == profile
    assert yaml.safe_load(selected.read_text()) == {
        '/livox/imu': dict(history='keep_last', depth=4096,
                           reliability='reliable', durability='volatile')}
    assert command[command.index('--topics') + 1:] == [
        '/livox/lidar', '/livox/imu', '/rear_axle/wheel_odom']


def test_explicit_profile_is_passed_to_player_and_hashed(tmp_path):
    profile = tmp_path / 'explicit replay.yaml'
    profile.write_text('/livox/imu: {history: keep_last, depth: 8192, reliability: reliable}\n')
    command = replay.playback_command(Path('/bag'), 2., profile)
    selected = Path(command[command.index('--qos-profile-overrides-path') + 1])
    source_hash, installed_hash = replay.artifact_hashes(profile, selected)
    assert source_hash == installed_hash == hashlib.sha256(profile.read_bytes()).hexdigest()


def test_changed_installed_profile_is_rejected_before_playback(tmp_path):
    source, installed = tmp_path / 'source.yaml', tmp_path / 'installed.yaml'
    source.write_text('depth: 4096\n')
    installed.write_text('depth: 10\n')
    with pytest.raises(ValueError, match='installed artifact differs from source'):
        replay.artifact_hashes(source, installed)
