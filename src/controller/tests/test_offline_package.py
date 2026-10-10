"""Installed commands and the default vehicle profile must match the field stack."""
import configparser
from pathlib import Path
import shutil
import subprocess
import sys

from aims_mpcc.io import load_config


def test_installed_commands_only_record_and_prepare_paths(tmp_path):
    source = Path(__file__).parents[1]
    copy = tmp_path / 'controller'
    shutil.copytree(source, copy, ignore=shutil.ignore_patterns(
        '__pycache__', 'build', 'dist', '*.egg-info'))
    result = subprocess.run([sys.executable, 'setup.py', 'egg_info'],
                            cwd=copy, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    metadata = configparser.ConfigParser()
    metadata.read(next(copy.glob('*.egg-info/entry_points.txt')))
    assert dict(metadata['console_scripts']) == {
        'record_path': 'aims_mpcc.recorder:main',
        'prepare_path': 'aims_mpcc.prepare:main',
    }


def test_default_vehicle_matches_the_archived_field_configuration():
    root = Path(__file__).parents[1]
    actual = load_config(root / 'config' / 'vehicle.yaml')
    archived = load_config(Path(__file__).parent / 'fixtures' /
                           'vehicle_field_20261010_v35.yaml')
    assert actual == archived
