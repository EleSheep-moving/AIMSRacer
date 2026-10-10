"""Check source preservation, complete patch certificates, and non-destructive preparation."""
import hashlib
import importlib.util
import subprocess
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[1] / 'setup_dependencies.py'


def module():
    assert TOOL.exists(), 'Unified dependency preparation tool is not implemented'
    spec = importlib.util.spec_from_file_location('setup_dependencies', TOOL)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def git(root, *args):
    return subprocess.check_output(['git', '-C', str(root), *args], text=True).strip()


@pytest.fixture
def repo(tmp_path):
    root = tmp_path / 'source'
    root.mkdir()
    git(root, 'init', '-q')
    git(root, 'config', 'user.email', 'fixture@example.invalid')
    git(root, 'config', 'user.name', 'Fixture')
    (root / 'data.txt').write_text('baseline\n')
    git(root, 'add', '.')
    git(root, 'commit', '-qm', 'baseline')
    return root


def test_existing_wrong_revision_is_preserved(repo):
    tool = module()
    original = (repo / 'data.txt').read_bytes()
    with pytest.raises(tool.PreparationError, match='revision'):
        tool.prepare_repository(repo, {'commit': '0' * 40, 'url': 'unused'})
    assert (repo / 'data.txt').read_bytes() == original
    assert git(repo, 'status', '--porcelain') == ''


def test_existing_source_drift_is_preserved(repo):
    tool = module()
    (repo / 'data.txt').write_text('operator work\n')
    with pytest.raises(tool.PreparationError, match='delta|dirty'):
        tool.prepare_repository(repo, {'commit': git(repo, 'rev-parse', 'HEAD'), 'url': 'unused'})
    assert (repo / 'data.txt').read_text() == 'operator work\n'


def test_patch_is_exact_idempotent_and_rejects_additional_changes(repo, tmp_path):
    tool = module()
    commit = git(repo, 'rev-parse', 'HEAD')
    (repo / 'data.txt').write_text('patched\n')
    (repo / 'new.txt').write_text('added by audited patch\n')
    git(repo, 'add', '-N', 'new.txt')
    patch = tmp_path / 'audited.patch'
    patch.write_bytes(subprocess.check_output(['git', '-C', str(repo), 'diff', 'HEAD', '--binary']))
    git(repo, 'reset', '--hard', '-q', 'HEAD')
    spec = {'commit': commit, 'url': 'unused', 'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest()}
    tool.prepare_repository(repo, spec, patch)
    tool.prepare_repository(repo, spec, patch)
    assert subprocess.check_output(['git', '-C', str(repo), 'diff', 'HEAD', '--binary']) == patch.read_bytes()
    (repo / 'unexpected.txt').write_text('preserve me\n')
    with pytest.raises(tool.PreparationError, match='untracked|unexpected'):
        tool.prepare_repository(repo, spec, patch)
    assert (repo / 'unexpected.txt').read_text() == 'preserve me\n'


def test_bad_patch_checksum_never_changes_source(repo, tmp_path):
    tool = module()
    patch = tmp_path / 'bad.patch'
    patch.write_text('not a patch\n')
    with pytest.raises(tool.PreparationError, match='checksum'):
        tool.prepare_repository(repo, {'commit': git(repo, 'rev-parse', 'HEAD'), 'url': 'unused', 'patch_sha256': '0'*64}, patch)
    assert git(repo, 'status', '--porcelain') == ''


def test_parent_repository_is_not_accepted_as_dependency(repo):
    tool = module()
    child = repo / 'nested'
    child.mkdir()
    with pytest.raises(tool.PreparationError, match='root|repository'):
        tool.prepare_repository(child, {'commit': git(repo, 'rev-parse', 'HEAD'), 'url': 'unused'})


def test_livox_generates_only_ignored_ros2_files_and_preserves_conflicts(repo):
    tool = module()
    (repo / '.gitignore').write_text('package.xml\nlaunch/\n')
    (repo / 'package_ROS2.xml').write_text('<package><name>livox_ros_driver2</name></package>\n')
    (repo / 'launch_ROS2').mkdir()
    (repo / 'launch_ROS2/live.launch.py').write_text('ROS2 launch\n')
    tool.prepare_livox_ros2(repo)
    tool.prepare_livox_ros2(repo)
    assert (repo / 'package.xml').read_bytes() == (repo / 'package_ROS2.xml').read_bytes()
    assert not (repo / 'launch').exists()
    (repo / 'package.xml').write_text('existing different config\n')
    with pytest.raises(tool.PreparationError, match='existing'):
        tool.prepare_livox_ros2(repo)
    assert (repo / 'package.xml').read_text() == 'existing different config\n'


def test_existing_sdk_records_library_and_all_headers_without_building(tmp_path):
    tool = module()
    (tmp_path / 'lib').mkdir()
    (tmp_path / 'include').mkdir()
    for rel in ['lib/liblivox_lidar_sdk_shared.so', 'include/livox_lidar_api.h', 'include/livox_lidar_cfg.h', 'include/livox_lidar_def.h']:
        (tmp_path / rel).write_bytes(rel.encode())
    result = tool.existing_sdk(tmp_path)
    assert result['complete'] is True
    assert len(result['files']) == 4
    assert all(len(x['sha256']) == 64 for x in result['files'])


@pytest.mark.parametrize('name,files', [
    ('Sophus', ['include/sophus/se3.hpp', 'share/sophus/cmake/SophusConfig.cmake']),
    ('CppLinuxSerial', ['include/CppLinuxSerial/SerialPort.hpp',
                        'lib/cmake/CppLinuxSerial/CppLinuxSerialConfig.cmake', 'lib/libCppLinuxSerial.a']),
])
def test_existing_native_install_is_recorded_without_build(tmp_path, monkeypatch, name, files):
    tool = module()
    for rel in files:
        p = tmp_path / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(rel)
    monkeypatch.setattr(tool, 'run', lambda *a, **k: pytest.fail('existing install must not build'))
    result = tool.prepare_system_dependency(tmp_path/'workspace', name, {}, tmp_path, 2)
    assert result['status'] == 'existing_installation_recorded'
    assert result['source_commit_proven'] is False
    assert len(result['files']) == len(files)


def test_partial_native_install_is_never_overwritten(tmp_path):
    tool = module()
    p = tmp_path/'include/sophus/se3.hpp'
    p.parent.mkdir(parents=True)
    p.write_text('existing header')
    with pytest.raises(tool.PreparationError, match='partial'):
        tool.prepare_system_dependency(tmp_path/'workspace', 'Sophus', {}, tmp_path, 2)
    assert p.read_text() == 'existing header'


def test_acados_generator_requires_real_cmake_metadata(tmp_path):
    tool = module()
    with pytest.raises(tool.PreparationError, match='link_libs'):
        tool.prepare_acados_generator(tmp_path/'source', tmp_path/'build', tmp_path/'install')

def test_privileged_install_only_elevates_the_cmake_install_step(tmp_path, monkeypatch):
    tool=module();calls=[]
    prefix=tmp_path/'system-prefix';prefix.mkdir()
    monkeypatch.setattr(tool.os,'access',lambda p,mode:False)
    monkeypatch.setattr(tool.shutil,'which',lambda name:'/usr/bin/'+name)
    monkeypatch.setattr(tool,'run',lambda *args,**kwargs:calls.append(args))
    tool.install_build(tmp_path/'user-owned-build',prefix)
    assert calls==[('sudo','/usr/bin/cmake','--install',str(tmp_path/'user-owned-build'))]

def test_writable_install_prefix_does_not_use_sudo(tmp_path, monkeypatch):
    tool=module();calls=[]
    monkeypatch.setattr(tool.os,'access',lambda p,mode:True)
    monkeypatch.setattr(tool.shutil,'which',lambda name:'/usr/bin/'+name)
    monkeypatch.setattr(tool,'run',lambda *args,**kwargs:calls.append(args))
    tool.install_build(tmp_path/'build',tmp_path/'user-prefix')
    assert calls==[('/usr/bin/cmake','--install',str(tmp_path/'build'))]
