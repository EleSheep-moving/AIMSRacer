"""Destination memoization must retain live roots and source fingerprints."""
import hashlib
import os
from pathlib import Path

import pytest

from aims_mpcc import rollout_native


@pytest.fixture
def source(tmp_path,monkeypatch):
    file=tmp_path/'rollout.c';file.write_text('original kernel')
    monkeypatch.setattr(rollout_native,'_KERNEL_SOURCE',file)
    return file


def forbidden_home():
    raise AssertionError('configured rollout root must not resolve home')


def test_configured_environment_does_not_eagerly_resolve_home(source,tmp_path,monkeypatch):
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'configured'))
    monkeypatch.setattr(Path,'home',forbidden_home)
    assert rollout_native.kernel_path().parent==tmp_path/'configured'


def test_explicit_directory_wins_over_live_environment(source,tmp_path,monkeypatch):
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'environment'))
    monkeypatch.setattr(Path,'home',forbidden_home)
    assert rollout_native.kernel_path(tmp_path/'explicit').parent==tmp_path/'explicit'


def test_empty_environment_retains_current_directory_semantics(source,monkeypatch):
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR','')
    monkeypatch.setattr(Path,'home',forbidden_home)
    assert rollout_native.kernel_path().parent==Path('.')


def test_unset_environment_resolves_live_home(source,tmp_path,monkeypatch):
    monkeypatch.delenv('AIMS_MPCC_ROLLOUT_DIR',raising=False)
    homes=iter([tmp_path/'first_home',tmp_path/'second_home'])
    monkeypatch.setattr(Path,'home',lambda:next(homes))
    first=rollout_native.kernel_path();second=rollout_native.kernel_path()
    assert first.parent==tmp_path/'first_home'/'.cache/aims_mpcc/rollout'
    assert second.parent==tmp_path/'second_home'/'.cache/aims_mpcc/rollout'


def test_live_environment_change_cannot_reuse_old_destination(source,tmp_path,monkeypatch):
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'first'))
    first=rollout_native.kernel_path()
    assert rollout_native.kernel_path() is first
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'second'))
    second=rollout_native.kernel_path()
    assert second.parent==tmp_path/'second' and second!=first
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'first'))
    assert rollout_native.kernel_path() is first


def test_source_stat_and_machine_are_read_each_call_even_when_path_is_memoized(source,tmp_path,monkeypatch):
    calls=[];stat=Path.stat
    def counted_stat(self,*args,**kwargs):
        if self==source:calls.append('source_stat')
        return stat(self,*args,**kwargs)
    def machine():
        calls.append('machine')
        return 'proof_arch'
    monkeypatch.setattr(Path,'stat',counted_stat)
    monkeypatch.setattr(rollout_native.platform,'machine',machine)
    first=rollout_native.kernel_path(tmp_path)
    assert rollout_native.kernel_path(tmp_path) is first
    assert calls==['source_stat','machine','source_stat','machine']


def test_source_edit_invalidates_immutable_destination(source,tmp_path):
    first=rollout_native.kernel_path(tmp_path);before=source.stat()
    source.write_text('modified kernel')
    os.utime(source,ns=(before.st_atime_ns,before.st_mtime_ns+1))
    second=rollout_native.kernel_path(tmp_path)
    digest=hashlib.sha256(source.read_bytes()+rollout_native.platform.machine().encode()).hexdigest()
    assert second.name==f'rollout-{digest}.so'
    assert second!=first and rollout_native.kernel_path(tmp_path) is second


def test_machine_change_selects_its_own_source_fingerprint(source,tmp_path,monkeypatch):
    monkeypatch.setattr(rollout_native.platform,'machine',lambda:'first_arch')
    first=rollout_native.kernel_path(tmp_path)
    monkeypatch.setattr(rollout_native.platform,'machine',lambda:'second_arch')
    second=rollout_native.kernel_path(tmp_path)
    assert first!=second
    digest=hashlib.sha256(source.read_bytes()+b'second_arch').hexdigest()
    assert second.name==f'rollout-{digest}.so'


def test_pathlike_and_empty_explicit_directory_preserve_precedence(source,tmp_path,monkeypatch):
    class Directory:
        def __fspath__(self):return str(tmp_path/'pathlike')
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'environment'))
    assert rollout_native.kernel_path(Directory()).parent==tmp_path/'pathlike'
    assert rollout_native.kernel_path('').parent==tmp_path/'environment'
    assert rollout_native.kernel_path(Path('.')).parent==Path('.')
