"""Fingerprint reuse must retain source and cache-directory invalidation."""
import os
from pathlib import Path

from aims_mpcc import rollout_native


def test_hot_rollout_path_reads_source_once_and_detects_a_changed_source(tmp_path,monkeypatch):
    source=tmp_path/'rollout.c';source.write_text('first source')
    monkeypatch.setattr(rollout_native,'_KERNEL_SOURCE',source)
    original=Path.read_bytes;reads=[]
    def counted(path):
        if path==source:reads.append(path)
        return original(path)
    monkeypatch.setattr(Path,'read_bytes',counted)
    first=rollout_native.kernel_path(tmp_path/'cache')
    assert rollout_native.kernel_path(tmp_path/'cache')==first
    assert len(reads)==1
    previous=source.stat()
    source.write_text('other source')
    os.utime(source,ns=(previous.st_atime_ns,previous.st_mtime_ns+1_000_000))
    assert rollout_native.kernel_path(tmp_path/'cache')!=first
    assert len(reads)==2


def test_runtime_cache_directory_change_does_not_reuse_the_wrong_library(tmp_path,monkeypatch):
    source=tmp_path/'rollout.c';source.write_text('source')
    monkeypatch.setattr(rollout_native,'_KERNEL_SOURCE',source)
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'first'))
    first=rollout_native.kernel_path()
    monkeypatch.setenv('AIMS_MPCC_ROLLOUT_DIR',str(tmp_path/'second'))
    second=rollout_native.kernel_path()
    assert first.parent!=second.parent and first.name==second.name
