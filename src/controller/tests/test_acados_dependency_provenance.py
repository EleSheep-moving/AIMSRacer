"""Dependency provenance follows the libraries selected for offline export."""
import hashlib
import sys

import pytest

from aims_mpcc.acados_backend import _dependency_versions


LIBRARIES = ('libacados.so', 'libhpipm.so', 'libblasfeo.so')


def write_libraries(libroot, label):
    libroot.mkdir(parents=True)
    expected = {}
    for name in LIBRARIES:
        data = f'{label}:{name}'.encode()
        (libroot / name).write_bytes(data)
        expected[name] = hashlib.sha256(data).hexdigest()
    return expected


@pytest.mark.parametrize('relative', ['install/lib', 'install-x86/lib', 'lib'])
def test_source_library_layouts_are_fingerprinted(monkeypatch, tmp_path, relative):
    source = tmp_path / 'source'
    libroot = source / relative
    expected = write_libraries(libroot, relative)
    monkeypatch.setenv('ACADOS_SOURCE_DIR', str(source))
    monkeypatch.delenv('ACADOS_INSTALL_DIR', raising=False)

    versions = _dependency_versions()

    assert versions['acados_lib_path'] == str(libroot.resolve())
    assert versions['libraries'] == expected


def test_source_install_precedes_legacy_layout(monkeypatch, tmp_path):
    source = tmp_path / 'source'
    expected = write_libraries(source / 'install' / 'lib', 'current')
    write_libraries(source / 'install-x86' / 'lib', 'legacy')
    write_libraries(source / 'lib', 'in-source')
    monkeypatch.setenv('ACADOS_SOURCE_DIR', str(source))
    monkeypatch.delenv('ACADOS_INSTALL_DIR', raising=False)

    versions = _dependency_versions()

    assert versions['acados_lib_path'] == str((source / 'install' / 'lib').resolve())
    assert versions['libraries'] == expected


def test_explicit_install_prefix_overrides_source_layouts(monkeypatch, tmp_path):
    source = tmp_path / 'source'
    write_libraries(source / 'install' / 'lib', 'source-install')
    write_libraries(source / 'install-x86' / 'lib', 'legacy')
    prefix = tmp_path / 'sdk-install'
    expected = write_libraries(prefix / 'lib', 'explicit')
    monkeypatch.setenv('ACADOS_SOURCE_DIR', str(source))
    monkeypatch.setenv('ACADOS_INSTALL_DIR', str(prefix))

    versions = _dependency_versions()

    assert versions['acados_lib_path'] == str((prefix / 'lib').resolve())
    assert versions['libraries'] == expected


def test_explicit_install_is_fingerprinted_without_source(monkeypatch, tmp_path):
    prefix = tmp_path / 'sdk-install'
    expected = write_libraries(prefix / 'lib', 'explicit')
    monkeypatch.delenv('ACADOS_SOURCE_DIR', raising=False)
    monkeypatch.setenv('ACADOS_INSTALL_DIR', str(prefix))
    monkeypatch.setitem(sys.modules, 'acados_template', None)

    versions = _dependency_versions()

    assert versions['acados_source'] is None
    assert versions['acados_lib_path'] == str((prefix / 'lib').resolve())
    assert versions['libraries'] == expected
