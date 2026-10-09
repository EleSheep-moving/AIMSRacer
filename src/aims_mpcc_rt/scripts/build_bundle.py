#!/usr/bin/env python3
"""Compile already generated C on the target; no CasADi/template imports."""
import argparse
import hashlib
import json
from pathlib import Path
import platform
import subprocess


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(root, install, cc='gcc'):
    root = Path(root).resolve(); install = Path(install).resolve()
    manifest = json.loads((root / 'manifest.json').read_text())
    commit_file=install/'lib/git_commit_hash'
    commit=commit_file.read_text().strip() if commit_file.is_file() else ''
    if commit in ('','unknown'):
        # acados writes this marker in the source tree rather than installing
        # it. A pinned checkout beside the install is an alternative proof.
        commit=subprocess.check_output(['git','-C',str(install.parent),'rev-parse','HEAD'],text=True).strip()
        if subprocess.check_output(['git','-C',str(install.parent),'diff','HEAD','--'],text=True).strip():
            raise ValueError('target acados checkout contains tracked source changes')
    if commit != manifest['acados_commit']:
        raise ValueError('target acados commit differs from exported OCP')
    for name, expected in manifest['files'].items():
        if sha(root / name) != expected:
            raise ValueError('source artifact hash mismatch: ' + name)
    # Remove existing .o files so an ARM rebuild cannot retain x86 objects.
    subprocess.run(['make', '-C', str(root / 'generated'), 'clean'], check=True)
    # The generated Makefile embeds exporter paths; command-line overrides
    # allow rebuilding the immutable C sources with target acados libraries.
    subprocess.run(['make', '-C', str(root / 'generated'), '-j2', 'shared_lib',
                    'CC=' + cc, 'INCLUDE_PATH=' + str(install / 'include'),
                    'LIB_PATH=' + str(install / 'lib')], check=True)
    subprocess.run([cc, '-shared', '-fPIC', '-O2', str(root/'bridge.c'), str(root/'model_eval.c'),
                    '-I'+str(root), '-I'+str(install/'include'), '-I'+str(install/'include/acados'),
                    '-I'+str(install/'include/blasfeo/include'), '-I'+str(install/'include/hpipm/include'),
                    '-L'+str(root/'generated'), '-L'+str(install/'lib'),
                    '-lacados_ocp_solver_aims_runtime', '-lacados', '-lhpipm', '-lblasfeo', '-lm',
                    '-Wl,--disable-new-dtags,-rpath,$ORIGIN/generated:'+str(install/'lib'),
                    '-o', str(root/'libaims_mpcc_bundle.so')], check=True)
    dependencies = {str(p): sha(p) for p in [install/'lib/libacados.so', install/'lib/libhpipm.so', install/'lib/libblasfeo.so']}
    native = dict(schema_version=1, machine=platform.machine(), system=platform.system(),
                  source_manifest_sha256=sha(root/'manifest.json'),
                  acados_commit=commit, source_fingerprint=manifest['fingerprint'], dependencies=dependencies,
                  libraries={name: sha(root/name) for name in ['libaims_mpcc_bundle.so', 'generated/libacados_ocp_solver_aims_runtime.so']})
    (root/'native_manifest.json').write_text(json.dumps(native, sort_keys=True, indent=2)+'\n')


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--acados-install', required=True)
    parser.add_argument('--cc', default='gcc')
    parser.add_argument('--bundle', default=str(Path(__file__).resolve().parent))
    args=parser.parse_args();build(args.bundle,args.acados_install,args.cc)
