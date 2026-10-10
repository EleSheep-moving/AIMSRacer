#!/usr/bin/env bash
# Prepare/build the field-pinned numerical runtime; never remove prior trees.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
workspace="$(cd -- "${script_dir}/.." && pwd -P)"
build_jobs=2
while (($#)); do
  case "$1" in
    --workspace) workspace="$(realpath -m -- "$2")"; shift 2 ;;
    --jobs) build_jobs="$2"; shift 2 ;;
    -h|--help)
      printf '%s\n' 'Usage: tools/setup_acados.sh [--workspace ROOT] [--jobs N]' \
        'Builds pinned acados/BLASFEO/HPIPM into ROOT/dependencies/work/acados/install.' \
        'ARM: Cortex A57 BLASFEO; x86: GENERIC BLASFEO. HPIPM GENERIC; OpenMP OFF.'
      exit 0 ;;
    *) printf 'Unknown argument: %s\n' "$1" >&2; exit 2 ;;
  esac
done
[[ "$build_jobs" =~ ^[1-9][0-9]*$ ]] || { printf 'Positive --jobs required\n' >&2; exit 2; }
python3 "${script_dir}/setup_dependencies.py" --workspace "$workspace" --only acados --skip-sdk
python3 - "$workspace" "$build_jobs" "${script_dir}/../dependencies/manifest.json" <<'PY'
import json, platform, sys
from pathlib import Path
workspace=Path(sys.argv[1]);jobs=sys.argv[2]
# Load the implementation from its actual script directory; it may prepare a
# different --workspace without requiring tools to be copied into that workspace.
import importlib.util
manifest_path=Path(sys.argv[3]).resolve()
module_spec=importlib.util.spec_from_file_location('setup_dependencies',manifest_path.parent.parent/'tools/setup_dependencies.py')
tool=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(tool)
manifest=json.loads(manifest_path.read_text())
architecture=platform.machine()
if architecture not in manifest['acados_cmake']:
    raise SystemExit(f'Unsupported acados architecture: {architecture}')
base=workspace/'dependencies/work/acados'
source,build,prefix=base/'source',base/'build',base/'install'
tool.check_build_source(build,source)
settings={**manifest['acados_cmake']['common'],**manifest['acados_cmake'][architecture]}
settings['CMAKE_INSTALL_PREFIX']=str(prefix)
arguments=[f'-D{key}={value}' for key,value in settings.items()]
# Older CMake projects require compatibility when the installed CMake is >=4.
arguments.append('-DCMAKE_POLICY_VERSION_MINIMUM=3.5')
tool.run('cmake','-S',str(source),'-B',str(build),*arguments)
tool.run('cmake','--build',str(build),'--parallel',jobs)
tool.run('cmake','--install',str(build))
# The generated-bundle builder accepts this installed revision marker. Its
# source is checked again after compilation; never derive it from a parent git.
verified=tool.prepare_repository(source,manifest['native_sources']['acados'])
(prefix/'lib/git_commit_hash').write_text(verified['commit']+'\n')
generator=tool.prepare_acados_generator(source,build,prefix)
libraries=[prefix/'lib'/name for name in ['libacados.so','libblasfeo.so','libhpipm.so']]
if not all(path.is_file() for path in libraries):
    raise SystemExit('acados installation omitted a required shared library')
receipt=dict(architecture=architecture,source_commit=manifest['native_sources']['acados']['commit'],
             submodule_pins=manifest['native_sources']['acados']['submodules'],cmake=settings,
             libraries={str(path):tool.sha256(path) for path in libraries},generator=generator)
(base/'runtime-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(f'acados runtime prepared: {prefix}')
print(f'Offline generator module: {source}/interfaces/acados_template')
PY
