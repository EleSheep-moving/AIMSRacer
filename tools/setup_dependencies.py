#!/usr/bin/env python3
"""Prepare the field-pinned dependencies without replacing existing source or build trees.

Default: four ROS sources in workspace/src, Sophus, CppLinuxSerial and Livox SDK
(record existing installations; build only when absent).
No controller or hardware process is started.
"""
import argparse
import ast
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import typing
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


class PreparationError(RuntimeError):
    """Source or generated files differ from the explicitly pinned dependency."""


def run(*args, cwd=None, check=True):
    env = dict(os.environ, GIT_TERMINAL_PROMPT='0')
    result = subprocess.run(args, cwd=cwd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, check=False)
    if check and result.returncode:
        raise PreparationError(f'{args[0]} failed ({result.returncode}): '
                               f'{result.stderr.decode(errors="replace").strip()}')
    return result


def install_build(build, prefix):
    """Keep source/build/receipts user-owned; elevate only system installation."""
    destination = Path(prefix)
    while not destination.exists():
        destination = destination.parent
    command = [shutil.which('cmake') or 'cmake', '--install', str(build)]
    if not os.access(destination, os.W_OK):
        if shutil.which('sudo') is None:
            raise PreparationError(f'{prefix}: system installation needs write permission; no sudo available')
        command.insert(0, 'sudo')
    return run(*command)


def git(directory, *args):
    return run('git', '-C', str(directory), *args).stdout


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def prepare_repository(target, spec, patch=None):
    target = Path(target)
    if patch is not None:
        patch = Path(patch)
        if sha256(patch) != spec['patch_sha256']:
            raise PreparationError(f'{patch}: patch checksum differs')
    if not target.exists():
        target.parent.mkdir(parents=True, exist_ok=True)
        run('git', 'clone', '--no-checkout', spec['url'], str(target))
        run('git', '-C', str(target), 'checkout', '--detach', spec['commit'])
    root = git(target, 'rev-parse', '--show-toplevel').decode().strip()
    if Path(root).resolve() != target.resolve():
        raise PreparationError(f'{target}: dependency repository root differs')
    head = git(target, 'rev-parse', 'HEAD').decode().strip()
    if head != spec['commit']:
        raise PreparationError(f'{target}: existing revision differs; source preserved')
    untracked = git(target, 'ls-files', '--others', '--exclude-standard').decode().strip()
    if untracked:
        raise PreparationError(f'{target}: unexpected untracked source: {untracked}')
    delta = git(target, 'diff', 'HEAD', '--binary')
    if patch is None:
        if delta or git(target, 'status', '--porcelain').strip():
            raise PreparationError(f'{target}: existing dirty source delta; source preserved')
    else:
        expected = patch.read_bytes()
        if delta != expected:
            if delta or git(target, 'status', '--porcelain').strip():
                raise PreparationError(f'{target}: complete source delta differs from audited patch')
            run('git', '-C', str(target), 'apply', '--check', str(patch.resolve()))
            run('git', '-C', str(target), 'apply', str(patch.resolve()))
            added = re.findall(r'^diff --git a/(.+) b/\1\nnew file mode',
                               expected.decode(), re.MULTILINE)
            if added:
                run('git', '-C', str(target), 'add', '-N', '--', *added)
            delta = git(target, 'diff', 'HEAD', '--binary')
            if delta != expected:
                raise PreparationError(f'{target}: complete source delta differs after patch')
    return dict(path=str(target.resolve()),commit=head,url=spec['url'],
                patch_sha256=sha256(patch) if patch else None,
                complete_diff_sha256=hashlib.sha256(delta).hexdigest())


def prepare_livox_ros2(directory):
    """Generate only the upstream ignored ROS2 package manifest."""
    directory = Path(directory)
    copies = [(directory/'package_ROS2.xml', directory/'package.xml')]
    # Preflight every file before writing anything: conflicts stay untouched.
    for src, dest in copies:
        if not src.is_file():
            raise PreparationError(f'{src}: required ROS2 source missing')
        ignored = run('git','-C',str(directory),'check-ignore','--quiet',str(dest.relative_to(directory)),check=False)
        if ignored.returncode:
            raise PreparationError(f'{dest}: ROS2 generated path is not ignored by upstream')
        if dest.exists() and (not dest.is_file() or dest.read_bytes()!=src.read_bytes()):
            raise PreparationError(f'{dest}: existing generated file differs; preserved')
    for src, dest in copies:
        if not dest.exists():
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src,dest)
    return dict(generated_files=[str(dest) for _,dest in copies],cmake_arguments=['-DROS_EDITION=ROS2','-DDISTRO_ROS=humble'])


def existing_sdk(prefix):
    prefix = Path(prefix)
    paths = [prefix/'lib/liblivox_lidar_sdk_shared.so'] + [prefix/'include'/name for name in
              ['livox_lidar_api.h','livox_lidar_cfg.h','livox_lidar_def.h']]
    return dict(prefix=str(prefix),complete=all(p.is_file() for p in paths),
                files=[dict(path=str(p),sha256=sha256(p)) for p in paths if p.is_file()],
                source_commit_proven=False)


def check_build_source(build, source):
    """Keep an existing CMake tree if and only if its recorded source matches."""
    cache = Path(build)/'CMakeCache.txt'
    if cache.exists():
        match = re.search(r'^CMAKE_HOME_DIRECTORY:INTERNAL=(.+)$',cache.read_text(),re.MULTILINE)
        if not match or Path(match[1]).resolve()!=Path(source).resolve():
            raise PreparationError(f'{build}: existing CMake source differs; tree preserved')


def existing_system_dependency(name, prefix):
    prefix = Path(prefix)
    if name == 'Sophus':
        headers = sorted((prefix/'include/sophus').glob('*.hpp'))
        configs = sorted(prefix.glob('share/sophus/cmake/Sophus*.cmake')) + sorted(prefix.glob('lib/cmake/Sophus/Sophus*.cmake'))
        complete = (prefix/'include/sophus/se3.hpp').is_file() and any(p.name=='SophusConfig.cmake' for p in configs)
        paths = headers + configs
    elif name == 'CppLinuxSerial':
        headers = sorted((prefix/'include/CppLinuxSerial').glob('*.hpp'))
        configs = sorted(prefix.glob('lib/cmake/CppLinuxSerial/*.cmake'))
        libraries = sorted(prefix.glob('lib/libCppLinuxSerial.*'))
        complete = (prefix/'include/CppLinuxSerial/SerialPort.hpp').is_file() and any(p.name=='CppLinuxSerialConfig.cmake' for p in configs) and bool(libraries)
        paths = headers + configs + libraries
    else:
        raise PreparationError(f'Unknown system dependency: {name}')
    return dict(prefix=str(prefix), complete=complete, source_commit_proven=False,
                files=[dict(path=str(p),sha256=sha256(p)) for p in paths if p.is_file()])


def prepare_system_dependency(workspace, name, spec, prefix, jobs):
    receipt = existing_system_dependency(name, prefix)
    if receipt['complete']:
        receipt['status'] = 'existing_installation_recorded'
        return receipt
    if receipt['files']:
        raise PreparationError(f'{name}: partial existing installation at {prefix}; preserved')
    base = workspace/'dependencies/work'/name
    source, build = base/'source', base/'build'
    source_receipt = prepare_repository(source, spec)
    check_build_source(build, source)
    settings = dict(CMAKE_BUILD_TYPE='Release', CMAKE_INSTALL_PREFIX=str(prefix),
                    CMAKE_POLICY_VERSION_MINIMUM='3.5')
    if name == 'Sophus':
        settings.update(BUILD_SOPHUS_EXAMPLES='OFF', BUILD_SOPHUS_TESTS='OFF',
                        SOPHUS_INSTALL='ON', SOPHUS_USE_BASIC_LOGGING='ON')
    else:
        settings.update(BUILD_TESTS='OFF')
    run('cmake','-S',str(source),'-B',str(build),*[f'-D{k}={v}' for k,v in settings.items()])
    run('cmake','--build',str(build),'--parallel',str(jobs))
    install_build(build, prefix)
    receipt = existing_system_dependency(name, prefix)
    if not receipt['complete']:
        raise PreparationError(f'{name}: installation omitted required artifacts')
    receipt.update(status='built_from_pinned_source', source=source_receipt,
                   source_commit_proven=True, cmake=settings)
    return receipt


def prepare_acados_generator(source, build, prefix):
    """Install CMake's actual metadata and run the pinned SDK's Tera downloader.

    Only the standalone downloader definitions are loaded from the SDK's AST,
    so preparing native libraries does not require its unrelated Python solver
    dependencies. URL, version and architecture selection remain SDK-owned.
    """
    source, build, prefix = Path(source), Path(build), Path(prefix)
    # This pinned CMake writes relative to its source directory, not build.
    metadata = source/'lib/link_libs.json'
    if not metadata.is_file():
        raise PreparationError(f'{metadata}: CMake-generated link_libs.json missing')
    json.loads(metadata.read_text())
    (prefix/'lib').mkdir(parents=True,exist_ok=True)
    shutil.copy2(metadata,prefix/'lib/link_libs.json')
    utility = source/'interfaces/acados_template/acados_template/utils.py'
    wanted = {'TERA_DEFAULT_VERSION','PLATFORM2TERA','get_acados_path',
              'get_tera_exec_path','get_binary_ext','get_architecture_amd64_arm64','get_tera'}
    tree = ast.parse(utility.read_text(),filename=str(utility))
    statements = [node for node in tree.body if
                  (isinstance(node,ast.FunctionDef) and node.name in wanted) or
                  (isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id in wanted for t in node.targets))]
    scope = dict(os=os,sys=sys,platform=platform,shutil=shutil,urllib=urllib,
                 Optional=typing.Optional)
    exec(compile(ast.Module(body=statements,type_ignores=[]),str(utility),'exec'),scope)
    renderer = source/'bin/t_renderer'
    if renderer.exists() and (not renderer.is_file() or not os.access(renderer,os.X_OK)):
        raise PreparationError(f'{renderer}: existing renderer is not executable; preserved')
    previous = {key:os.environ.get(key) for key in ('ACADOS_SOURCE_DIR','TERA_PATH')}
    try:
        os.environ['ACADOS_SOURCE_DIR']=str(source)
        os.environ['TERA_PATH']=str(renderer)
        scope['get_tera'](force_download=not renderer.exists())
    finally:
        for key,value in previous.items():
            if value is None:os.environ.pop(key,None)
            else:os.environ[key]=value
    # Exercise the downloaded binary with an actual template, offline.
    smoke = build/'tera-smoke'
    smoke.mkdir(exist_ok=True)
    (smoke/'template.in').write_text('{{ value }}\n')
    (smoke/'values.json').write_text('{"value":"acados-generator-ready"}\n')
    run(str(renderer),str(smoke/'*.in'),'template.in',str(smoke/'values.json'),str(smoke/'rendered.txt'))
    if (smoke/'rendered.txt').read_text().strip()!='acados-generator-ready':
        raise PreparationError('Tera renderer smoke output differs')
    return dict(link_libs_sha256=sha256(metadata),renderer_path=str(renderer),
                renderer_sha256=sha256(renderer),sdk_tera_version=scope['TERA_DEFAULT_VERSION'])


def prepare_sdk(workspace, spec, prefix, jobs):
    receipt = existing_sdk(prefix)
    if receipt['complete']:
        receipt['status']='existing_installation_recorded'
        return receipt
    if receipt['files']:
        raise PreparationError(f'Livox SDK: partial existing installation at {prefix}; preserved')
    base = workspace/'dependencies/work/Livox-SDK2'
    source, build = base/'source', base/'build'
    source_receipt = prepare_repository(source,spec)
    check_build_source(build,source)
    run('cmake','-S',str(source),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release',
        f'-DCMAKE_INSTALL_PREFIX={prefix}','-DCMAKE_POLICY_VERSION_MINIMUM=3.5')
    run('cmake','--build',str(build),'--parallel',str(jobs))
    install_build(build, prefix)
    receipt = existing_sdk(prefix)
    if not receipt['complete']:
        raise PreparationError('SDK install did not provide the required library and headers')
    receipt.update(status='built_from_pinned_source',source=source_receipt,source_commit_proven=True)
    return receipt


def prepare_acados_source(workspace, spec):
    source=workspace/'dependencies/work/acados/source'
    receipt=prepare_repository(source,spec)
    # Inspect already populated submodules before an update, preserving drift.
    for rel,commit in spec['submodules'].items():
        child=source/rel
        if (child/'.git').exists():
            prepare_repository(child,{'commit':commit,'url':f'{spec["url"]}:{rel}'})
    run('git','-C',str(source),'submodule','update','--init','--recursive',*spec['submodules'])
    receipt['submodules']={rel:prepare_repository(source/rel,{'commit':commit,'url':rel})
                           for rel,commit in spec['submodules'].items()}
    return receipt


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[1])
    parser.add_argument('--only',nargs='+',help='Select ROS packages, Sophus, CppLinuxSerial, sdk, or acados')
    parser.add_argument('--skip-sdk',action='store_true',help='Do not inspect, build, or install Livox SDK')
    parser.add_argument('--sdk-install-prefix',type=Path,default=Path('/usr/local'))
    parser.add_argument('--system-install-prefix',type=Path,default=Path('/usr/local'),help='Sophus and CppLinuxSerial installation/search prefix')
    parser.add_argument('--jobs',type=int,default=2)
    args=parser.parse_args(argv)
    if args.jobs<1:parser.error('--jobs must be positive')
    workspace=args.workspace.expanduser().resolve()
    manifest_path=Path(__file__).resolve().parents[1]/'dependencies/manifest.json'
    manifest=json.loads(manifest_path.read_text())
    if manifest.get('schema_version')!=1:raise PreparationError('Unsupported dependency manifest schema')
    names=args.only or list(manifest['ros_sources'])+list(manifest['existing_system_source_pins'])
    allowed=set(manifest['ros_sources'])|set(manifest['existing_system_source_pins'])|{'sdk','acados'}
    if set(names)-allowed:parser.error('Unknown dependencies: '+', '.join(sorted(set(names)-allowed)))
    destination=workspace/'dependencies/work/preparation-receipt.json'
    (workspace/'dependencies').mkdir(parents=True,exist_ok=True)
    (workspace/'dependencies/COLCON_IGNORE').touch(exist_ok=True)
    receipt=dict(created_utc=datetime.now(timezone.utc).isoformat(),workspace=str(workspace),
                 manifest_sha256=sha256(manifest_path),sources={})
    if destination.exists():
        previous=json.loads(destination.read_text())
        if previous.get('manifest_sha256')==receipt['manifest_sha256'] and previous.get('workspace')==str(workspace):
            receipt['sources'].update(previous.get('sources',{}))
            if 'sdk' in previous:receipt['sdk']=previous['sdk']
    for name in names:
        if name in manifest['ros_sources']:
            spec=manifest['ros_sources'][name]
            patch=manifest_path.parent/spec['patch'] if 'patch' in spec else None
            target=workspace/'src'/name
            receipt['sources'][name]=prepare_repository(target,spec,patch)
            if spec.get('prepare_ros2'):
                receipt['sources'][name]['ros2']=prepare_livox_ros2(target)
            print(f'{name}: {spec["commit"]} prepared',flush=True)
        elif name=='acados':
            receipt['sources'][name]=prepare_acados_source(workspace,manifest['native_sources']['acados'])
            print('acados numerical source and pinned submodules prepared',flush=True)
        elif name in manifest['existing_system_source_pins']:
            receipt['sources'][name]=prepare_system_dependency(workspace,name,
                manifest['existing_system_source_pins'][name],args.system_install_prefix.expanduser().resolve(),args.jobs)
            print(name+': '+receipt['sources'][name]['status'],flush=True)
    if not args.skip_sdk and (args.only is None or 'sdk' in names):
        receipt['sdk']=prepare_sdk(workspace,manifest['native_sources']['Livox-SDK2'],
                                   args.sdk_install_prefix.expanduser().resolve(),args.jobs)
        print('Livox SDK: '+receipt['sdk']['status'],flush=True)
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(receipt,indent=2)+'\n')
    print(f'Receipt: {destination}',flush=True)
    return receipt


if __name__=='__main__':
    try:
        main()
    except (PreparationError,OSError,json.JSONDecodeError) as exc:
        print(f'Dependency preparation stopped: {exc}',file=sys.stderr)
        sys.exit(1)
