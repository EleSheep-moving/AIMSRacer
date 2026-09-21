"""Persistent compiler cache; online workers never compile a cache miss."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import tempfile


def ccache_command():
    command = shutil.which('ccache')
    local = Path.home() / '.local/bin/ccache'
    if command is None and local.is_file():
        command = str(local)
    if command is None:
        raise RuntimeError('Install ccache before preparing the MPCC solver cache')
    return command


def cache_directory():
    base = Path(os.environ.get('XDG_CACHE_HOME', str(Path.home() / '.cache')))
    return Path(os.environ.get('AIMS_MPCC_CACHE_DIR', str(base / 'aims_mpcc/ccache'))).expanduser().resolve()


def solver_options():
    compiler = shutil.which('gcc')
    if compiler is None:
        raise RuntimeError('Install gcc before preparing the MPCC solver cache')
    return {
        'jit_temp_suffix': False,
        'jit_name': 'aims_mpcc_callbacks',
        'jit_options': {
            'compiler': shlex.quote(ccache_command()) + ' ' + shlex.quote(compiler),
            'flags': ['-O2'],
            'name': 'aims_mpcc_native',
            'temp_suffix': False,
        },
    }


@contextmanager
def build_context(allow_compile=False):
    """Use only in a dedicated worker/CLI process (cwd and environment change).

    ccache hashes generated C, headers, options and compiler content. The extra
    fingerprint also invalidates on CasADi version or platform changes. Unique
    working directories keep stable JIT filenames safe across concurrent workers.
    """
    import casadi
    cache = cache_directory()
    cache.mkdir(parents=True, exist_ok=True)
    ccache_command()
    with tempfile.TemporaryDirectory(prefix='aims-mpcc-native-') as work:
        fingerprint = Path(work) / 'abi.json'
        fingerprint.write_text(json.dumps({
            'schema': 1, 'casadi': casadi.__version__,
            'machine': platform.machine(), 'system': platform.system(),
        }, sort_keys=True))
        settings = {
            'CCACHE_DIR': str(cache),
            'CCACHE_COMPILERCHECK': 'content',
            'CCACHE_EXTRAFILES': str(fingerprint),
            'CCACHE_BASEDIR': work,
            'CCACHE_NOHASHDIR': '1',
            'CCACHE_SLOPPINESS': '',
            'CCACHE_DISABLE': None, 'CCACHE_RECACHE': None,
            'CCACHE_READONLY_DIRECT': None,
            'CCACHE_READONLY': None if allow_compile else '1',
            # ccache runs PREFIX only on a miss; /usr/bin/false rejects compilation.
            'CCACHE_PREFIX': None if allow_compile else '/usr/bin/false',
            'CCACHE_PREFIX_CPP': None,
        }
        previous = {key: os.environ.get(key) for key in settings}
        cwd = os.getcwd()
        try:
            for key, value in settings.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
            os.chdir(work)
            yield cache
        finally:
            os.chdir(cwd)
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
