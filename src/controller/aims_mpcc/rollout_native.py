"""Offline preparation and read-only loading of the independent C rollout."""
import ctypes
from functools import lru_cache
import hashlib
import math
import os
from pathlib import Path
import platform
import subprocess
import tempfile

import numpy as np

_KERNEL_SOURCE=Path(__file__).parent/'kernels'/'rollout.c'


@lru_cache(maxsize=8)
def _source_digest(source,mtime_ns,ctime_ns,size,machine):
    return hashlib.sha256(Path(source).read_bytes()+machine.encode()).hexdigest()


def kernel_path(directory=None):
    stat=_KERNEL_SOURCE.stat()
    digest=_source_digest(str(_KERNEL_SOURCE),stat.st_mtime_ns,stat.st_ctime_ns,stat.st_size,platform.machine())
    root=Path(directory or os.environ.get('AIMS_MPCC_ROLLOUT_DIR',Path.home()/'.cache'/'aims_mpcc'/'rollout'))
    return root/f'rollout-{digest}.so'


def prepare_kernel(directory=None):
    target=kernel_path(directory)
    if not target.is_file():
        target.parent.mkdir(parents=True,exist_ok=True)
        source=_KERNEL_SOURCE
        with tempfile.TemporaryDirectory(dir=target.parent) as work:
            compiled=Path(work)/'rollout.so'
            subprocess.run(['gcc','-O3','-shared','-fPIC',str(source),'-lm','-o',str(compiled)],check=True)
            os.replace(compiled,target)
    _load.cache_clear()
    return target


@lru_cache(maxsize=8)
def _load(path):
    if not Path(path).is_file():
        return None
    try:
        library=ctypes.CDLL(str(path))
        pointer=ctypes.POINTER(ctypes.c_double)
        library.aims_rollout.argtypes=[pointer,pointer,pointer,ctypes.c_int,ctypes.c_int,
                                     ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,pointer]
        library.aims_rollout.restype=ctypes.c_int
    except (OSError, AttributeError) as exc:
        # Missing caches retain the Python fallback; an existing invalid cache
        # must reject the candidate through the normal validation error path.
        # Runtime loading never repairs or compiles the artifact.
        raise ValueError(f'Cannot load independent rollout cache {path}; '
                         'replace the artifact offline before restarting') from exc
    return library


def native_rollout(path,initial,applied,controls,config,dt=.1,bias=0.):
    library=_load(str(path))
    if library is None:
        return None
    if not math.isfinite(dt) or dt<=0 or not math.isfinite(bias):
        raise ValueError('Positive finite dt and finite steering bias required')
    count=round(dt/.02)
    if count<1 or not math.isclose(count*.02,dt,abs_tol=1e-10):
        raise ValueError('dt must be a multiple of 20 ms')
    initial,applied,controls=[np.ascontiguousarray(v,dtype=np.float64) for v in (initial,applied,controls)]
    if (initial.shape!=(6,) or applied.shape!=(3,) or controls.ndim!=2 or
            controls.shape[1]!=3 or len(controls)<1 or
            not all(np.isfinite(v).all() for v in (initial,applied,controls))):
        raise ValueError('Finite initial(6), applied(3), controls(N,3) required')
    result=np.empty((len(controls)*count+1,6),dtype=np.float64)
    pointer=ctypes.POINTER(ctypes.c_double)
    error=library.aims_rollout(initial.ctypes.data_as(pointer),applied.ctypes.data_as(pointer),
        controls.ctypes.data_as(pointer),len(controls),count,config.wheelbase,config.steering_tau,
        config.understeer_coefficient,bias,result.ctypes.data_as(pointer))
    if error:
        raise ValueError(f'Independent native rollout failed ({error})')
    return result
