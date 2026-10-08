"""Exact scalar projection: offline library preparation, live geometry at runtime.

Only the library is cached. Geometry is never copied or made read-only here.
Unsupported/custom geometry uses the existing Python projector; an eligible
existing corrupt cache is an error, and runtime never compiles or repairs it.
"""
import ctypes
import errno
from functools import lru_cache
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

import numpy as np
import scipy
from scipy.interpolate import PPoly
from scipy.interpolate import _interpolate

from . import path as path_module
from .vendor import track

_SOURCE=Path(__file__).parent/'kernels'/'projector.c'
_ABI=1
_FLAGS=('-O3','-std=c99','-fPIC','-shared','-fno-fast-math',
        '-ffp-contract=off','-fno-associative-math','-fexcess-precision=standard')
_COVERED_ENVIRONMENTS={('x86_64','1.15.3','1.26.4'),('aarch64','1.8.0','1.21.5')}
_REFERENCE=path_module.ReferencePath
_PROJECT=_REFERENCE.project
_PROJECT_THETA=_REFERENCE.project_theta
_CURVE=track.PeriodicQuintic
_NUMPY=_CURVE.numpy
_WRAP=track.wrap_s
_MINIMIZE=path_module.minimize_scalar
_PPOLY_CALL=PPoly.__call__
_PPOLY_EVALUATE=PPoly._evaluate
_PPOLY_CONTIGUOUS=PPoly._ensure_c_contiguous
_PPOLY_KERNEL=_interpolate._ppoly.evaluate
_POINTER=ctypes.POINTER(ctypes.c_double)


def _covered_environment():
    return (platform.machine(),scipy.__version__,np.__version__) in _COVERED_ENVIRONMENTS


def _digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()


@lru_cache(maxsize=8)
def _source_hash(path,mtime,ctime,size):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _identity():
    stat=_SOURCE.stat()
    return dict(abi=_ABI,source_sha256=_source_hash(str(_SOURCE),stat.st_mtime_ns,
                stat.st_ctime_ns,stat.st_size),flags=list(_FLAGS),machine=platform.machine(),
                scipy=scipy.__version__,numpy=np.__version__)


@lru_cache(maxsize=8)
def _identity_folder(root,source_hash,machine,scipy_version,numpy_version):
    identity=dict(abi=_ABI,source_sha256=source_hash,flags=list(_FLAGS),machine=machine,
                  scipy=scipy_version,numpy=numpy_version)
    return Path(root)/f'projector-{_digest(identity)}'


def _folder(directory=None):
    root=directory or os.environ.get('AIMS_MPCC_PROJECTOR_DIR')
    if root is None:root=Path.home()/'.cache'/'aims_mpcc'/'projection'
    stat=_SOURCE.stat()
    return _identity_folder(str(root),_source_hash(str(_SOURCE),stat.st_mtime_ns,
        stat.st_ctime_ns,stat.st_size),platform.machine(),scipy.__version__,np.__version__)


def _cache_error(folder):
    return ValueError(f'Invalid native projection cache {folder}; '
                      'replace the artifact offline before restarting')


@lru_cache(maxsize=8)
def _read_metadata(folder,manifest_stat):
    folder=Path(folder)
    try:
        metadata=json.loads((folder/'manifest.json').read_text())
        build=metadata['build']
        compiler=build['compiler']
        if (_digest(metadata['identity'])!=_digest(_identity()) or
                _digest(build['identity'])!=_digest(metadata['identity']) or
                type(compiler) is not dict or set(compiler)!={'path','version'} or
                not all(type(v) is str and v for v in compiler.values()) or
                metadata['build_digest']!=_digest(build) or
                type(metadata['binary_sha256']) is not str or
                len(metadata['binary_sha256'])!=64):
            raise ValueError('identity mismatch')
        return metadata,folder/f"projector-{metadata['build_digest']}.so"
    except (OSError,ValueError,KeyError,TypeError) as exc:
        raise _cache_error(folder) from exc


def _metadata(folder):
    try:return _read_metadata(str(folder),_stat_key(folder/'manifest.json'))
    except OSError as exc:raise _cache_error(folder) from exc


def kernel_path(directory=None):
    """Return prepared binary location; missing caches have no runtime side effects."""
    folder=_folder(directory)
    return _metadata(folder)[1] if folder.exists() else folder/'projector.so'


def prepare_kernel(directory=None):
    """Explicit offline preparation; called by prepare_solver, never projection."""
    if not _covered_environment():
        return None # Uncovered versions retain the existing Python projector.
    folder=_folder(directory)
    if not folder.exists():
        compiler=shutil.which('gcc')
        if compiler is None:raise ValueError('gcc is required for offline projector preparation')
        version=subprocess.check_output([compiler,'--version'],text=True).splitlines()[0]
        identity=_identity()
        build=dict(identity=identity,compiler=dict(path=compiler,version=version))
        digest=_digest(build)
        folder.parent.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(dir=folder.parent) as temporary:
            prepared=Path(temporary)/'prepared';prepared.mkdir()
            binary=prepared/f'projector-{digest}.so'
            subprocess.run([compiler,*_FLAGS,str(_SOURCE),'-lm','-o',str(binary)],check=True)
            metadata=dict(identity=identity,build=build,build_digest=digest,
                          binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
            (prepared/'manifest.json').write_text(json.dumps(metadata,indent=2)+'\n')
            try:os.rename(prepared,folder)
            except OSError as exc:
                if exc.errno not in (errno.EEXIST,errno.ENOTEMPTY):raise
                # Another offline preparer published first; validate below.
    _load_library.cache_clear()
    _library(folder) # Validate even an existing cache; do not silently repair it.
    return kernel_path(directory)


def _stat_key(path):
    stat=path.stat()
    return stat.st_mtime_ns,stat.st_ctime_ns,stat.st_size


@lru_cache(maxsize=8)
def _load_library(folder,manifest_stat,binary_path,binary_stat):
    folder=Path(folder)
    metadata,binary=_metadata(folder)
    try:
        if str(binary)!=binary_path or hashlib.sha256(binary.read_bytes()).hexdigest()!=metadata['binary_sha256']:
            raise ValueError('binary hash mismatch')
        library=ctypes.CDLL(str(binary))
        library.aims_projector_abi.argtypes=[]
        library.aims_projector_abi.restype=ctypes.c_int
        if library.aims_projector_abi()!=_ABI:raise ValueError('ABI mismatch')
        library.aims_project.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p,
            ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,
            _POINTER,_POINTER]
        library.aims_project.restype=ctypes.c_double
        library.aims_project_checked.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,
            ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_double,ctypes.c_double,
            ctypes.c_double,ctypes.c_double,_POINTER]
        library.aims_project_checked.restype=ctypes.c_int
    except (OSError,AttributeError,ValueError) as exc:
        raise _cache_error(folder) from exc
    return library


def _library(folder):
    # Stat-sensitive memoization detects edits/replacement without rehashing an
    # unchanged library. The manifest and binary are read-only runtime inputs.
    try:
        manifest_stat=_stat_key(_manifest_path(folder))
        metadata,binary=_read_metadata(str(folder),manifest_stat)
        return _load_library(str(folder),manifest_stat,str(binary),_stat_key(binary))
    except FileNotFoundError as exc:
        if not folder.exists():return None
        raise _cache_error(folder) from exc
    except OSError as exc:
        raise _cache_error(folder) from exc


@lru_cache(maxsize=8)
def _manifest_path(folder):
    return folder/'manifest.json'


def _bound_method(obj,name,function):
    return getattr(getattr(obj,name,None),'__func__',None) is function


def _live_geometry(path):
    """Hold original arrays through the native call; never normalize/copy them."""
    if (not _covered_environment() or
            type(path) is not _REFERENCE or
            not _bound_method(path,'project',_PROJECT) or
            not _bound_method(path,'project_theta',_PROJECT_THETA) or
            path_module.minimize_scalar is not _MINIMIZE or track.wrap_s is not _WRAP or
            _interpolate._ppoly.evaluate is not _PPOLY_KERNEL):
        return None
    try:
        curve=path.curve;spline=curve._spline
        if (type(curve) is not _CURVE or curve.degree!=5 or
                not _bound_method(curve,'numpy',_NUMPY) or type(spline) is not PPoly or
                not _bound_method(spline,'__call__',_PPOLY_CALL) or
                not _bound_method(spline,'_evaluate',_PPOLY_EVALUATE) or
                not _bound_method(spline,'_ensure_c_contiguous',_PPOLY_CONTIGUOUS) or
                spline.extrapolate!='periodic' or spline.axis!=0):
            return None
        grid,points,knots,coeff=(path._projection_grid,path._projection_points,spline.x,spline.c)
        arrays=(grid,points,knots,coeff)
        if any(type(a) is not np.ndarray or a.dtype!=np.dtype('float64') or
               not a.flags.c_contiguous or not a.flags.aligned for a in arrays):return None
        if (grid.ndim!=1 or not 0<len(grid)<2**31 or points.shape!=(len(grid),2) or
                knots.ndim!=1 or not 1<len(knots)<2**31 or
                coeff.shape!=(6,len(knots)-1,2)):
            return None
        if any(type(v) not in (float,np.float64) for v in
               (path.length,curve.length,path._projection_step)):return None
        length=float(path.length);step=float(path._projection_step)
        if (not math.isfinite(length) or length<=0 or not math.isfinite(step) or step<=0 or
                curve.length!=length or knots[0]!=0. or knots[-1]!=length):
            return None
        return arrays,length,step
    except (AttributeError,TypeError,ValueError,OverflowError):
        return None


def _tables_eligible_python(arrays,step):
    # Error path only: an incompatible live table must retain Python fallback
    # even if a cache exists but cannot be loaded for native eligibility checks.
    grid,points,knots,coeff=arrays
    with np.errstate(over='ignore',invalid='ignore'):
        return (all(np.isfinite(a).all() for a in arrays) and
                np.all(knots[1:]>knots[:-1]) and
                np.isfinite(grid-step).all() and np.isfinite(grid+step).all())


def project_theta(path,xy):
    """Return native progress or None for Python fallback. xy is prevalidated."""
    geometry=_live_geometry(path)
    if geometry is None:return None
    try:library=_library(_folder())
    except ValueError:
        if not _tables_eligible_python(geometry[0],geometry[2]):return None
        raise
    if library is None:return None
    arrays,length,step=geometry
    grid,points,knots,coeff=arrays
    # Locals own strong references to live arrays while CDLL releases the GIL.
    result=ctypes.c_double()
    eligible=library.aims_project_checked(grid.ctypes.data,points.ctypes.data,
        len(grid),knots.ctypes.data,coeff.ctypes.data,len(knots)-1,
        length,step,xy[0],xy[1],ctypes.byref(result))
    return result.value if eligible else None
