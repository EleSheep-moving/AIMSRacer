"""Native projection must preserve public geometry and customization semantics."""
import importlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest
from scipy.interpolate import PPoly

from aims_mpcc.path import ReferencePath
import aims_mpcc.path as path_module
import aims_mpcc.vendor.track as track


def circle():
    theta=np.linspace(0,2*np.pi,80,endpoint=False)
    return ReferencePath(np.c_[2*np.cos(theta),2*np.sin(theta)],.9,.9)


def test_offline_prepared_projection_module_exists():
    assert importlib.util.find_spec('aims_mpcc.projection_native') is not None, \
        'offline-prepared native projector is missing'


@pytest.fixture(scope='module')
def prepared(tmp_path_factory):
    assert importlib.util.find_spec('aims_mpcc.projection_native') is not None, \
        'offline-prepared native projector is missing'
    native=importlib.import_module('aims_mpcc.projection_native')
    directory=tmp_path_factory.mktemp('projection-library')
    library=native.prepare_kernel(directory)
    return native,directory,library


@pytest.fixture
def enabled(prepared,monkeypatch):
    native,directory,library=prepared
    monkeypatch.setenv('AIMS_MPCC_PROJECTOR_DIR',str(directory))
    return native,library


def oracle(path,xy):
    """The existing public Python algorithm, isolated from native dispatch."""
    xy=np.asarray(xy,float)
    guess=path._projection_grid[np.argmin(np.sum((path._projection_points-xy)**2,axis=1))]
    result=path_module.minimize_scalar(
        lambda s:float(np.sum((path.curve.numpy(s)-xy)**2)),
        bounds=(guess-path._projection_step,guess+path._projection_step),
        method='bounded',options={'xatol':1e-12})
    return float(result.x%path.length)


def test_offline_kernel_is_used_and_hundreds_of_queries_are_bit_exact(enabled):
    native,_=enabled;path=circle()
    rng=np.random.default_rng(901)
    queries=list(rng.uniform(-3,3,(400,2)))
    for knot in path.curve._spline.x:
        for theta in [knot,np.nextafter(knot,-np.inf),np.nextafter(knot,np.inf)]:
            queries.append(path.curve.numpy(theta))
    queries.extend([[0.,0.],[-0.,0.],[1e154,-1e154],[1e308,-1e308]])
    with np.errstate(over='ignore',invalid='ignore'):
        for xy in queries:
            expected=oracle(path,xy)
            actual=native.project_theta(path,np.asarray(xy,float))
            assert actual is not None,'canonical prepared geometry did not use native kernel'
            assert actual.hex()==expected.hex()
            assert path.project_theta(xy).hex()==expected.hex()


@pytest.mark.parametrize('mutation',['coefficient_in_place','coefficient_replacement',
                                    'spline_replacement','knots_in_place','grid_replacement'])
def test_live_geometry_mutation_matches_python_without_stale_snapshot(enabled,mutation):
    native,_=enabled;path=circle();xy=np.array([1.8,.3])
    before=path.project_theta(xy)
    spline=path.curve._spline
    if mutation=='coefficient_in_place':spline.c[-1,:,0]+=.3
    elif mutation=='coefficient_replacement':
        spline.c=spline.c.copy();spline.c[-1,:,0]+=.3
    elif mutation=='spline_replacement':
        coeff=spline.c.copy();coeff[-1,:,0]+=.3
        path.curve._spline=PPoly(coeff,spline.x.copy(),extrapolate='periodic')
    elif mutation=='knots_in_place':spline.x[2]+=.01
    else:
        path._projection_grid=path._projection_grid+.04
    after=oracle(path,xy)
    assert before.hex()!=after.hex(),'mutation witness must change the projection'
    assert native.project_theta(path,xy).hex()==after.hex()
    assert path.project_theta(xy).hex()==after.hex()


@pytest.mark.parametrize('override',['path_instance','path_class','path_subclass',
    'curve_instance','curve_class','curve_subclass','minimize_scalar','ppoly_call',
    'ppoly_evaluate','ppoly_kernel','ppoly_subclass','wrap_s'])
def test_overridden_methods_fall_back_to_original_python(enabled,monkeypatch,override):
    native,_=enabled;path=circle();xy=np.array([1.8,.3])
    if override=='path_instance':
        path.project=lambda xy:(.5,0.)
    elif override=='path_class':
        monkeypatch.setattr(ReferencePath,'project',lambda self,xy:(.5,0.))
    elif override=='path_subclass':
        class Custom(ReferencePath):pass
        path=Custom(path.points,.9,.9)
    elif override=='curve_instance':
        original=path.curve.numpy;path.curve.numpy=lambda *a,**kw:original(*a,**kw)
    elif override=='curve_class':
        original=track.PeriodicQuintic.numpy
        monkeypatch.setattr(track.PeriodicQuintic,'numpy',lambda self,*a,**kw:original(self,*a,**kw))
    elif override=='curve_subclass':
        class Custom(track.PeriodicQuintic):pass
        path.curve.__class__=Custom
    elif override=='minimize_scalar':
        original=path_module.minimize_scalar
        monkeypatch.setattr(path_module,'minimize_scalar',lambda *a,**kw:original(*a,**kw))
    elif override=='ppoly_call':
        original=PPoly.__call__
        monkeypatch.setattr(PPoly,'__call__',lambda self,*a,**kw:original(self,*a,**kw))
    elif override=='ppoly_evaluate':
        original=PPoly._evaluate
        monkeypatch.setattr(PPoly,'_evaluate',lambda self,*a,**kw:original(self,*a,**kw))
    elif override=='ppoly_kernel':
        from scipy.interpolate import _interpolate
        original=_interpolate._ppoly.evaluate
        monkeypatch.setattr(_interpolate._ppoly,'evaluate',lambda *a,**kw:original(*a,**kw))
    elif override=='ppoly_subclass':
        class Custom(PPoly):pass
        path.curve._spline=Custom(path.curve._spline.c,path.curve._spline.x,extrapolate='periodic')
    else:
        original=track.wrap_s;monkeypatch.setattr(track,'wrap_s',lambda *a:original(*a))
    assert native.project_theta(path,xy) is None
    assert path.project_theta(xy).hex()==oracle(path,xy).hex()


@pytest.mark.parametrize('change',['version','float32','noncontiguous','degree','span',
                                  'nonperiodic','axis','nonfinite','nonmonotone','custom_scalar'])
def test_incompatible_geometry_is_ineligible_without_native_error(enabled,monkeypatch,change):
    native,_=enabled;path=circle()
    if change=='version':monkeypatch.setattr(native.scipy,'__version__','0.0')
    elif change=='float32':path.curve._spline.c=path.curve._spline.c.astype(np.float32)
    elif change=='noncontiguous':path.curve._spline.c=np.asfortranarray(path.curve._spline.c)
    elif change=='degree':path.curve.degree=3
    elif change=='span':path.curve.length+=.1
    elif change=='nonperiodic':path.curve._spline.extrapolate=True
    elif change=='axis':path.curve._spline.axis=1
    elif change=='nonfinite':path.curve._spline.c[0,0,0]=np.nan
    elif change=='custom_scalar':
        class Custom(float):pass
        path._projection_step=Custom(path._projection_step)
    else:path.curve._spline.x[2]=path.curve._spline.x[1]
    assert native.project_theta(path,np.array([1.8,.3])) is None


def test_missing_cache_uses_python_and_never_runs_subprocess(enabled,tmp_path,monkeypatch):
    native,_=enabled;path=circle();xy=[1.8,.3];expected=oracle(path,xy)
    monkeypatch.setenv('AIMS_MPCC_PROJECTOR_DIR',str(tmp_path/'missing'))
    def forbidden(*a,**kw):raise AssertionError('runtime attempted subprocess')
    monkeypatch.setattr(subprocess,'run',forbidden)
    monkeypatch.setattr(subprocess,'check_output',forbidden)
    assert native.project_theta(path,np.asarray(xy)) is None
    assert path.project_theta(xy).hex()==expected.hex()
    assert not (tmp_path/'missing').exists()


@pytest.mark.parametrize('corruption',['binary','metadata','abi','symbol','manifest_missing'])
def test_existing_invalid_cache_raises_without_online_repair(prepared,tmp_path,monkeypatch,corruption):
    import shutil
    native,directory,_=prepared
    cache=tmp_path/'cache';shutil.copytree(directory,cache)
    monkeypatch.setenv('AIMS_MPCC_PROJECTOR_DIR',str(cache))
    artifact=native.kernel_path(cache);manifest=artifact.parent/'manifest.json'
    metadata=json.loads(manifest.read_text())
    if corruption=='binary':artifact.write_bytes(b'not ELF')
    elif corruption=='metadata':manifest.write_text('{}')
    elif corruption=='abi':metadata['identity']['abi']=-1;manifest.write_text(json.dumps(metadata))
    elif corruption=='symbol':
        # A hash-valid but unrelated ELF must fail the exported-ABI contract.
        import ctypes.util,hashlib
        lib=ctypes.util.find_library('m')
        candidates=list(Path('/lib').glob('**/'+lib))+list(Path('/usr/lib').glob('**/'+lib))
        artifact.write_bytes(candidates[0].read_bytes())
        metadata['binary_sha256']=hashlib.sha256(artifact.read_bytes()).hexdigest()
        manifest.write_text(json.dumps(metadata))
    else:manifest.unlink()
    def forbidden(*a,**kw):raise AssertionError('runtime attempted repair')
    monkeypatch.setattr(subprocess,'run',forbidden)
    with pytest.raises(ValueError,match='projection cache'):circle().project_theta([1.8,.3])


def test_runtime_subprocess_forbidden_with_valid_cache(prepared):
    _,directory,_=prepared
    script='''
import subprocess, os
def forbidden(*a,**kw):raise AssertionError('runtime compiler invoked')
subprocess.run=forbidden;subprocess.check_output=forbidden;subprocess.Popen=forbidden
from aims_mpcc.path import ReferencePath
import numpy as np
t=np.linspace(0,2*np.pi,80,endpoint=False)
p=ReferencePath(np.c_[2*np.cos(t),2*np.sin(t)],.9,.9)
from aims_mpcc.projection_native import project_theta
assert project_theta(p,np.array([1.8,.3])) is not None
assert p.project_theta([1.8,.3]) == project_theta(p,np.array([1.8,.3]))
'''
    import os
    env=dict(os.environ,AIMS_MPCC_PROJECTOR_DIR=str(directory))
    result=subprocess.run([sys.executable,'-c',script],env=env,capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_projector_can_be_imported_before_path():
    result=subprocess.run([sys.executable,'-c',
        'from aims_mpcc.projection_native import prepare_kernel; from aims_mpcc.path import ReferencePath'],
        capture_output=True,text=True)
    assert result.returncode==0,result.stderr


def test_distributed_source_contains_projector_and_redistribution_licenses():
    root=Path(__file__).parents[1]
    assert (root/'aims_mpcc/kernels/projector.c').is_file()
    for name in ['SCIPY_LICENSE','NUMPY_LICENSE']:
        license_file=root/'aims_mpcc/vendor'/name
        assert license_file.is_file(),'redistribution license must be packaged'
        text=license_file.read_text()
        assert 'Redistribution and use' in text
        assert 'THIS SOFTWARE IS PROVIDED' in text


@pytest.mark.parametrize('unsupported',['version','machine'])
def test_uncovered_versions_preserve_offline_python_fallback(enabled,monkeypatch,unsupported):
    native,_=enabled
    if unsupported=='version':monkeypatch.setattr(native.scipy,'__version__','0.0')
    else:monkeypatch.setattr(native.platform,'machine',lambda:'unproven-architecture')
    def forbidden(*a,**kw):raise AssertionError('unsupported version attempted compilation')
    monkeypatch.setattr(subprocess,'run',forbidden)
    assert native.prepare_kernel() is None
    path=circle()
    assert path.project_theta([1.8,.3]).hex()==oracle(path,[1.8,.3]).hex()


def test_cache_corruption_after_warm_load_is_not_hidden_by_memoization(enabled):
    native,library=enabled
    # Use a private copy: other tests retain the offline-prepared valid library.
    import shutil,tempfile,os
    with tempfile.TemporaryDirectory() as temp:
        source=library.parent.parent
        shutil.copytree(source,Path(temp)/'cache')
        old=os.environ['AIMS_MPCC_PROJECTOR_DIR']
        try:
            os.environ['AIMS_MPCC_PROJECTOR_DIR']=str(Path(temp)/'cache')
            path=circle();assert native.project_theta(path,np.array([1.8,.3])) is not None
            # Replace the file, as offline publication does. Truncating a
            # mapped ELF in place can SIGBUS in the dynamic loader itself.
            replacement=Path(temp)/'corrupt.so';replacement.write_bytes(b'changed after loading')
            os.replace(replacement,native.kernel_path())
            with pytest.raises(ValueError,match='projection cache'):path.project_theta([1.8,.3])
        finally:os.environ['AIMS_MPCC_PROJECTOR_DIR']=old


def test_hash_valid_replacement_after_warm_load_requires_restart(enabled,tmp_path,monkeypatch):
    import ctypes.util,hashlib,os,shutil
    native,library=enabled
    shutil.copytree(library.parent.parent,tmp_path/'cache')
    monkeypatch.setenv('AIMS_MPCC_PROJECTOR_DIR',str(tmp_path/'cache'))
    path=circle();xy=np.array([1.8,.3])
    assert native.project_theta(path,xy) is not None
    artifact=native.kernel_path();manifest=artifact.parent/'manifest.json'
    metadata=json.loads(manifest.read_text())
    libm=ctypes.util.find_library('m')
    candidates=list(Path('/lib').glob('**/'+libm))+list(Path('/usr/lib').glob('**/'+libm))
    replacement=tmp_path/'replacement.so';replacement.write_bytes(candidates[0].read_bytes())
    os.replace(replacement,artifact)
    metadata['binary_sha256']=hashlib.sha256(artifact.read_bytes()).hexdigest()
    manifest.write_text(json.dumps(metadata))
    def forbidden(*a,**kw):raise AssertionError('warm cache change attempted runtime repair')
    monkeypatch.setattr(subprocess,'run',forbidden)
    with pytest.raises(ValueError,match='projection cache'):
        path.project_theta(xy)


def test_wheel_install_retains_kernel_licenses_and_offline_loader(tmp_path):
    import os,shutil,zipfile
    root=Path(__file__).parents[1];source=tmp_path/'source'
    shutil.copytree(root,source,ignore=shutil.ignore_patterns('__pycache__','build','dist','*.egg-info'))
    built=subprocess.run([sys.executable,'setup.py','bdist_wheel','--dist-dir',str(tmp_path/'dist')],
        cwd=source,capture_output=True,text=True)
    assert built.returncode==0,built.stderr
    wheel=next((tmp_path/'dist').glob('*.whl'))
    with zipfile.ZipFile(wheel) as archive:
        names=archive.namelist()
        for name in ['aims_mpcc/projection_native.py','aims_mpcc/kernels/projector.c',
                     'aims_mpcc/vendor/SCIPY_LICENSE','aims_mpcc/vendor/NUMPY_LICENSE']:
            assert name in names
        assert any(name.endswith('/share/aims_mpcc/NOTICE.md') for name in names)
    target=tmp_path/'installed'
    installed=subprocess.run([sys.executable,'-m','pip','install','--no-deps','--target',str(target),str(wheel)],
        capture_output=True,text=True)
    assert installed.returncode==0,installed.stderr
    script='''
import subprocess
def forbidden(*a,**kw):raise AssertionError('installed runtime attempted compilation')
subprocess.run=forbidden;subprocess.check_output=forbidden
from aims_mpcc import projection_native
from aims_mpcc.path import ReferencePath
import numpy as np
assert projection_native._SOURCE.is_file()
t=np.linspace(0,2*np.pi,80,endpoint=False)
p=ReferencePath(np.c_[2*np.cos(t),2*np.sin(t)],.9,.9)
assert np.isfinite(p.project_theta([1.8,.3]))
'''
    environment=dict(os.environ,PYTHONPATH=str(target),AIMS_MPCC_PROJECTOR_DIR=str(tmp_path/'missing'))
    checked=subprocess.run([sys.executable,'-c',script],cwd=tmp_path,env=environment,capture_output=True,text=True)
    assert checked.returncode==0,checked.stderr


def test_checked_native_entry_validates_live_tables_before_projection(enabled):
    native,_=enabled;path=circle();xy=np.array([1.8,.3])
    library=native._library(native._folder())
    assert hasattr(library,'aims_project_checked'),'live table eligibility entry point missing'
    import ctypes
    spline=path.curve._spline
    result=ctypes.c_double()
    def checked():
        return library.aims_project_checked(path._projection_grid.ctypes.data,
            path._projection_points.ctypes.data,len(path._projection_grid),spline.x.ctypes.data,
            spline.c.ctypes.data,len(spline.x)-1,path.length,path._projection_step,
            xy[0],xy[1],ctypes.byref(result))
    assert checked()==1
    assert result.value.hex()==oracle(path,xy).hex()
    old=spline.c[0,0,0];spline.c[0,0,0]=np.nan
    assert checked()==0
    spline.c[0,0,0]=old;spline.x[2]=spline.x[1]
    assert checked()==0


def test_incompatible_live_tables_keep_python_fallback_even_with_bad_cache(enabled,tmp_path,monkeypatch):
    native,_=enabled
    monkeypatch.setenv('AIMS_MPCC_PROJECTOR_DIR',str(tmp_path))
    artifact=native.kernel_path();artifact.parent.mkdir(parents=True)
    (artifact.parent/'manifest.json').write_text('{}')
    path=circle();path.curve._spline.c[0,0,0]=np.nan
    assert native.project_theta(path,np.array([1.8,.3])) is None


def test_complete_bounded_optimizer_trace_and_status_policy_are_exact(enabled):
    from scipy.optimize._optimize import _minimize_scalar_bounded
    import ctypes
    native,_=enabled;path=circle();library=native._library(native._folder())
    # Force a real first-argmin tie at the seam, without regenerating geometry.
    path._projection_points=path._projection_points.copy()
    path._projection_points[1]=path._projection_points[0]
    queries=[path._projection_points[0],path.curve.numpy(np.nextafter(0.,-np.inf)),
             path.curve.numpy(np.nextafter(path.length,np.inf)),np.array([0.,0.]),
             np.array([1e154,-1e154]),np.array([1e308,-1e308])]
    queries.extend(np.random.default_rng(52).uniform(-3,3,(8,2)))
    pointer=ctypes.POINTER(ctypes.c_double);spline=path.curve._spline
    with np.errstate(over='ignore',invalid='ignore'):
        for xy in queries:
            distances=np.sum((path._projection_points-xy)**2,axis=1)
            first=int(np.argmin(distances));guess=path._projection_grid[first];trace=[]
            def objective(theta):
                value=float(np.sum((path.curve.numpy(theta)-xy)**2))
                trace.append([theta,value]);return value
            result=_minimize_scalar_bounded(objective,
                (guess-path._projection_step,guess+path._projection_step),xatol=1e-12)
            stats=np.empty(5);native_trace=np.empty((500,2))
            theta=library.aims_project(path._projection_grid.ctypes.data,
                path._projection_points.ctypes.data,len(path._projection_grid),spline.x.ctypes.data,
                spline.c.ctypes.data,len(spline.x)-1,path.length,path._projection_step,xy[0],xy[1],
                stats.ctypes.data_as(pointer),native_trace.ctypes.data_as(pointer))
            assert theta.hex()==float(result.x%path.length).hex()
            expected=np.asarray([result.x,result.fun,result.nfev,result.status,first],dtype=np.float64)
            assert np.array_equal(stats.view(np.uint64),expected.view(np.uint64))
            assert np.array_equal(native_trace[:len(trace)].view(np.uint64),np.asarray(trace).view(np.uint64))
