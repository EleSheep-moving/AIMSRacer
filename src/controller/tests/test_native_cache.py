"""Exercise real CasADi/ccache reuse and invalidation with a tiny native function."""
import os
import subprocess
from pathlib import Path
import pytest
import casadi as ca
from aims_mpcc.native import build_context, solver_options, ccache_command


def stats(directory):
    raw=subprocess.check_output([ccache_command(),'--print-stats'],env={**os.environ,'CCACHE_DIR':str(directory)},text=True)
    return {line.split()[0]:int(line.split()[1]) for line in raw.splitlines()}


def evaluate(allow,offset=1.,flag='-O2'):
    with build_context(allow_compile=allow):
        x=ca.SX.sym('x');options=solver_options();options.update(jit=True,compiler='shell')
        options['jit_options']['flags']=[flag]
        f=ca.Function('cached_test',[x],[x+offset],options)
        result=float(f(2.));del f
        return result


def test_cache_hit_miss_and_invalidation(tmp_path,monkeypatch):
    cache=tmp_path/'cache';monkeypatch.setenv('AIMS_MPCC_CACHE_DIR',str(cache))
    cwd=os.getcwd();prefix=os.environ.get('CCACHE_PREFIX')
    with pytest.raises(RuntimeError):evaluate(False)
    assert os.getcwd()==cwd and os.environ.get('CCACHE_PREFIX')==prefix
    assert evaluate(True)==3.
    before=stats(cache)
    assert evaluate(False)==3.
    assert evaluate(True)==3.
    after=stats(cache)
    assert after['direct_cache_hit']+after['preprocessed_cache_hit']>=before['direct_cache_hit']+before['preprocessed_cache_hit']+2
    # Changed embedded constant / compiler flags cannot load the old object.
    with pytest.raises(RuntimeError):evaluate(False,offset=2.)
    with pytest.raises(RuntimeError):evaluate(False,flag='-O3')
    with monkeypatch.context() as changed:
        changed.setattr(ca,'__version__',ca.__version__+'-different-abi')
        with pytest.raises(RuntimeError):evaluate(False)
    assert evaluate(True,offset=2.)==4.
    assert evaluate(False,offset=2.)==4.
