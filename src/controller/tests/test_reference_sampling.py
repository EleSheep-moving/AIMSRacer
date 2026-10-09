"""Expose inherited progress units and projection ambiguity without changing them."""
import numpy as np
import pytest
from aims_mpcc.path import ReferencePath


def circle(count):
    angle=np.arange(count)*2*np.pi/count
    return ReferencePath(2*np.c_[np.cos(angle),np.sin(angle)],1.,1.)


def test_sparse_and_dense_chord_progress_are_not_exact_arc_length():
    sparse,dense=circle(8),circle(128)
    exact=4*np.pi
    assert 0.<exact-dense.length<exact-sparse.length
    probes=np.linspace(0.,1.,400,endpoint=False)
    norms=[np.linalg.norm(path.curve.numpy(probes*path.length,1),axis=1)
           for path in (sparse,dense)]
    assert np.max(np.abs(norms[0]-1.))>.01
    assert np.max(np.abs(norms[1]-1.))<.001
    for path in (sparse,dense):
        for fraction in (-.01,0.,.99,1.01):
            a,b=path.at(fraction*path.length),path.at((fraction+2)*path.length)
            assert [a['x'],a['y']]==pytest.approx([b['x'],b['y']],abs=1e-10)


def test_nearby_branches_project_to_closest_geometry():
    # Rounded periodic spline around a narrow rectangle; both branches are
    # geometrically close, but the global nearest branch is unambiguous here.
    path=ReferencePath([[0,0],[1,0],[2,0],[2,.4],[1,.4],[0,.4]],1.,1.)
    dense=path.curve.numpy(np.linspace(0,path.length,20000,endpoint=False))
    for point in ([.8,.04],[.8,.36],[1.2,.04],[1.2,.36]):
        theta,_=path.project(point)
        selected=path.curve.numpy(theta)
        distance=np.linalg.norm(selected-point)
        assert distance<=np.min(np.linalg.norm(dense-point,axis=1))+1e-4
