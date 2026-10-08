import math
import numpy as np

from aims_mpcc.config import VehicleConfig
from aims_mpcc.path import ReferencePath


def test_curvature_cap_and_periodic_acceleration_feasibility():
    from aims_mpcc.speed_planner import SpeedPlanner
    angles=np.arange(128)*2*math.pi/128
    path=ReferencePath(np.c_[2*np.cos(angles),2*np.sin(angles)],1.,1.)
    config=VehicleConfig(cruise_speed=2.,max_speed=2.5,lateral_accel_limit=.5)
    planner=SpeedPlanner(path,config)
    assert max(planner.speeds)<1.02
    assert min(planner.speeds)>.95
    assert planner.at(-.1)==planner.at(path.length-.1)
    assert planner.at(path.length+.2)==planner.at(.2)


def test_speed_preview_advances_along_reference_and_stays_bounded():
    from aims_mpcc.speed_planner import SpeedPlanner
    angles=np.arange(128)*2*math.pi/128
    path=ReferencePath(np.c_[2*np.cos(angles),np.sin(angles)],1.,1.)
    c=VehicleConfig(cruise_speed=2.,max_speed=2.5,lateral_accel_limit=.5)
    planner=SpeedPlanner(path,c)
    refs=planner.refs(0.,10,.1)
    assert len(refs)==11
    assert np.isfinite(refs).all()
    assert all(0.<=v<=c.cruise_speed for v in refs)
    ds=[]
    for lo,hi in zip(planner.positions,np.r_[planner.positions[1:],path.length]):
        positions=np.linspace(lo,hi,21)
        norms=[np.linalg.norm(path.curve.numpy(s,1)) for s in positions]
        ds.append(np.trapz(norms,positions))
    ds=np.asarray(ds)
    next_v=np.roll(planner.speeds,-1)
    assert np.all(next_v**2-planner.speeds**2<=2*c.accel_limit*ds+1e-6)
    assert np.all(planner.speeds**2-next_v**2<=2*c.brake_limit*ds+1e-6)
