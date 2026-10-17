import numpy as np
from scipy.spatial.transform import Rotation
from observer import Observer, Config


def trajectory(o, velocity=np.array([2.,.3,0.]), delay=0., yaw_rate=0., duration=2.):
    pending=[]; output=[]
    for i,t in enumerate(np.arange(0,duration+.001,.005)):
        ns=1_000_000_000+round(t*1e9)
        q=Rotation.from_rotvec([0,0,yaw_rate*t]);force=q.inv().apply([0,0,9.80665])/9.80665
        if i%20==0: pending.append((t+delay,ns,velocity*t,q.as_quat()))
        while pending and pending[0][0]<=t+1e-9:
            _,stamp,p,r=pending.pop(0);assert o.add_lio(stamp,p,r)
        assert o.add_imu(ns,force,np.array([0.,0.,yaw_rate]))
        if o.latest()['valid']: output.append(o.latest())
    return output


def test_moving_initialization_and_nonzero_lateral_velocity():
    o=Observer(Config(lever=np.zeros(3)))
    out=trajectory(o)
    assert out and out[0]['stamp_ns']>=1_200_000_000
    assert abs(out[-1]['vx']-2)<1e-8
    assert np.allclose(out[-1]['velocity_world'],[2,.3,0],atol=1e-8)


def test_delayed_anchor_replays_to_same_latest_state_and_snapshots_are_values():
    c=Config(lever=np.zeros(3));normal=Observer(c);late=Observer(c)
    a=trajectory(normal,delay=0.);b=trajectory(late,delay=.15)
    assert abs(a[-1]['vx']-b[-1]['vx'])<1e-7
    snapshot=b[-1];saved=snapshot['velocity_world'].copy()
    late.add_imu(3_005_000_000,np.array([0.,0.,1.]),np.zeros(3))
    assert np.array_equal(snapshot['velocity_world'],saved)
    assert late.replayed_imus>0


def test_turning_world_velocity_and_rear_lever_conversion():
    c=Config(lever=np.array([.3,.03,.02]));o=Observer(c)
    a=trajectory(o,yaw_rate=.8)
    last=a[-1];R=Rotation.from_rotvec([0,0,.8*2])
    expected=R.inv().apply([2,.3,0])-np.cross([0,0,.8],c.lever)
    assert abs(last['vx']-expected[0])<1e-8


def test_duplicate_nan_gap_reset_and_bounded_history():
    o=Observer(Config(max_events=100));trajectory(o,duration=3.)
    assert o.history_size<=100
    assert not o.add_imu(4_000_000_000,np.array([0.,0.,1.]),np.zeros(3))
    before=o.latest()['stamp_ns'];assert not o.add_imu(4_005_000_000,np.array([np.nan,0,1]),np.zeros(3))
    assert o.latest()['stamp_ns']==before
    assert o.add_imu(4_040_000_000,np.array([0.,0.,1.]),np.zeros(3))
    assert not o.latest()['valid'] and o.latest()['status']=='imu_gap'


def test_lio_outage_degrades_and_expires_without_zeros():
    o=Observer(Config());trajectory(o,duration=1.)
    for i in range(1,103):o.add_imu(2_000_000_000+i*5_000_000,np.array([0.,0.,1.]),np.zeros(3))
    assert not o.latest()['valid'] and o.latest()['status']=='lio_stale'


def test_anchor_only_stream_is_bounded_before_imu_ready():
    o=Observer(Config(max_events=20))
    for i in range(100):assert o.add_lio(1_000_000_000+i*100_000_000,np.zeros(3),[0,0,0,1])
    assert o.history_size<=20


def test_single_imu_then_anchor_only_stream_is_bounded():
    o=Observer(Config(max_events=20))
    assert o.add_imu(1_000_000_000,[0,0,1],[0,0,0])
    for i in range(100):assert o.add_lio(1_000_000_000+i*100_000_000,np.zeros(3),[0,0,0,1])
    assert o.history_size<=20


def test_huge_finite_input_rejects_transactionally():
    o=Observer();trajectory(o,duration=1.);before=o.latest()
    assert not o.add_imu(2_005_000_000,np.array([1e308,0,1.]),np.zeros(3))
    assert o.latest()['stamp_ns']==before['stamp_ns']


def test_high_frequency_lio_can_initialize():
    o=Observer()
    for i in range(201):
        t=1_000_000_000+i*5_000_000
        if i%4==0:assert o.add_lio(t,[i*.005*2,0,0],[0,0,0,1])
        assert o.add_imu(t,[0,0,1],[0,0,0])
    assert o.latest()['valid'] and abs(o.latest()['vx']-2)<1e-8


def test_mounting_must_be_finite_proper_rotation():
    import pytest
    for matrix in [np.full((3,3),np.nan),np.eye(2),np.diag([1,1,-1]),np.eye(3)*2]:
        with pytest.raises(ValueError):Observer(Config(mounting=matrix))


def test_finite_state_must_not_project_to_infinite_output():
    c=Config(mounting=Rotation.from_euler('z',45,degrees=True).as_matrix(),lever=np.zeros(3));o=Observer(c)
    velocity=np.array([1.5*2.**1023,-1.5*2.**1023,0.])
    for i in range(101):
        t=1_000_000_000+i*5_000_000
        if i%50==0:o.add_lio(t,velocity*(i*.005),[0,0,0,1])
        o.add_imu(t,[0,0,1],[0,0,0])
    assert not o.latest()['valid'] or np.isfinite(o.latest()['vx'])


def test_lio_only_after_imu_outage_retains_two_seconds():
    o=Observer()
    o.add_imu(1_000_000_000,[0,0,1],[0,0,0])
    for i in range(100):o.add_lio(1_000_000_000+i*100_000_000,[0,0,0],[0,0,0,1])
    assert max(o.anchors)-min(o.anchors)<=2_000_000_000


def test_reacquire_after_expired_anchor_requires_new_three_pose_support():
    o=Observer();trajectory(o,duration=1.)
    for i in range(1,141):assert o.add_imu(2_000_000_000+i*5_000_000,[0,0,1],[0,0,0])
    assert not o.latest()['valid']
    for i in range(2,43):
        t=2_700_000_000+i*5_000_000
        if i in [2,22,42]:o.add_lio(t,[100.+(t-2_710_000_000)*2e-9,0,0],[0,0,0,1])
        o.add_imu(t,[0,0,1],[0,0,0])
        if i<42:assert not o.latest()['valid']
    assert o.latest()['valid'] and abs(o.latest()['vx']-2)<1e-8
    assert o.latest()['lio_reinitializations']==1
