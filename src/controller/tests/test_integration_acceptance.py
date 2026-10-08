"""Acceptance follows control authority and bounded worker recovery contracts."""
import pytest
from pathlib import Path
from types import SimpleNamespace
import numpy as np

from aims_mpcc import integration
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath


def metrics(status='COMPLETE'):
    return dict(status_at_end=status, maximum_command_speed_mps=.5,
                maximum_command_steering_rad=.2, minimum_footprint_margin_m=.3,
                cross_track_rms_m=.02, cross_track_max_m=.04,
                finish_error_m=.03, final_speed_mps=.001,
                lap_progress_m=12.5, reference_length_m=12.56)


def test_nominal_recovery_stop_reports_cause_without_waiting_for_timeout():
    with pytest.raises(RuntimeError, match='Recovery stopped; re-enable required'):
        integration.check_nominal_status('READY', 'Recovery stopped; re-enable required')


@pytest.mark.parametrize('scenario', ['manual', 'rc_loss'])
def test_authority_withdrawal_requires_latched_fault_and_zero_output(scenario):
    result=metrics('FAULT')
    result['fault_detection_s']=.1
    integration.check_scenario_acceptance(result,scenario,1.,.4,0.,0.)
    result['status_at_end']='RUNNING'
    with pytest.raises(AssertionError):
        integration.check_scenario_acceptance(result,scenario,1.,.4,0.,0.)


@pytest.mark.parametrize('scenario', ['solver_stall', 'solver_crash'])
def test_worker_recovery_accepts_moving_resume_or_latched_stop(scenario):
    result=metrics('RUNNING')
    result.update(worker_restarts=1,worker_ready=True,recovery_observed=True,
                  post_restart_activations=2,final_speed_mps=.2)
    integration.check_scenario_acceptance(result,scenario,1.,.4,.2,.2)
    result['post_restart_activations']=1
    with pytest.raises(AssertionError):
        integration.check_scenario_acceptance(result,scenario,1.,.4,.2,.2)
    result.update(status_at_end='READY',final_speed_mps=.001,
                  reason_at_end='Recovery stopped; re-enable required',
                  stopped_latch_duration_s=.6)
    integration.check_scenario_acceptance(result,scenario,1.,.4,0.,0.)


def test_nominal_geometry_and_finish_limits_remain_strict():
    result=metrics()
    integration.check_scenario_acceptance(result,'nominal',1.,.4,0.,0.)
    result['minimum_footprint_margin_m']=-.001
    with pytest.raises(AssertionError):
        integration.check_scenario_acceptance(result,'nominal',1.,.4,0.,0.)


def test_speed_attainment_includes_slow_recovery_samples_in_central_lap():
    fractions=np.arange(10)/10
    angles=2*np.pi*fractions
    speed=np.array([0.,.4,.5,.6,.1,.2,.3,.7,1.,0.])
    samples=np.c_[fractions,2*np.cos(angles),2*np.sin(angles),speed,
                  np.zeros((10,3))]
    report=integration.speed_attainment(samples,4*np.pi,1.,(2.,0.))
    assert report['central_sample_count']==7
    assert report['central_speed_mean_mps']==pytest.approx(.4)
    assert report['central_speed_p50_mps']==pytest.approx(.4)
    assert report['central_speed_p50_ratio']==pytest.approx(.4)
    assert not report['speed_tracking_pass']


def test_speed_attainment_uses_requested_speed_and_rejects_missing_central_motion():
    fractions=np.linspace(0.,.9,100)
    angles=2*np.pi*fractions
    samples=np.c_[fractions,2*np.cos(angles),2*np.sin(angles),
                  np.full(100,.45),np.zeros((100,3))]
    report=integration.speed_attainment(samples,4*np.pi,.5,(2.,0.))
    assert report['central_speed_p50_ratio']==pytest.approx(.9)
    assert report['speed_tracking_pass']
    report=integration.speed_attainment(samples[:5],4*np.pi,.5,(2.,0.))
    assert report['central_sample_count']==0
    assert report['central_speed_p50_mps'] is None
    assert not report['speed_tracking_pass']


@pytest.mark.parametrize('horizon', [10,15,20])
def test_same_horizon_reaches_offline_cache_and_online_ros_node(horizon,tmp_path,monkeypatch):
    from aims_mpcc import prepare_solver
    config_file=Path(integration.__file__).parents[1]/'config/synthetic.yaml'
    config=load_config(config_file)
    angle=np.arange(16)*2*np.pi/16
    geometry={key:getattr(config,key) for key in ('wheelbase','rear_offset','half_width',
                                                'half_length','front_extent','rear_extent')}
    path=ReferencePath(np.c_[2*np.cos(angle),2*np.sin(angle)],.9,.9,
                       metadata=dict(closed_lap=True,vehicle_geometry=geometry))
    path.save(tmp_path/'reference')
    args=SimpleNamespace(output=str(tmp_path),vehicle_config=str(config_file),
                         reuse_prepared_reference=True,prepare_only=True,
                         backend='qp',solve_frequency=20.,horizon=horizon)
    prepared=[]
    monkeypatch.setattr(prepare_solver,'main',prepared.append)
    integration.run(args)
    assert prepared[0][prepared[0].index('--horizon')+1]==str(horizon)
    ros_arguments=[]
    def capture_init(*,args):
        ros_arguments.extend(args)
        raise RuntimeError('ROS arguments captured')
    monkeypatch.setattr(integration.rclpy,'init',capture_init)
    args.prepare_only=False
    with pytest.raises(RuntimeError,match='ROS arguments captured'):
        integration.run(args)
    assert f'horizon:={horizon}' in ros_arguments
