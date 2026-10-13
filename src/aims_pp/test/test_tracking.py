import math

import pytest

from conftest import health_values, refresh


def test_health_anchor_heartbeat_epoch_and_replay():
    from aims_pp.tracking import LocalizationHealth
    h = LocalizationHealth()
    assert not h.observe(health_values(), 10_000_000_000, 10.)
    assert h.qualified and h.usable(10_100_000_000, 10.1)
    h.observe(health_values(health_sequence=2), 10_900_000_000, 10.9)
    assert h.usable(10_900_000_000, 10.9)
    assert not h.usable(11_100_000_000, 11.1)  # heartbeat cannot renew anchor
    assert h.observe(health_values(epoch='two'), 10_000_000_000, 10.)
    h.observe(health_values(epoch='one', health_sequence=8), 10_000_000_000, 10.)
    assert h.epoch == 'two'


def test_loss_cannot_resume_same_anchor_and_mutation_rejected():
    from aims_pp.tracking import LocalizationHealth
    h = LocalizationHealth()
    h.observe(health_values(), 10_000_000_000, 10.)
    loss = health_values(health_sequence=2)
    loss['ready'] = 'false'
    h.observe(loss, 10_000_000_000, 10.)
    h.observe(health_values(health_sequence=3), 10_000_000_000, 10.)
    assert not h.usable(10_000_000_000, 10.)
    h.observe(health_values(sequence=2, health_sequence=4), 10_000_000_000, 10.)
    assert h.usable(10_000_000_000, 10.)
    mutated = health_values(sequence=2, health_sequence=5)
    mutated['map_odom_x'] = '1'
    h.observe(mutated, 10_000_000_000, 10.)
    assert not h.qualified


def test_input_authority_freshness_exclusivity_map(inputs):
    assert inputs.reason(10_000_000_000, 10.) == ''
    assert 'publisher' in inputs.reason(10_000_000_000, 10., 2).lower()
    inputs.observe_authority(True, 10.11)
    assert 'odometry' in inputs.reason(10_110_000_000, 10.11).lower()
    inputs.observe_authority(False, 10.)
    assert 'authority' in inputs.reason(10_000_000_000, 10.).lower()
    refresh(inputs, 10., 10.)
    inputs.observe_map('b'*64)
    assert 'map' in inputs.reason(10_000_000_000, 10.).lower()


def test_odom_frames_time_finite_and_rear_transform(inputs):
    assert inputs.map_pose() == pytest.approx((2., 0., math.pi/2, 1.))
    with pytest.raises(ValueError): inputs.observe_odometry(0, 0, 0, 1, 11_000_000_000, 10_000_000_000, 10.)
    with pytest.raises(ValueError): inputs.observe_odometry(0, 0, 0, 1, 10_000_000_000, 10_000_000_000, 10., frame_id='map')
    with pytest.raises(ValueError): inputs.observe_odometry(float('nan'), 0, 0, 1, 10_000_000_000, 10_000_000_000, 10.)
    with pytest.raises(ValueError): inputs.map_pose()


def test_runtime_starts_disabled_requires_reenable_after_fault(inputs, reference):
    from aims_pp.tracking import PPTrackingRuntime
    runtime = PPTrackingRuntime(inputs, reference, speed_limit=1.)
    assert runtime.tick(10_000_000_000, 10.)['speed'] == 0
    assert runtime.enable(10_000_000_000, 10.)[0]
    assert runtime.tick(10_000_000_000, 10.)['speed'] == 1.
    inputs.observe_authority(False, 10.)
    assert runtime.tick(10_000_000_000, 10.)['speed'] == 0
    refresh(inputs, 10., 10.)
    assert runtime.tick(10_000_000_000, 10.)['speed'] == 0
    assert runtime.enable(10_000_000_000, 10.)[0]
    runtime.disable()
    assert runtime.tick(10_000_000_000, 10.)['speed'] == 0


def test_clock_reset_and_epoch_latch(inputs, reference):
    from aims_pp.tracking import PPTrackingRuntime
    runtime = PPTrackingRuntime(inputs, reference)
    runtime.enable(10_000_000_000, 10.)
    runtime.tick(10_000_000_000, 10.)
    assert runtime.tick(9_000_000_000, 10.01)['speed'] == 0
    assert runtime.phase == 'FAULT'
    assert not runtime.enable(9_000_000_000, 10.01)[0]


def test_one_lap_association_wrap_does_not_complete_on_enable(inputs, reference):
    from aims_pp.tracking import PPTrackingRuntime
    runtime = PPTrackingRuntime(inputs, reference)
    start = reference.length-.05
    pose = reference.at(start)
    inputs.source_ros_ns = -1
    inputs.observe_odometry(pose['x'], pose['y'], pose['yaw'], 1., 10_000_000_000, 10_000_000_000, 10.)
    assert runtime.enable(10_000_000_000, 10.)[0]
    assert runtime.tick(10_000_000_000, 10.)['speed'] > 0
    # Dense forward observations; never infer a lap from initial association.
    for i in range(1, 201):
        now = 10. + i*.07
        p = reference.at(start+i*reference.length/200)
        refresh(inputs, now, now, sequence=i+1, pose=(p['x'],p['y'],p['yaw'],1.))
        runtime.tick(round(now*1e9), now)
    assert runtime.phase == 'COMPLETE'
    assert not runtime.enabled


def test_calibration_localization_gate_needs_no_fake_speed_authority(inputs):
    inputs.observe_authority(False,10.)
    inputs.command_valid = False
    assert inputs.localization_reason(10_000_000_000,10.) == ''
    assert inputs.reason(10_000_000_000,10.)


def test_transient_map_withdrawal_and_epoch_change_stay_latched(inputs,reference):
    from aims_pp.tracking import PPTrackingRuntime
    runtime = PPTrackingRuntime(inputs,reference)
    assert runtime.enable(10_000_000_000,10.)[0]
    inputs.observe_map_valid(False)
    inputs.observe_map_valid(True)
    assert runtime.tick(10_000_000_000,10.)['speed']==0
    assert runtime.phase=='FAULT'
    assert runtime.enable(10_000_000_000,10.)[0]
    inputs.observe_health(health_values(epoch='two'),10_000_000_000,10.)
    refresh(inputs,10.02,10.02,sequence=2)  # retired epoch cannot restore health
    assert runtime.tick(10_020_000_000,10.02)['speed']==0
    assert runtime.phase=='FAULT'


def test_duplicate_physical_sources_cannot_renew_receipt(inputs):
    inputs.observe_authority(True,10.09)
    inputs.observe_odometry(2.,0.,math.pi/2,1.,10_000_000_000,10_090_000_000,10.09)
    inputs.observe_command(0,0.,1.,10_000_000_000,10_090_000_000,10.09)
    assert inputs.received==10.
    assert inputs.command_received==10.
    inputs.observe_authority(True,10.11)
    assert inputs.localization_reason(10_000_000_000,10.11)


def test_map_alignment_and_rear_axle_conversion(reference):
    from dataclasses import replace
    from aims_pp.core import PPConfig
    from aims_pp.tracking import TrackingInputs
    reference.metadata['vehicle_geometry']['rear_offset']=.1
    inputs = TrackingInputs(replace(PPConfig(),rear_offset=.1),reference)
    values = health_values()
    values.update(map_odom_x='10',map_odom_y='-3',map_odom_qz=str(math.sin(math.pi/4)),map_odom_qw=str(math.cos(math.pi/4)))
    inputs.observe_health(values,10_000_000_000,10.)
    inputs.observe_odometry(1.,2.,0.,1.,10_000_000_000,10_000_000_000,10.)
    assert inputs.map_pose()==pytest.approx((8.,-2.1,math.pi/2,1.))


def test_repeat_laps_and_forward_heading_gate(inputs,reference):
    from aims_pp.tracking import PPTrackingRuntime
    runtime = PPTrackingRuntime(inputs,reference,repeat_laps=True)
    assert runtime.enable(10_000_000_000,10.)[0]
    for i in range(1,221):
        now=10.+i*.07
        p=reference.at(i*reference.length/200)
        refresh(inputs,now,now,sequence=i+1,pose=(p['x'],p['y'],p['yaw'],1.))
        output=runtime.tick(round(now*1e9),now)
    assert runtime.enabled and runtime.traveled>reference.length and output['speed']>0
    runtime.disable()
    now+=.07
    refresh(inputs,now,now,sequence=999,pose=(p['x'],p['y'],p['yaw']+.3,1.))
    # Large heading mismatch with fresh initial association must reject enable.
    inputs.pose=(p['x'],p['y'],p['yaw']+math.pi,1.)
    assert not runtime.enable(round(now*1e9),now)[0]


def test_nominal_widths_gate_footprint_only_when_explicitly_enabled(reference):
    from dataclasses import replace
    from aims_pp.core import PPConfig
    from aims_pp.tracking import PPTrackingRuntime,TrackingInputs
    outside_nominal_width=(2.95,0.,math.pi/2,1.)
    default=TrackingInputs(PPConfig(),reference)
    refresh(default,10.,10.,pose=outside_nominal_width)
    runtime=PPTrackingRuntime(default,reference)
    assert runtime.enable(10_000_000_000,10.)[0]
    assert runtime.tick(10_000_000_000,10.)['speed']>0
    strict=TrackingInputs(replace(PPConfig(),enforce_corridor=True),reference)
    refresh(strict,10.,10.,pose=outside_nominal_width)
    accepted,reason=PPTrackingRuntime(strict,reference).enable(10_000_000_000,10.)
    assert not accepted
    assert 'footprint' in reason.lower()


def test_static_map_identity_survives_clock_rollback_but_dynamic_gates_reset(inputs,reference):
    from aims_pp.tracking import PPTrackingRuntime
    runtime=PPTrackingRuntime(inputs,reference)
    assert runtime.enable(10_000_000_000,10.)[0]
    assert runtime.tick(10_000_000_000,10.)['speed']>0
    # The map identity was published only during the initial fixture setup.
    inputs.observe_clock(9_000_000_000)
    assert inputs.map_identity==reference.map_sha256
    assert not inputs.map_valid and not inputs.authority
    assert inputs.pose is None and not inputs.command_valid
    assert not inputs.health.qualified
    assert runtime.tick(9_000_000_000,10.01)['speed']==0
    assert runtime.phase=='FAULT'
    assert not runtime.enable(9_000_000_000,10.01)[0]
    # Dynamic streams resume with a new qualified anchor; no new SHA message.
    inputs.observe_map_valid(True)
    values=health_values(stamp_ns=9_000_000_000,epoch='after-reset')
    inputs.observe_health(values,9_000_000_000,10.02)
    inputs.observe_authority(True,10.02)
    inputs.observe_command(0,0.,1.,9_000_000_000,9_000_000_000,10.02)
    inputs.observe_odometry(2.,0.,math.pi/2,1.,9_000_000_000,9_000_000_000,10.02)
    assert inputs.reason(9_000_000_000,10.02)==''
    assert runtime.tick(9_000_000_000,10.02)['speed']==0
    assert runtime.enable(9_000_000_000,10.02)[0]
    assert runtime.tick(9_000_000_000,10.02)['speed']>0


def test_rollback_retained_identity_still_requires_matching_qualified_health(inputs,reference):
    from aims_pp.tracking import PPTrackingRuntime
    inputs.observe_clock(9_000_000_000)
    inputs.observe_map_valid(True)
    inputs.observe_authority(True,10.02)
    inputs.observe_command(0,0.,1.,9_000_000_000,9_000_000_000,10.02)
    values=health_values(stamp_ns=9_000_000_000,epoch='after-reset')
    values['map_sha256']='b'*64
    inputs.observe_health(values,9_000_000_000,10.02)
    inputs.observe_odometry(2.,0.,math.pi/2,1.,9_000_000_000,9_000_000_000,10.02)
    assert not PPTrackingRuntime(inputs,reference).enable(9_000_000_000,10.02)[0]
    values['map_sha256']=reference.map_sha256
    values['alignment_valid']='false'
    inputs.observe_health(values,9_000_000_000,10.02)
    assert not PPTrackingRuntime(inputs,reference).enable(9_000_000_000,10.02)[0]
