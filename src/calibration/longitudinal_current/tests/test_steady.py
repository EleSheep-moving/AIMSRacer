"""Native source-clock evidence, independent of control timer frequency."""
import math
from dataclasses import FrozenInstanceError
import pytest
from aims_calibration import steady


def feed(detector, start=0., end=3., velocity=lambda t: 2., current=lambda t: 10.,
         target=10., ack=0., epoch=0.):
    evidence=None
    for i in range(round(start*100), round(end*100)+1):
        t=i/100
        evidence=detector.observe(epoch+t,velocity(t),epoch+t,current(t),target,epoch+ack)
    return evidence


def test_native_stability_needs_window_confirmation_and_post_ack_settle():
    d=steady.SteadyDetector()
    e=feed(d,end=2.24)
    assert not e['steady']
    e=feed(d,start=2.25,end=2.25)
    assert e['steady']
    assert e['sample_count']>=100
    assert e['slope']==pytest.approx(0.) and e['speed_span']==0.
    assert e['current_good_fraction']==1.
    assert e['window_start_stamp']>=.25
    assert e['confirmation_start_stamp']==pytest.approx(1.25)
    assert e['wheel_stamp']==2.25 and e['telemetry_stamp']==2.25


@pytest.mark.parametrize('epoch',[0.,1_800_000_000.])
def test_small_noise_is_stable_with_epoch_safe_duration_arithmetic(epoch):
    e=feed(steady.SteadyDetector(),velocity=lambda t: 2+.025*math.sin(t*30),epoch=epoch)
    assert e['steady'] and abs(e['slope'])<.1 and e['speed_span']<.15


@pytest.mark.parametrize('velocity',[
    lambda t:2+.2*t,
    lambda t:2+.1*math.sin(t*30),
])
def test_ramp_and_large_oscillation_are_not_stable(velocity):
    assert not feed(steady.SteadyDetector(),end=4.,velocity=velocity)['steady']


def test_duplicate_wheel_and_iq_stamps_cannot_create_samples_or_confirmation():
    d=steady.SteadyDetector()
    for _ in range(1000):e=d.observe(1.,2.,1.,10.,10.,0.)
    assert e['sample_count']==1 and not e['steady']


def test_rings_use_independent_native_sample_rates():
    d=steady.SteadyDetector()
    for i in range(301):
        t=i/100
        iq_t=(i//5)*.05
        e=d.observe(t,2.,iq_t,10.,10.,0.)
    assert e['steady'] and e['sample_count']>=100
    assert 10<=e['current_sample_count']<=12


@pytest.mark.parametrize('source',['wheel','iq','both'])
def test_source_gap_resets_window_and_confirmation(source):
    d=steady.SteadyDetector();assert feed(d)['steady']
    w=3.2 if source in ('wheel','both') else 3.01
    iq=3.2 if source in ('iq','both') else 3.01
    assert not d.observe(w,2.,iq,10.,10.,0.)['steady']
    assert not feed(d,start=3.21,end=4.3)['steady']
    assert feed(d,start=4.31,end=5.3)['steady']


def test_frozen_telemetry_cannot_confirm_stability():
    d=steady.SteadyDetector()
    for i in range(301):e=d.observe(i/100,2.,0.,10.,10.,0.)
    assert not e['steady']


@pytest.mark.parametrize('event',['backwards','nan','mode','target','ack'])
def test_invalidating_events_require_fresh_contiguous_evidence(event):
    d=steady.SteadyDetector();assert feed(d)['steady']
    args=[3.01,2.,3.01,10.,10.,0.,True]
    if event=='backwards':args[0]=2.9
    elif event=='nan':args[1]=float('nan')
    elif event=='mode':args[-1]=False
    elif event=='target':args[4]=20.
    elif event=='ack':args[5]=3.
    assert not d.observe(*args)['steady']
    assert not feed(d,start=3.02,end=4.1)['steady']


@pytest.mark.parametrize('target,actual,accepted',[(0.,0.,True),(0.,2.,False),(10.,0.,False),(-20.,-21.5,True),(-20.,-23.,False)])
def test_current_tracking_is_required_for_all_signs_including_coast(target,actual,accepted):
    e=feed(steady.SteadyDetector(),target=target,current=lambda t:actual)
    assert e['steady'] is accepted
    assert e['current_good_fraction']==float(accepted)


def test_current_tracking_loss_restarts_confirmation():
    d=steady.SteadyDetector();assert feed(d)['steady']
    assert not feed(d,start=3.01,end=3.3,current=lambda t:0.)['steady']
    assert not feed(d,start=3.31,end=4.3)['steady']
    assert feed(d,start=4.31,end=5.)['steady']


def test_minimum_sample_counts_and_real_source_duration_both_apply():
    d=steady.SteadyDetector()
    for i in range(30):e=d.observe(.25+i*.01,2.,.25+i*.01,10.,10.,0.)
    assert not e['steady']
    sparse=steady.SteadyDetector(steady.SteadySettings(source_gap=.2))
    for i in range(51):e=sparse.observe(i*.1,2.,i*.1,10.,10.,0.)
    assert not e['steady'] and e['sample_count']<40


def test_settings_are_frozen_validated_and_reset_removes_evidence():
    settings=steady.SteadySettings()
    with pytest.raises(FrozenInstanceError):settings.window_seconds=2.
    for kwargs in [dict(window_seconds=0.),dict(min_samples=1),dict(current_good_fraction=1.1),dict(source_gap=float('nan'))]:
        with pytest.raises(ValueError):steady.SteadySettings(**kwargs)
    d=steady.SteadyDetector();assert feed(d)['steady'];d.reset()
    assert not d.observe(3.01,2.,3.01,10.,10.,0.)['steady']


def test_pre_ack_samples_never_count_even_with_zero_settle():
    d=steady.SteadyDetector(steady.SteadySettings(current_settle_seconds=0.))
    for i in range(301):
        e=d.observe(i/100,2.,i/100,10.,10.,3.)
    assert not e['steady'] and e['sample_count']==0
    assert not feed(d,start=3.01,end=5.,ack=3.)['steady']
    assert feed(d,start=5.01,end=5.01,ack=3.)['steady']


def test_ninety_percent_tracking_uses_fraction_of_native_iq_samples():
    settings=steady.SteadySettings(current_window_seconds=.49)
    d=steady.SteadyDetector(settings)
    # Exactly 50 samples in each half-second IQ ring, five outliers accepted.
    e=feed(d,end=3.,current=lambda t:0. if round(t*100)%10==0 else 10.)
    assert e['current_sample_count']==50
    assert e['current_good_fraction']==pytest.approx(.9)
    assert e['steady']
