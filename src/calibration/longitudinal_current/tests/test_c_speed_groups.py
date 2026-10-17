from copy import deepcopy
from dataclasses import asdict
import json
import pytest

from aims_calibration.planner import TrialPlan,build_sweep_plan,export_baseline
from aims_calibration.operator import summarize_results,canonical_hash
from aims_calibration.io import OutputLog
from test_engine import planned_engine,prepare_item,step,tracking_ticks
from test_planner import a_payload


def items():
    return [TrialPlan('decel_current',2.,1.,-5.,kind='braking'),
            TrialPlan('decel_current',2.,1.,-10.,kind='braking'),
            TrialPlan('decel_current',2.,1.,-15.,kind='braking'),
            TrialPlan('decel_current',1.5,1.,-5.,kind='braking')]


def saved(tmp_path,e):
    cfg=asdict(e.config);settings=asdict(e.tracking_config)
    metadata=dict(evidence_valid=True,plan_mode='manual',configuration_sha256=canonical_hash(cfg),
        tracking_configuration_sha256=canonical_hash(settings),
        decel_group_policy='skip_on_reverse_or_current_tracking_failure')
    plan=e.serializable_plan
    out=OutputLog(tmp_path,e.config,metadata,dict(plan=plan,plan_sha256=canonical_hash(plan),tracking_configuration=settings))
    out.trials(e.trials,skipped=e.skipped);out.close()
    return json.loads((out.directory/'trials.json').read_text())


def test_C_descending_default_and_historical_explicit_ascending_order():
    b=export_baseline(a_payload())
    p=build_sweep_plan('decel_current',None,b)
    assert list(dict.fromkeys(t.start_speed for t in p))==[5.,4.5,4.,3.5,3.,2.5,2.,1.5]
    old=build_sweep_plan('decel_current',[1.5,2.],b)
    assert list(dict.fromkeys(t.start_speed for t in old))==[1.5,2.]
    with pytest.raises(ValueError):build_sweep_plan('decel_current',[5.,3.,4.],b)
    with pytest.raises(ValueError):build_sweep_plan('accel_interval',[2.,1.5,1.],b)


def test_reverse_skips_one_speed_and_waits_for_neutral_and_forward_sources(tmp_path):
    e=planned_engine('decel_current',items());prepare_item(e)
    step(e,1.17,v=2.,current=-5.)
    req=step(e,1.18,v=-.075,current=-5.)
    assert e.enabled and e.state=='GROUP_RECOVER' and e.index==3
    assert req.mode==1 and req.current==0
    assert len(e.trials)==1 and e.trials[0]['reason']=='reverse_motion'
    assert [r['plan_index'] for r in e.skipped]==[1,2]
    for t in (1.19,1.22,1.26):
        assert step(e,t,v=-.075,current=0.).current==0
        assert e.index==3 and len(e.skipped)==2
    step(e,1.27,v=0.,current=0.)
    assert e.state=='GROUP_RECOVER'
    req=step(e,1.38,v=0.,current=0.)
    assert e.state=='PREPARE' and req.speed==1.5
    data=saved(tmp_path,e);summary=summarize_results(data)
    assert summary['evidence_valid'],summary
    assert summary['skipped']==2 and not summary['coverage_complete']
    assert summary['accepted']==0


def test_reverse_after_success_revises_real_attempt_without_fabricating_one(tmp_path):
    e=planned_engine('decel_current',items());prepare_item(e)
    step(e,1.17,v=2.,current=-5.);step(e,1.27,v=1.,current=-5.)
    assert e.trials[0]['success'] and e.index==1
    step(e,1.28,v=-.075,current=-2.)
    assert e.index==3 and len(e.trials)==1 and e.trials[0]['attempt_id']==1
    assert e.trials[0]['response_completed'] and not e.trials[0]['success']
    assert e.results_revision==1
    assert summarize_results(saved(tmp_path,e))['evidence_valid']


def test_reverse_after_last_current_keeps_next_lower_speed_available(tmp_path):
    e=planned_engine('decel_current',[items()[0],items()[-1]]);prepare_item(e)
    step(e,1.17,v=2.,current=-5.);step(e,1.27,v=1.,current=-5.)
    step(e,1.28,v=-.075,current=-2.)
    assert e.index==1 and not e.skipped and e.results_revision==1
    assert len(e.trials)==1 and e.state=='GROUP_RECOVER'
    assert summarize_results(saved(tmp_path,e))['evidence_valid']


@pytest.mark.parametrize('fault',[dict(rc_valid=False),dict(fault_code=3),dict(tracking_reason='localization_lost')])
def test_other_faults_remain_fatal_during_reverse(fault):
    e=planned_engine('decel_current',items(),steering_source='pp');prepare_item(e);step(e,1.17,v=2.,current=-5.)
    step(e,1.18,v=-.075,current=-5.,**fault)
    assert e.state=='FAULT' and not e.skipped


def test_reverse_outside_a_real_C_current_attempt_remains_fatal_and_B_unchanged():
    e=planned_engine('decel_current',items());e.enable(True)
    step(e,1.,v=2.);step(e,1.01,v=-.075)
    assert e.state=='FAULT' and not e.skipped
    b=planned_engine();prepare_item(b);step(b,1.17,v=-.075)
    assert b.state=='FAULT' and b.reason=='reverse_motion'


def test_reverse_recovery_does_not_hide_persistent_reverse_motion():
    e=planned_engine('decel_current',items(),trial_timeout=.3);prepare_item(e)
    step(e,1.17,v=2.,current=-5.);step(e,1.18,v=-.075,current=-5.)
    for t in (1.28,1.38,1.49):step(e,t,v=-.075,current=0.)
    assert e.state=='FAULT' and e.reason=='decel_group_recovery_timeout' and len(e.skipped)==2


def test_current_failure_skip_is_evidenced_and_never_complete_measurement_coverage(tmp_path):
    e=planned_engine('decel_current',items());prepare_item(e)
    t=tracking_ticks(e,1.16,-1.)
    assert e.index==3 and e.state=='GROUP_RECOVER'
    step(e,t+.05,v=0.,current=0.);step(e,t+.17,v=0.,current=0.)
    prepare_item(e,t+.2);step(e,t+.38,v=1.5,current=-5.);step(e,t+.48,v=1.,current=-5.)
    summary=summarize_results(saved(tmp_path,e))
    assert summary['execution_complete'] and summary['evidence_valid'],summary
    assert summary['accepted']==1 and summary['skipped']==2
    assert not summary['coverage_complete'] and summary['coverage_gaps']==[0,1,2]


@pytest.mark.parametrize('damage',['positive_wheel','stale_wheel','wrong_speed','wrong_evidence','wrong_policy'])
def test_skip_acceptance_rejects_fabricated_or_wrong_scope_reverse_evidence(tmp_path,damage):
    e=planned_engine('decel_current',items());prepare_item(e);step(e,1.17,v=2.,current=-5.)
    step(e,1.18,v=-.075,current=-5.);d=saved(tmp_path,e)
    if damage=='positive_wheel':
        d['trials'][0]['decision_evidence']['observation']['velocity']=.075
        for s in d['skipped']:s['evidence']=deepcopy(d['trials'][0]['decision_evidence'])
    elif damage=='stale_wheel':d['trials'][0]['decision_evidence']['observation']['wheel_stamp']=1.
    elif damage=='wrong_speed':d['skipped'][0]['start_speed']=1.5
    elif damage=='wrong_evidence':d['skipped'][0]['evidence']['observed_at']=2.
    else:d['metadata'].pop('decel_group_policy')
    assert not summarize_results(d)['evidence_valid']
