"""Regression checks against sparse/stopped/failed benchmark false passes."""
import csv
import importlib.util
from pathlib import Path
import pytest
from types import SimpleNamespace


def module(name):
    file=Path(__file__).resolve().parents[1]/'tools'/(name+'.py')
    spec=importlib.util.spec_from_file_location(name,file)
    answer=importlib.util.module_from_spec(spec);spec.loader.exec_module(answer)
    return answer


def nominal():
    return dict(requests=3600,failures=0,handover_rejected=0,late=0,
        max_consecutive_failures=0,measurement_s=180.,publications=9000,log_integrity_pass=True,
        complete=dict(p95=.005,p99=.007),publish_gap=dict(p99=.02,max=.021))


def test_sparse_and_three_consecutive_failures_cannot_qualify():
    qualify=module('nx_joint_load').timing_pass
    assert qualify(nominal(),20.,.05)
    sparse=nominal();sparse['requests']=1
    assert not qualify(sparse,20.,.05)
    burst=nominal();burst.update(failures=3,max_consecutive_failures=3)
    assert not qualify(burst,20.,.05)
    stopped=nominal();stopped['publications']=4000
    assert not qualify(stopped,20.,.05)
    inflight=nominal();inflight['inflight']=1
    assert not qualify(inflight,20.,.05)
    incomplete=nominal();incomplete['accounted']=3599
    assert not qualify(incomplete,20.,.05)
    retained=nominal();retained.update(requests=1800,eligible_slots=3600,skipped_pending_slots=1800,
                                     max_consecutive_skipped_pending_slots=4)
    assert qualify(retained,20.,.05)
    retained['eligible_slots']=4000
    assert not qualify(retained,20.,.05)


def test_window_counts_inactive_publications_and_activation_rejections(tmp_path):
    timing=module('acceptance').timing_report
    fields='event,steady_s,sequence,source_epoch,forecast_epoch,submitted,complete_s,core_s,native_s,preparation_s,validation_s,accepted,status,passes,violation,observation_age_s,publish_gap_s,speed,steering,reason'.split(',')
    rows=[]
    def row(event,stamp,sequence=0,accepted=1):
        values={k:0 for k in fields};values.update(event=event,steady_s=stamp,sequence=sequence,
            accepted=accepted,submitted=stamp-.01,complete_s=.001,publish_gap_s=.02,reason='')
        rows.append(values)
    row('request',.5,99,0) # startup is outside the measurement
    for sequence in (1,2,3):
        row('submission',1.+sequence*.05,sequence)
        row('request',1.+sequence*.05,sequence)
        row('activation',1.01+sequence*.05,sequence,0)
        row('publish',1.02+sequence*.05) # zero-sequence output still counts
    row('publish',2.5)
    file=tmp_path/'runtime.csv'
    with file.open('w') as handle:
        writer=csv.DictWriter(handle,fields);writer.writeheader();writer.writerows(rows)
    report=timing(file,.05,1.,2.)
    assert report['requests']==3 and report['failures']==0
    assert report['handover_rejected']==3 and report['max_consecutive_failures']==3
    assert report['publications']==3 and report['measurement_s']==1.
    assert not report['log_integrity_pass'] # missing final summary and unmatched startup result


def test_complete_cohort_counts_cutoff_inflight_and_rejection_separately(tmp_path):
    fields='event,steady_s,sequence,submitted,complete_s,core_s,native_s,preparation_s,validation_s,accepted,status,violation,observation_age_s,publish_gap_s,disposition'.split(',')
    rows=[]
    def add(event,seq,stamp,submitted=1.1,accepted=1,disposition=''):
        row={k:0 for k in fields}
        row.update(event=event,steady_s=stamp,sequence=seq,submitted=submitted,accepted=accepted,disposition=disposition,complete_s=stamp-submitted)
        rows.append(row)
    for seq in range(1,5):add('submission',seq,1.1)
    add('request',1,1.12)
    add('request',2,1.15,accepted=0,disposition='rejected')
    add('request',3,2.1)
    add('publish',1,1.14)
    add('summary',4,2.2)
    file=tmp_path/'runtime.csv'
    with file.open('w') as handle:
        writer=csv.DictWriter(handle,fields);writer.writeheader();writer.writerows(rows)
    report=module('acceptance').timing_report(file,.05,1.,2.)
    assert report['requests']==4
    assert report['inflight']==1 and report['cutoff']==1 and report['rejected']==1
    assert report['failures']==1
    assert report['accounted']==4
    assert report['request_to_first_publish']['p50']==pytest.approx(.04)
    assert report['complete']['max']==pytest.approx(.05)
    assert not report['log_integrity_pass']


def test_fault_requires_running_positive_output_and_reason_and_bound():
    assess=module('acceptance').fault_pass
    good=dict(positive_before=True,phase='FAULT',reason='Authority withdrawn',expected_reason='withdrawn',
              actionability_s=1.,fault_s=1.04,zero_s=1.06,zero_speed=0.)
    assert assess(**good)
    for change in ({'positive_before':False},{'phase':'READY'},{'reason':'Plan expired'},
                   {'zero_s':1.101},{'fault_s':1.101},{'zero_speed':.1}):
        assert not assess(**{**good,**change})


def test_deadline_notification_is_not_terminal_native_delivery():
    record=module('acceptance').record_worker_reply
    attempt=dict(submitted=1.,events=[],failed=False)
    record(attempt,dict(kind='skipped'),1.06)
    assert attempt['failed'] and 'terminal' not in attempt
    record(attempt,dict(kind='result',success=True,discarded=True),1.2)
    assert attempt['terminal']==1.2 and attempt['failed']


def test_independent_plant_motor_floor_acceleration_violation_is_reported():
    report=module('acceptance').plant_envelope_report
    config=SimpleNamespace(wheelbase=.36,accel_limit=.5,brake_limit=.5,lateral_accel_limit=1.,
                           longitudinal_envelope_accel=None,longitudinal_envelope_brake=None)
    metrics=report([0.,.2],[0.,0.],[1.,0.],config)
    assert metrics['model_utilization_max']==4.
    assert metrics['violation_samples']==1 and metrics['samples']==2


def test_operator_stop_waits_for_terminal_ready_after_stationary_confirmation():
    assess=module('acceptance').operator_stop_pass
    values=dict(positive_before=True,injected_s=1.,phase='READY',reason='Stopped',speed=.01,target_speed=0.)
    assert assess(**values)
    assert not assess(**{**values,'phase':'STOPPING'})
    assert not assess(**{**values,'speed':.06})
