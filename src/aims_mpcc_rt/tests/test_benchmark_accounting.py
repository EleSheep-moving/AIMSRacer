"""Regression checks against sparse/stopped/failed benchmark false passes."""
import csv
import importlib.util
from pathlib import Path


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
