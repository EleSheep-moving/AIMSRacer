import json
import hashlib
from types import SimpleNamespace
import pytest
from aims_calibration import cli


def stage_a_payload():
    configuration=dict(stage='speed_hold',speeds=[1.+.5*i for i in range(9)],
                       repeats=1,measurement_seconds=5.)
    digest=hashlib.sha256(json.dumps(configuration,sort_keys=True,separators=(',',':')).encode()).hexdigest()
    return dict(configuration=configuration,
        metadata=dict(evidence_valid=True,configuration_sha256=digest,map_sha256='map',reference_sha256='reference'),
        trials=[dict(trial_id=i+1,plan_index=i,stage='speed_hold',requested_target=v,
                     success=True,evidence_valid=True,duration_s=5.01,samples=250,
                     mean_current_motor=8.9+.7*i,std_current_motor=.7)
                for i,v in enumerate(configuration['speeds'])])


def test_baseline_export_and_sweep_preview_are_offline(tmp_path,capsys):
    source=tmp_path/'trials.json';source.write_text(json.dumps(stage_a_payload()))
    baseline=tmp_path/'baseline_current.json'
    assert cli.baseline_main([str(source),'--output',str(baseline)])==0
    assert json.loads(baseline.read_text())['points'][0]['mean_current_motor']==pytest.approx(8.9)
    capsys.readouterr()
    assert cli.plan_main(['--stage','accel_interval','--base-current-file',str(baseline)])==0
    report=json.loads(capsys.readouterr().out)
    assert report['plan'][0]['current']==pytest.approx(13.9)
    assert report['plan'][0]['start_speed']==1. and report['plan'][0]['end_speed']==1.5
    assert report['plan'][-1]['current']==100. and report['plan'][-1]['end_speed']==5.
    assert cli.plan_main(['--stage','decel_current','--base-current-file',str(baseline)])==0
    report=json.loads(capsys.readouterr().out)
    kinds={r['kind'] for r in report['plan']}
    assert kinds=={'positive_deceleration','coast','braking'}


def test_baseline_never_rewrites_source_and_invalid_results_refuse(tmp_path):
    path=tmp_path/'trials.json';original=json.dumps(stage_a_payload());path.write_text(original)
    assert cli.baseline_main([str(path),'--output',str(path)])==2
    assert path.read_text()==original
    data=stage_a_payload();data['trials'][0]['success']=False
    path.write_text(json.dumps(data))
    assert cli.baseline_main([str(path),'--output',str(tmp_path/'baseline.json')])==2
    assert not (tmp_path/'baseline.json').exists()


def test_baseline_voltage_uses_only_admitted_a_segments(tmp_path):
    import csv
    from aims_calibration.planner import load_baseline
    data=stage_a_payload();rows=[]
    for i,trial in enumerate(data['trials']):
        start=100.+i*10.;end=start+5.01
        trial.update(samples=2,segments=[dict(start_stamp=start,end_stamp=end,
            start_telemetry_stamp=start,end_telemetry_stamp=end,samples=2)])
        rows.extend([dict(wheel_stamp=t,telemetry_stamp=t,voltage_input=16.-.1*i) for t in (start,end)])
        rows.append(dict(wheel_stamp=end+1.,telemetry_stamp=end+1.,voltage_input=99.))
    source=tmp_path/'trials.json';source.write_text(json.dumps(data));original=source.read_bytes()
    with (tmp_path/'observations.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=['wheel_stamp','telemetry_stamp','voltage_input'])
        writer.writeheader();writer.writerows(rows)
    output=tmp_path/'baseline.json'
    assert cli.baseline_main([str(source),'--output',str(output)])==0
    baseline=load_baseline(output)
    assert baseline['points'][0]['voltage_input']==16.
    assert baseline['points'][-1]['voltage_input']==pytest.approx(15.2)
    assert baseline['points'][0]['voltage_samples']==2
    assert len(baseline['source']['observations_sha256'])==64
    assert source.read_bytes()==original


def test_baseline_refuses_incomplete_admitted_voltage_evidence(tmp_path):
    data=stage_a_payload()
    data['trials'][0]['segments']=[dict(start_stamp=1.,end_stamp=6.,start_telemetry_stamp=1.,end_telemetry_stamp=6.,samples=250)]
    source=tmp_path/'trials.json';source.write_text(json.dumps(data))
    (tmp_path/'observations.csv').write_text('wheel_stamp,telemetry_stamp,voltage_input\n1,1,16\n')
    assert cli.baseline_main([str(source),'--output',str(tmp_path/'baseline.json')])==2


def test_sweep_monitor_limit_does_not_change_stage_a():
    assert cli.acquisition_timeout('speed_hold',None)==900.
    assert cli.acquisition_timeout('accel_interval',None)==7200.
    assert cli.acquisition_timeout('decel_current',None)==7200.
    assert cli.acquisition_timeout('decel_current',15.)==15.


def test_help_and_results_do_not_import_ros(tmp_path,capsys):
    for main in (cli.inspect_main,cli.start_main,cli.stop_main,cli.results_main):
        with pytest.raises(SystemExit) as exit:
            main(['--help'])
        assert exit.value.code==0
    path=tmp_path/'trials.json';path.write_text(json.dumps({}))
    assert cli.results_main([str(path)])==3
    assert 'metadata_evidence_invalid' in capsys.readouterr().out


def test_start_never_retries_enable_on_uncertain_response(monkeypatch):
    calls=[]
    class Monitor:
        def __init__(self,*args): pass
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def wait_status(self,timeout):
            return {'preflight_json':json.dumps({'ready':True}), 'start_ready':'true'}
        def request(self,value,timeout):
            calls.append(value)
            if value: raise RuntimeError('service response timeout')
    monkeypatch.setattr(cli,'Monitor',Monitor)
    monkeypatch.setattr(cli,'check_start',lambda data:[])
    assert cli.start_main(['--timeout','1'])==3
    assert calls==[True,False]


def test_start_refusal_never_calls_enable(monkeypatch):
    calls=[]
    class Monitor:
        def __init__(self,*args): pass
        def __enter__(self): return self
        def __exit__(self,*args): pass
        def wait_status(self,timeout): return {'preflight_json':'{}'}
        def request(self,*args): calls.append(args)
    monkeypatch.setattr(cli,'Monitor',Monitor)
    assert cli.start_main([])==2
    assert calls==[]


def test_late_fault_after_complete_cannot_accept_old_checkpoint(monkeypatch,tmp_path):
    (tmp_path/'trials.json').write_text(json.dumps(stage_a_payload()))
    calls=[];clock=[0.]
    def monotonic():clock[0]+=.2;return clock[0]
    class Monitor:
        def __init__(self,*args):
            self.status_received=0.;self.spins=0
            self.values=dict(state='COMPLETE',enable_count='1',stage='speed_hold',preflight_json='{}',
                output_directory=str(tmp_path),logging_error='',logging_dropped='0',
                logging_committed_trials='9',trials='9',logging_committed_skips='0',skipped_trials='0')
        def __enter__(self):return self
        def __exit__(self,*args):pass
        def wait_status(self,*args):self.status_received=clock[0];return self.values
        def request(self,value,*args):calls.append(value)
        def stationary(self):return True
        def spin(self):
            self.spins+=1;self.status_received=clock[0]
            if self.spins>1:self.values.update(state='FAULT',logging_error='queue overflow',logging_dropped='1')
    monkeypatch.setattr(cli,'Monitor',Monitor)
    monkeypatch.setattr(cli,'check_start',lambda data:[])
    monkeypatch.setattr(cli,'time',SimpleNamespace(monotonic=monotonic))
    assert cli.start_main(['--timeout','5'])==3
    assert calls==[True,False]


def test_operator_interrupt_is_deferred_until_native_spin_returns():
    monitor=object.__new__(cli.Monitor);monitor.interrupted=False;completed=[]
    def spin_once(*args,**kwargs):
        monitor._interrupt(None,None)
        completed.append(True)
    monitor.rclpy=SimpleNamespace(spin_once=spin_once);monitor.node=object()
    with pytest.raises(KeyboardInterrupt):monitor.spin()
    assert completed==[True] and not monitor.interrupted


def test_repeated_interrupt_does_not_break_disable_response(capsys):
    monitor=object.__new__(cli.Monitor);monitor.interrupted=True;monitor.node=object()
    finished=[False]
    future=SimpleNamespace(done=lambda:finished[0],result=lambda:SimpleNamespace(success=True,message='Stopped'))
    monitor.SetBool=SimpleNamespace(Request=lambda:SimpleNamespace(data=None))
    monitor.client=SimpleNamespace(wait_for_service=lambda **kwargs:True,call_async=lambda request:future)
    def wait(*args,**kwargs):monitor._interrupt(None,None);finished[0]=True
    monitor.rclpy=SimpleNamespace(spin_once=wait)
    monitor.request(False,1.)
    assert 'Stopped' in capsys.readouterr().out
