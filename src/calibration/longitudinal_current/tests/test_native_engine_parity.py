"""Replay real Python state-machine tests against a standalone C++17 runtime.

This oracle uses Python only during testing. Engine runtime has no interpreter.
Tests that directly alter internal state are reported as skipped trace cases.
"""
import dataclasses
import functools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from collections import deque

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from aims_calibration.engine import Engine

KEYS=('state reason enabled selected index plan trials skipped results_revision hold_elapsed hold_segments '
      'steady_evidence tracking_status tracking_evidence handoff exit_prediction full_step retry_region retry_left '
      'last_prepare_wheel started stable_since phase_started phase_ack_stamp first_current_request_stamp '
      'starting_wheel_observation start_trigger_observation active_samples samples last_pair last_wheel last_telemetry '
      'armed_at last_now attempt_counts attempt_id attempt_number decel_owner_index '
      'group_recovery_started group_recovery_request_stamp group_recovery_ack_stamp group_forward_since '
      'group_iq_samples group_iq_stamp tracking_observations tracking_window last_tracking_stamp '
      'tracking_started_stamp hold_previous hold_segment start_velocity region history').split()

def clean(value):
    if dataclasses.is_dataclass(value):return clean(dataclasses.asdict(value))
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(tuple,list,deque)):return [clean(v) for v in value]
    if isinstance(value,float) and not math.isfinite(value):return None
    return value

def snapshot(engine):
    result={}
    for k in KEYS:
        value=getattr(engine,k)
        if k in ('samples','active_samples','tracking_observations','tracking_window'):
            result[k]=dict(size=len(value),last=clean(value[-1]) if value else None)
        else:result[k]=clean(value)
    return result

TRACES=[]
ACTIVE=None
DEPTH=0
ORIGINALS={name:getattr(Engine,name) for name in ('__init__','tick','sent','enable','fault','invalidate_evidence')}

def descriptor(engine):
    baseline=None;limits=None
    if engine.protection:
        points=[]
        for i in range(9):
            speed=1.+i*.5
            try:iq=engine.protection.holding_current(speed)
            except (ValueError,KeyError):continue
            points.append(dict(speed=speed,mean_iq=iq))
        baseline=dict(points=points)
        rows=[]
        for item in engine.trial_plan:
            try:bound=engine.protection.response_limit(item)
            except (ValueError,TypeError,AssertionError):bound=2.
            rows.append(dict(stage=item.stage,start_speed=item.start_speed,current=item.current,acceleration_bound=bound))
        limits=dict(limits=rows)
    return dict(config=clean(engine.config),plan=engine.serializable_plan,
                tracking=clean(engine.tracking_config),protection=clean(engine.protection.settings) if engine.protection else None,
                baseline=baseline,response_limits=limits)

def install():
    def protection_identity(engine):
        p=engine.protection
        return (id(p),id(p.settings),id(p.holding_current),id(p.response_limit)) if p else None
    def init(self,*args,**kwargs):
        ORIGINALS['__init__'](self,*args,**kwargs)
        self._native_trace=dict(prepared=descriptor(self),events=[],expected=[],name=ACTIVE,mutated=False,previous=snapshot(self))
        self._native_trace['protection_identity']=protection_identity(self)
        TRACES.append(self._native_trace)
    Engine.__init__=init
    for name in ('tick','sent','enable','fault','invalidate_evidence'):
        original=ORIGINALS[name]
        def wrapped(self,*args,_name=name,_original=original,**kwargs):
            global DEPTH
            trace=getattr(self,'_native_trace',None)
            top=DEPTH==0 and trace is not None
            if top:
                if snapshot(self)!=trace['previous']:trace['mutated']=True
                if protection_identity(self)!=trace['protection_identity']:trace['mutated']=True
                event=dict(op=_name)
                if _name=='tick':event.update(now=args[0],observation=clean(args[1]))
                elif _name=='sent':event['request']=clean(args[0])
                elif _name=='enable':
                    event['value']=args[0]
                    now=args[1] if len(args)>1 else kwargs.get('now')
                    if now is not None:event['now']=now
                elif _name=='fault':event['reason']=args[0]
                else:event.update(first_trial=args[0],reason=args[1])
            DEPTH+=1
            try:result=_original(self,*args,**kwargs)
            finally:DEPTH-=1
            if top:
                trace['events'].append(event)
                trace['expected'].append(dict(request=clean(result) if _name=='tick' else None,snapshot=snapshot(self)))
                trace['previous']=snapshot(self)
            return result
        setattr(Engine,name,wrapped)

def compare(expected,actual,path=''):
    if isinstance(expected,dict):
        for k,v in expected.items():
            assert k in actual,f'{path}: missing {k}'
            compare(v,actual[k],path+'.'+k)
    elif isinstance(expected,list):
        assert len(expected)==len(actual),f'{path}: lengths {len(expected)} != {len(actual)}'
        for i,(a,b) in enumerate(zip(expected,actual)):compare(a,b,f'{path}[{i}]')
    elif isinstance(expected,float):
        tolerance=max(1e-10,4*math.ulp(expected))
        assert isinstance(actual,(int,float)) and math.isclose(expected,actual,rel_tol=0.,abs_tol=tolerance),f'{path}: {expected} != {actual} (absolute tolerance {tolerance})'
    elif type(expected) is bool:
        assert type(actual) is bool and expected==actual,f'{path}: boolean {expected!r} != {actual!r}'
    elif type(expected) is int:
        assert type(actual) is int and expected==actual,f'{path}: integer {expected!r} != {actual!r}'
    else:assert expected==actual,f'{path}: {expected!r} != {actual!r}'

class Plugin:
    def pytest_runtest_setup(self,item):
        global ACTIVE
        ACTIVE=item.nodeid

def test_compare_rejects_epoch_millisecond_error():
    epoch=1791906320.4279172
    try:compare(epoch,epoch+.001,'epoch.wheel_stamp')
    except AssertionError:return
    raise AssertionError('Parity comparator admitted a 1 ms epoch timestamp error')

def test_compare_rejects_numeric_boolean():
    try:compare(True,1,'evidence_valid')
    except AssertionError:return
    raise AssertionError('Parity comparator admitted an integer in a boolean schema field')

def fresh_field_traces():
    """The newest C field behavior uses configuration before enable, through public APIs."""
    global ACTIVE
    from dataclasses import replace
    from aims_calibration.engine import Config
    from aims_calibration.planner import TrialPlan
    from aims_calibration.protection import ProtectionSettings,PulseProtection
    from test_engine import step
    for disabled in (False,True):
        ACTIVE='native_public_C_first_wheel_overshoot_reverse_guard_'+str(disabled)
        cfg=Config(stage='decel_current',steering_source='pp',sweep_protocol='full_range_step',
            max_speed=5.,speeds=(1.,5.),currents=(-5.,),start_speed=5.,end_speed=1.,
            stable_seconds=10.,measured_current_handoff=True,recovery_speed=2.)
        item=TrialPlan('decel_current',5.,1.,-5.,kind='braking',baseline_current=9.)
        p=PulseProtection(lambda v:8.+v,lambda item:2.,
            replace(ProtectionSettings(),reverse_guard_enabled=not disabled))
        e=Engine(cfg,trial_plan=[item,item],protection=p);e.enable(True)
        for t,v in ((1.,0.),(1.02,4.7),(1.04,4.99),(1.06,5.03)):
            req=step(e,t,v=v,straight_remaining_m=30.)
        assert e.state=='APPLY' and req.current==-5.
        assert e.start_trigger_observation['velocity']==5.03
        step(e,1.08,v=5.04,current=-5.,straight_remaining_m=30.)
        step(e,1.10,v=1.,current=-5.,straight_remaining_m=30.)
        req=step(e,1.12,v=-.1,current=-5.,straight_remaining_m=30.)
        if disabled:
            assert e.state=='UNLOAD' and e.enabled and req.current==9.
            step(e,1.14,v=-.1,current=9.,straight_remaining_m=30.)
            step(e,1.16,v=-.1,current=9.,straight_remaining_m=30.)
            assert e.state=='HOLD'
        else:assert e.state=='FAULT' and req.current==0.
        assert e.trials[-1]['wheel_speed_min']==-.1
    ACTIVE='native_public_steady_gap_independent_of_protection_gap'
    cfg=replace(cfg,stable_seconds=.1,steady_min_samples=10)
    p=PulseProtection(lambda v:8.+v,lambda item:2.,
        replace(ProtectionSettings(),source_gap=.04))
    e=Engine(cfg,trial_plan=[item,item],protection=p);e.enable(True)
    for t in (1.,1.02,1.04):step(e,t,v=5.,straight_remaining_m=30.)
    assert e.state=='APPLY'
    for i in range(1,65):
        step(e,1.04+i*.06,v=3.,current=-5.,straight_remaining_m=30.)
        if e.trials:break
    assert e.trials[-1]['stop_reason']=='steady_state'
    ACTIVE='native_public_epoch_source_clock_steady_and_iq_handoff'
    cfg=Config(stage='accel_interval',steering_source='pp',sweep_protocol='full_range_step',
        max_speed=5.,speeds=(1.,5.),currents=(20.,),start_speed=1.,end_speed=5.,
        stable_seconds=.1,measured_current_handoff=True,recovery_speed=2.)
    item=TrialPlan('accel_interval',1.,5.,20.,kind='acceleration',baseline_current=9.)
    e=Engine(cfg,trial_plan=[item,item],protection=PulseProtection(
        lambda v:8.+v,lambda item:2.,ProtectionSettings()))
    epoch=1791906320.4279172
    e.enable(True,epoch-.02)
    for i in range(170):
        now=epoch+i*.02
        step(e,now,v=1.,current=20.,straight_remaining_m=30.)
        if e.trials:break
    assert e.state=='UNLOAD' and e.trials[-1]['stop_reason']=='steady_state'
    assert e.trials[-1]['success'] and e.trials[-1]['steady_evidence']['steady']
    assert e.trials[-1]['start_stamp']>epoch
    for i in range(1,30):
        t=now+i*.02
        step(e,t,v=1.,current=9.,straight_remaining_m=30.)
        if e.handoff is None:break
    assert e.trials[-1]['handoff']['hold_confirmed'] and e.index==1
    assert e.trials[-1]['handoff']['actual_iq_confirm_stamp']>e.trials[-1]['handoff']['ack_stamp']
    # Completing a handoff resets the per-trial source watermark. Establish
    # the next trial's watermark, then send a fresh but backwards wheel stamp.
    step(e,t+.02,v=1.,current=9.,straight_remaining_m=30.)
    req=step(e,t+.04,v=1.,current=9.,wheel_stamp=t,straight_remaining_m=30.)
    assert e.state=='FAULT' and e.reason=='source_clock_backwards' and req.current==0.

def main():
    import pytest
    test_compare_rejects_epoch_millisecond_error()
    binary=Path(os.environ.get('NATIVE_ENGINE_TRACE','/tmp/native_engine_trace'))
    assert binary.is_file(),f'Native runtime trace executable missing: {binary}'
    install()
    status=pytest.main([str(ROOT/'tests'/name) for name in ('test_engine.py','test_full_step_engine.py','test_predictive_exit.py','test_c_speed_groups.py','test_steady.py')]+['-q'],plugins=[Plugin()])
    assert status==0
    fresh_field_traces()
    traces=[t for t in TRACES if t['events'] and not t['mutated']]
    case_filter=os.environ.get('NATIVE_ENGINE_CASE_FILTER')
    if case_filter:traces=[t for t in traces if case_filter in t['name']]
    else:assert len(traces)>30,'Insufficient public API safety traces'
    assert traces,'No matching public API safety traces'
    encoded=''.join(json.dumps(dict(prepared=t['prepared'],events=t['events']),allow_nan=False)+'\n' for t in traces)
    run=subprocess.run([str(binary)],input=encoded,text=True,capture_output=True,check=True)
    results=run.stdout.splitlines()
    assert len(results)==len(traces),run.stderr
    for trace,line in zip(traces,results):
        actual=json.loads(line)
        assert 'error' not in actual,(trace['name'],actual)
        assert len(actual['output'])==len(trace['expected']),(trace['name'],'Missing native event outputs')
        for i,(expected,got) in enumerate(zip(trace['expected'],actual['output'])):
            compare(expected,got,f"{trace['name']} event {i} {trace['events'][i]['op']}")
    print(f"Native/Python parity PASS: {len(traces)} traces, {sum(len(t['events']) for t in traces)} public API events; {len(TRACES)-len(traces)} internal-mutation or empty cases excluded")
    from aims_calibration.engine import Config,TrackingConfig
    benchmark=dict(prepared=dict(config=clean(Config(steering_source='rc',speeds=(1.,),
        stable_seconds=.001,measurement_seconds=1e6,hold_sampling='continuous')),
        tracking=clean(TrackingConfig()),plan=[],protection=None,baseline=None,response_limits=None),
        benchmark_status=20000)
    result=json.loads(subprocess.run([str(binary)],input=json.dumps(benchmark)+'\n',
        text=True,capture_output=True,check=True).stdout)
    assert 'error' not in result,result
    measured=result['benchmark']
    assert measured['large_enabled'] and measured['large_samples']>19000,measured
    print('Native bounded status benchmark: '+json.dumps(measured,sort_keys=True))
    if os.environ.get('NATIVE_ENGINE_EVIDENCE'):
        Path(os.environ['NATIVE_ENGINE_EVIDENCE']).write_text(encoded)

if __name__=='__main__':main()
