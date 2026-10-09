import importlib.util
from pathlib import Path


def module():
    file=Path(__file__).with_name('worker_stall_ros_probe.py')
    spec=importlib.util.spec_from_file_location('worker_stall_ros_probe',file)
    answer=importlib.util.module_from_spec(spec);spec.loader.exec_module(answer)
    return answer


def trace():
    commands=[dict(time=1.+i*.02,speed=.2 if i<70 else 0.) for i in range(151)]
    events=[]
    for i in range(31):
        t=1.+i*.1
        values=dict(status='RUNNING' if t<2. else 'RECOVERING' if t<3. else 'READY',
            enabled=int(t<3.),worker_busy=int(t>=1.2),test_worker_blocked=int(t>=1.2),
            reason='Plan expired' if 2.<=t<3. else '',plan_ttl=.8,plan_source_age_s=1. if t>=2. else .2,
            source_age_s=.02,receipt_age_s=.01,authority_age_s=.01,applied_age_s=.01)
        events.append(dict(time=t,status=values))
    return commands,events


def test_worker_stall_requires_positive_then_responsive_recovery_and_disabled_zero():
    commands,events=trace()
    evidence=module().assess(commands,events)
    assert all(evidence['checks'].values())
    assert evidence['blocked_command_samples']>=20
    assert not module().assess([{**r,'speed':0.} for r in commands],events)['checks']['positive_before_worker_block']
    changed=[dict(time=e['time'],status={**e['status'],'status':'RUNNING','enabled':1}) for e in events]
    assert not module().assess(commands,changed)['checks']['ttl_recovery_disabled_ready']
    young=[dict(time=e['time'],status={**e['status'],'plan_source_age_s':.3}) for e in events]
    assert not module().assess(commands,young)['checks']['original_source_ttl_exhausted']


def test_worker_stall_does_not_pass_when_the_whole_ros_process_stops_responding():
    commands,events=trace()
    evidence=module().assess(commands[:15],events)
    assert not evidence['checks']['output_remains_responsive_while_blocked']
