"""Exercise real ROS diagnostic types against the MPCC callback and supervisor."""
import sys
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from aims_mpcc.node import MPCCNode
from aims_mpcc.localization import LocalizationHealth
from aims_mpcc.runtime import Supervisor, Command
from aims_mpcc.config import VehicleConfig


def packet(epoch,anchor,health,stamp,ready=True):
    values=dict(protocol_version='1',epoch=epoch,anchor_sequence=str(anchor),health_sequence=str(health),
                last_anchor_stamp_ns=str(stamp),ready=str(ready).lower(),state='tracking' if ready else 'lost')
    return DiagnosticArray(status=[DiagnosticStatus(name='aims_racer_system/localization',
        values=[KeyValue(key=k,value=v) for k,v in values.items()])])


def test_regular_updates_preserve_plans_epoch_reset_revokes_and_recovery_does_not_restart(monkeypatch):
    import aims_mpcc.node as adapter
    now=[10.];ros=[1000000000]
    monkeypatch.setattr(adapter.time,'monotonic',lambda:now[0])
    supervisor=Supervisor(VehicleConfig(),10.)
    supervisor.status='RUNNING';supervisor.plan={'sentinel':1};supervisor.pending_plan={'sentinel':2}
    supervisor.last_command=Command(1.,.1)
    node=SimpleNamespace(supervisor=supervisor,localization_health=LocalizationHealth(),map_alignment=(0.,0.,0.),
                         get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=ros[0])))
    MPCCNode.localization_status(node,packet('one',1,1,ros[0]))
    generation=supervisor.generation
    ros[0]+=100000000;now[0]+=.1
    MPCCNode.localization_status(node,packet('one',2,2,ros[0]))
    assert supervisor.active and supervisor.plan=={'sentinel':1}
    assert supervisor.generation==generation
    MPCCNode.localization_status(node,packet('two',0,1,0,False))
    assert supervisor.status=='FAULT' and supervisor.last_command.speed==0
    assert supervisor.plan is None and supervisor.pending_plan is None
    assert supervisor.generation==generation+1 and node.map_alignment is None
    assert not supervisor.accept({'generation':generation},now[0])
    ros[0]+=100000000;now[0]+=.1
    MPCCNode.localization_status(node,packet('two',1,2,ros[0]))
    assert node.localization_health.usable(ros[0],now[0])
    assert supervisor.status=='FAULT' and not supervisor.active


def test_lost_ready_packet_cancels_pending_result_and_zeroes_speed(monkeypatch):
    import aims_mpcc.node as adapter
    monkeypatch.setattr(adapter.time,'monotonic',lambda:10.)
    supervisor=Supervisor(VehicleConfig(),10.)
    supervisor.status='RUNNING';supervisor.pending_plan={'sentinel':1};supervisor.last_command=Command(1.,.2)
    node=SimpleNamespace(supervisor=supervisor,localization_health=LocalizationHealth(),map_alignment=(0.,0.,0.),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)))
    MPCCNode.localization_status(node,packet('one',1,1,1000000000))
    MPCCNode.localization_status(node,packet('one',1,2,1000000000,False))
    assert not supervisor.active and supervisor.pending_plan is None
    assert supervisor.last_command==Command(0.,.2)
