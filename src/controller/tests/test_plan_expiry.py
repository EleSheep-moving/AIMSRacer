"""Keep forecast time separate from the real plan's source-age deadline.

The explicit 0.75 s override reproduces the historical October 4 event;
it is not the current horizon-derived default (0.8 s with horizon 10).
"""
import copy
from dataclasses import replace
from types import SimpleNamespace

import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.history import AppliedHistory
from aims_mpcc.node import MPCCNode
from aims_mpcc.runtime import Command, State, Supervisor


def running_supervisor():
    config = VehicleConfig(cruise_speed=1., max_speed=1.5,
                           minimum_drive_speed=.2, steer_rate=2.)
    supervisor = Supervisor(config, 100., plan_ttl=.75, handover_delay=.2)
    supervisor.observe(State(0., 0., 0., 0., 0., 100.), 10., 0., 0.)
    supervisor.set_mode(True, 10.)
    supervisor.start(10.)
    supervisor.state = replace(supervisor.state, speed=.96)
    supervisor.last_command = Command(.85, 0.)
    supervisor.plan = dict(
        source_stamp=9.38, stamp=9.60,
        states=[[i * .1, 0., 0., 1., i * .1, 0.] for i in range(11)],
        controls=[[0., 0., 0.] for _ in range(10)])
    return supervisor


def test_prepare_request_can_forecast_past_ttl_without_mutating_real_execution():
    supervisor = running_supervisor()
    history = AppliedHistory(supervisor.config.steering_tau)
    history.record(9.98, 0., .85, 0., 0.)
    history.record(10., 0., .85, 0., 0.)
    node = SimpleNamespace(
        supervisor=supervisor, config=supervisor.config, history=history,
        path=SimpleNamespace(frame_id='odom', length=100.,
                             project=lambda xy: (xy[0], 0.)),
        handover_delay=.2, solve_period=.2, horizon=10, last_solve=9.8)
    node.applied_ready = lambda now: MPCCNode.applied_ready(node, now)
    snapshot = copy.deepcopy(supervisor.__dict__)
    records = list(history.records)

    request = MPCCNode.prepare_request(node, 10.)

    assert request['stamp'] == pytest.approx(10.2)
    assert request['source_stamp'] == 10.
    assert request['stamp'] - supervisor.plan['source_stamp'] > supervisor.PLAN_TTL
    assert request['state']['x'] > 0.
    assert supervisor.__dict__ == snapshot
    assert list(history.records) == records
    supervisor.command(10.)
    assert supervisor.status == 'RUNNING'


def test_forecast_retains_nonzero_controls_after_source_age_deadline():
    supervisor = running_supervisor()
    forecast = copy.copy(supervisor)
    for index in range(1, 11):
        now = 10. + index * .02
        forecast.state_received = forecast.mode_received = now
        command = forecast.command(now, enforce_plan_age=False)
        assert forecast.status == 'RUNNING'
        assert command.speed > 0.
    assert supervisor.plan['source_stamp'] == 9.38
    assert supervisor.PLAN_TTL == .75
    assert supervisor.last_tick == 10.


def test_real_output_stops_at_original_deadline_after_forecasting():
    supervisor = running_supervisor()
    forecast = copy.copy(supervisor)
    for index in range(1, 11):
        now = 10. + index * .02
        forecast.state_received = forecast.mode_received = now
        forecast.command(now, enforce_plan_age=False)
    for index in range(1, 8):
        now = 10. + index * .02
        supervisor.state_received = supervisor.mode_received = now
        command = supervisor.command(now)
        if index < 7:
            assert supervisor.status == 'RUNNING'
            assert command.speed > 0.
    assert supervisor.status == 'FAULT'
    assert supervisor.reason == 'Plan expired'
    assert command.speed == 0.


def test_rejected_handover_does_not_refresh_old_plan_deadline():
    supervisor = running_supervisor()
    old = supervisor.plan
    candidate = dict(old, generation=supervisor.generation, success=True,
                     constraint_violation=0., source_stamp=9.8,
                     submitted_at=9.8, stamp=10.)
    assert supervisor.accept(candidate, 10.)
    # Current handover tolerance is 30 cm; exceeding it preserves the old plan.
    actual = State(.301, 0., 0., 1., 0., 100.)
    assert not supervisor.activate(10., actual)
    assert supervisor.plan is old
    assert supervisor.plan['source_stamp'] == 9.38
    assert supervisor.rejected_plans == 1
    for index in range(1, 8):
        now = 10. + index * .02
        supervisor.state_received = supervisor.mode_received = now
        command = supervisor.command(now)
    assert supervisor.reason == 'Plan expired'
    assert command.speed == 0.


@pytest.mark.parametrize('source_stamp, stamp, reason', [
    (9.38, 9., 'Prediction horizon exhausted'),
    (10.01, 9.60, 'Plan expired'),
    (9.38, 10.01, 'Plan activated before handover'),
])
def test_forecast_preserves_other_plan_guards(source_stamp, stamp, reason):
    supervisor = running_supervisor()
    supervisor.plan.update(source_stamp=source_stamp, stamp=stamp)
    command = supervisor.command(10., enforce_plan_age=False)
    assert supervisor.status == 'FAULT'
    assert supervisor.reason == reason
    assert command.speed == 0.
