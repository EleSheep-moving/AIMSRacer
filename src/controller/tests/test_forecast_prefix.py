"""The future bridge starts at this tick's issued effective motor target."""
import copy
from collections import deque
from types import SimpleNamespace

import pytest

from aims_mpcc.config import VehicleConfig
from aims_mpcc.history import AppliedHistory
from aims_mpcc.node import MPCCNode
from aims_mpcc.runtime import Command, State, Supervisor


class ObservedHistory(AppliedHistory):
    def __init__(self, tau):
        super().__init__(tau)
        self.steps = []
        self.states = []

    def _step(self, state, command, dt, config):
        self.steps.append((state.timestamp, dict(command)))
        self.states.append(state)
        return super()._step(state, command, dt, config)


def forecast_node(*, minimum_drive_speed=0., applied_speed=.35):
    config = VehicleConfig(cruise_speed=1., max_speed=1.5,
                           minimum_drive_speed=minimum_drive_speed,
                           enforce_corridor=False)
    supervisor = Supervisor(config, 100., plan_ttl=.75, handover_delay=.045,
                            solve_period=.05)
    supervisor.observe(State(0., 0., 0., 0., 0., 100.), 9.96, 0., 0.)
    supervisor.set_mode(True, 9.96)
    supervisor.start(9.96)
    supervisor.set_mode(True, 10.)
    supervisor.state = State(0., 0., 0., .3, 0., 100.)
    supervisor.last_tick = 9.98
    supervisor.last_command = Command(applied_speed, .02)
    supervisor.last_acceleration = .05
    supervisor.last_steering_rate = .1
    supervisor.plan = dict(
        source_stamp=9.96, stamp=9.96, previous_steering=.02,
        states=[[i * .06, 0., 0., .6, i * .06, 0.] for i in range(11)],
        controls=[[.3, .1, .6] for _ in range(10)],
        execution_speed_targets=[.6] * 11)
    history = ObservedHistory(config.steering_tau)
    history.record(9.94, .01, applied_speed / 2., .01, .05)
    history.record(9.98, .02, applied_speed, .05, .1)
    node = SimpleNamespace(
        supervisor=supervisor, config=config, history=history,
        path=SimpleNamespace(frame_id='odom', length=100.,
                             project=lambda xy: (xy[0], 0.)),
        handover_delay=.045, solve_period=.05, horizon=10, last_solve=9.95,
        recent_proposals=deque(maxlen=16))
    node.applied_ready = lambda now: MPCCNode.applied_ready(node, now)
    return node


def issue_current_proposal(node):
    s = node.supervisor
    command = s.command(10.)
    assert s.status == 'RUNNING'
    actuator = s.actuator_command(command)
    effective = dict(speed=actuator.speed, steering=actuator.steering,
                     acceleration=s.last_acceleration if actuator.speed == command.speed else 0.,
                     steering_rate=s.last_steering_rate)
    node.recent_proposals.append((10., actuator, effective['acceleration'],
                                  effective['steering_rate']))
    return effective


def future_steps(node):
    # The measured state epoch 100. corresponds to monotonic receipt 9.96.
    return [command for stamp, command in node.history.steps if stamp >= 100.04 - 1e-9]


def test_current_proposal_seeds_future_while_past_replays_real_selector_history():
    node = forecast_node()
    issued = issue_current_proposal(node)
    supervisor_snapshot = copy.deepcopy(node.supervisor.__dict__)
    records = list(node.history.records)
    # Independent historical replay ends before any proposal is forecast.
    actual_now, _ = node.history.predict(node.supervisor.state, 9.96, 10., node.config)
    node.history.steps.clear()
    node.history.states.clear()

    MPCCNode.prepare_request(node, 10.)

    past = [command for stamp, command in node.history.steps if stamp < 100.04 - 1e-9]
    assert past[0] == dict(speed=.175, steering=.01, acceleration=.01, steering_rate=.05)
    assert past[-1] == dict(speed=.35, steering=.02, acceleration=.05, steering_rate=.1)
    assert future_steps(node)[0] == pytest.approx(issued)
    future_states = [state for state in node.history.states if state.timestamp >= 100.04 - 1e-9]
    assert future_states[0] == actual_now
    assert future_steps(node)[-1]['speed'] > issued['speed']
    assert future_steps(node)[-1]['steering'] > issued['steering']
    assert actual_now.speed != issued['speed']
    assert node.supervisor.__dict__ == supervisor_snapshot
    assert list(node.history.records) == records


def test_minimum_drive_clamp_preserves_internal_smoothing_for_three_output_ticks():
    node = forecast_node(minimum_drive_speed=.2, applied_speed=.04)
    issued = issue_current_proposal(node)
    assert issued['speed'] == .2
    assert node.supervisor.last_command.speed < .2
    assert node.supervisor.last_acceleration > 0.
    assert issued['acceleration'] == 0.
    records = list(node.history.records)
    supervisor_snapshot = copy.deepcopy(node.supervisor.__dict__)
    real_output = copy.deepcopy(node.supervisor)
    expected = []
    continuous_speeds = []
    for tick in range(3):
        command = real_output.command(10. + tick * .02)
        actuator = real_output.actuator_command(command)
        continuous_speeds.append(command.speed)
        expected.append(dict(speed=actuator.speed, steering=actuator.steering,
                             acceleration=real_output.last_acceleration if actuator.speed == command.speed else 0.,
                             steering_rate=real_output.last_steering_rate))
    assert continuous_speeds == pytest.approx([.0414, .0432, .0454])

    MPCCNode.prepare_request(node, 10.)

    for tick, effective in enumerate(expected):
        forecast_output = next(command for stamp, command in node.history.steps
                               if stamp >= 100.04 + tick * .02 - 1e-9)
        assert forecast_output == pytest.approx(effective)
    assert node.supervisor.__dict__ == supervisor_snapshot
    assert list(node.history.records) == records


@pytest.mark.parametrize('proposal_stamp', [None, 9.98])
def test_no_current_issued_proposal_falls_back_to_actual_history(proposal_stamp):
    node = forecast_node()
    issue_current_proposal(node)
    if proposal_stamp is None:
        del node.recent_proposals
    else:
        entry = node.recent_proposals.pop()
        node.recent_proposals.append((proposal_stamp, *entry[1:]))

    MPCCNode.prepare_request(node, 10.)

    assert future_steps(node)[0] == pytest.approx(node.history.command_at(10.))


def test_manual_selector_mode_holds_actual_target_despite_issued_proposal():
    node = forecast_node()
    issue_current_proposal(node)
    node.supervisor.set_mode(False, 10.)

    request = MPCCNode.prepare_request(node, 10.)

    actual = dict(node.history.command_at(10.), acceleration=0., steering_rate=0.)
    assert all(command == actual for command in future_steps(node))
    assert request['handover_command'] == actual
