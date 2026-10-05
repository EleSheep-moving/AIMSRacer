"""Executable contract for authoritative anchor health, independent of ROS."""
import sys
from pathlib import Path
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from localization_policy import AnchorHealth, initialization_complete


def event(seq, stamp=10_000_000_000, committed=True, ready=True, epoch='process:1'):
    return dict(protocol_version='1', epoch=epoch, anchor_sequence=str(seq), event_sequence=str(seq),
                anchor_committed=str(committed).lower(), ready=str(ready).lower(),
                reason='ok' if committed else 'rejected', last_anchor_stamp_ns=str(stamp))


def accept(h, seq, now=10., mono=1., **kw):
    return h.observe(event(seq, int(now * 1e9), **kw), int(now * 1e9), int(now * 1e9), mono)


def test_first_committed_ready_is_tracking_and_rejection_holds():
    h = AnchorHealth()
    accept(h, 1)
    assert h.evaluate(10., 1., True)['ready']
    accept(h, 2, now=10.1, mono=1.1, committed=False)
    status = h.evaluate(10.2, 1.2, True)
    assert status['state'] == 'hold' and status['ready']
    assert h.evaluate(10.51, 1.51, True)['state'] == 'lost'


def test_timeout_requires_three_real_commits_to_recover():
    h = AnchorHealth()
    accept(h, 1)
    assert not h.evaluate(10.6, 1.6, True)['ready']
    for seq in (2, 3):
        accept(h, seq, now=10.6 + seq * .1, mono=1.6 + seq * .1)
        assert not h.evaluate(10.6 + seq * .1, 1.6 + seq * .1, True)['ready']
    accept(h, 4, now=11., mono=2.)
    assert h.evaluate(11., 2., True)['ready']


def test_duplicate_out_of_order_and_retired_epochs_never_refresh():
    h = AnchorHealth()
    accept(h, 2)
    assert not accept(h, 2, now=10.2, mono=1.2)
    assert not accept(h, 1, now=10.3, mono=1.3)
    assert h.last_anchor_stamp_ns == 10_000_000_000
    accept(h, 0, now=10.3, mono=1.3, epoch='process:2', committed=False, ready=False)
    assert not h.evaluate(10.3, 1.3, True)['ready']
    assert not accept(h, 3, now=10.4, mono=1.4)
    assert h.epoch == 'process:2'


def test_monotonic_watchdog_future_time_rewind_and_inputs_fail_closed():
    h = AnchorHealth()
    accept(h, 1)
    assert not h.evaluate(10., 1.6, True)['ready']
    accept(h, 2, now=10.1, mono=1.7)
    assert not h.observe(event(3, 11_000_000_000), 11_000_000_000, 10_100_000_000, 1.8)
    assert not h.evaluate(10.1, 1.8, True)['ready']
    h = AnchorHealth()
    accept(h, 1)
    assert not h.evaluate(9., 1.1, True)['ready']
    assert not h.evaluate(10., 1.2, True)['ready']
    h = AnchorHealth()
    accept(h, 1)
    assert not h.evaluate(10., 1., False)['ready']


@pytest.mark.parametrize('field,value', [('protocol_version','2'), ('anchor_sequence','-1'),
    ('anchor_sequence','1.1'), ('ready','yes'), ('anchor_committed','yes'), ('epoch','')])
def test_malformed_protocol_fails_closed(field, value):
    h = AnchorHealth()
    accept(h, 1)
    data = event(2); data[field] = value
    assert not h.observe(data, 10_000_000_000, 10_000_000_000, 1.1)
    assert not h.evaluate(10., 1.1, True)['ready']


def test_initializer_requires_new_epoch_ready_and_fresh_final_status():
    status = dict(protocol_version='1', health_sequence='1', epoch='old', ready='true', state='tracking', source_age='0.1', receive_age='0.1',
                  last_anchor_stamp_ns='10000000000')
    assert not initialization_complete(status, 'old', 10.1, 1.1, 1.)
    status['epoch'] = 'new'
    assert initialization_complete(status, 'old', 10.1, 1.1, 1.)
    assert not initialization_complete(status, 'old', 10.6, 1.1, 1.)
    assert not initialization_complete(status, 'old', 10.1, 1.6, 1.)
    status['state'] = 'hold'
    assert not initialization_complete(status, 'old', 10.1, 1.1, 1.)


def test_initial_reset_then_first_ready_commit_uses_native_confirmations():
    h = AnchorHealth()
    h.evaluate(10., 1., False)
    accept(h, 0, committed=False, ready=False)
    accept(h, 1, now=10.1, mono=1.1)
    assert h.evaluate(10.1, 1.1, True)['ready']

def test_receive_gap_requires_recovery_even_without_heartbeat():
    h = AnchorHealth()
    accept(h, 1)
    accept(h, 2, now=10.6, mono=1.6)
    assert not h.evaluate(10.6, 1.6, True)['ready']


def test_initial_pose_is_explicit_map_base_link_and_finite():
    from localization_policy import initial_pose_quaternion
    assert initial_pose_quaternion('base_link', [0., 0., 0.], [0., 0., 0.]) == (0., 0., 0., 1.)
    with pytest.raises(ValueError):
        initial_pose_quaternion('livox_frame', [0., 0., 0.], [0., 0., 0.])
    with pytest.raises(ValueError):
        initial_pose_quaternion('base_link', [float('nan'), 0., 0.], [0., 0., 0.])


def test_rejection_event_keeps_anchor_sequence_and_real_source_stamp():
    h = AnchorHealth()
    accept(h, 1)
    rejected = event(2, committed=False)
    rejected['anchor_sequence'] = '1'
    assert h.observe(rejected, 10_100_000_000, 10_100_000_000, 1.1)
    status = h.evaluate(10.1, 1.1, True)
    assert status['state'] == 'hold' and status['anchor_sequence'] == 1
    assert status['last_anchor_stamp_ns'] == 10_000_000_000
    assert status['source_age'] == pytest.approx(.1)


def test_clock_rewind_requires_new_epoch():
    h = AnchorHealth()
    accept(h, 1)
    h.evaluate(10., 1., True)
    assert not h.evaluate(9., 1.1, True)['ready']
    assert not accept(h, 2, now=9.1, mono=1.2)
    assert not h.evaluate(9.1, 1.2, True)['ready']
    accept(h, 1, now=9.2, mono=1.3, epoch='process:2')
    assert h.evaluate(9.2, 1.3, True)['ready']


def test_diagnostic_correction_is_discrete_and_pose_is_interpolated():
    import numpy as np
    from localization_policy import MapOdomPackets, interpolate_pose
    first, second = np.eye(4), np.eye(4)
    second[0, 3] = 2.
    pose = interpolate_pose([(1., first), (3., second)], 2.)
    assert pose[0, 3] == 1.
    assert interpolate_pose([(1., first), (3., second)], 3.1) is None
    packets = MapOdomPackets()
    packets.add(100, first); packets.add(200, second)
    assert packets.held(150)[0, 3] == 0.
    assert packets.held(200)[0, 3] == 2.
    assert packets.held(99) is None


def test_integer_nanosecond_freshness_avoids_epoch_float_rounding():
    h = AnchorHealth()
    now_ns = 1_790_000_000_000_000_123
    assert h.observe(event(1, now_ns), now_ns, now_ns, 1.)
    assert h.evaluate(now_ns * 1e-9, 1., True, source_now_ns=now_ns)['ready']


def test_initializer_rejects_missing_health_sequence_or_expired_anchor_receive_age():
    status = dict(protocol_version='1', epoch='new', ready='true', state='tracking',
                  source_age='0.1', receive_age='0.1', last_anchor_stamp_ns='10000000000')
    assert not initialization_complete(status, 'old', 10.1, 1.1, 1.)
    status.update(health_sequence='1', receive_age='0.6')
    assert not initialization_complete(status, 'old', 10.1, 1.1, 1.)


def test_deployed_anchor_hold_accepts_750ms_and_expires_after_one_second():
    import yaml
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
        'params/localization_monitor.yaml').read_text())['localization_monitor']['ros__parameters']
    h = AnchorHealth(max_age=config['anchor_max_age_sec'])
    accept(h, 1)
    reject = event(2, committed=False)
    reject['anchor_sequence'] = '1'
    assert h.observe(reject, 10_750_000_000, 10_750_000_000, 1.75)
    held = h.evaluate(10.75, 1.75, True, source_now_ns=10_750_000_000)
    assert held['state'] == 'hold' and held['ready']
    assert h.evaluate(11., 2., True, source_now_ns=11_000_000_000)['ready']
    assert not h.evaluate(11.000000001, 2.000000001, True,
        source_now_ns=11_000_000_001)['ready']


def test_deployed_anchor_receive_watchdog_expires_with_frozen_source_clock():
    import yaml
    config = yaml.safe_load((Path(__file__).resolve().parents[1] /
        'params/localization_monitor.yaml').read_text())['localization_monitor']['ros__parameters']
    h = AnchorHealth(max_age=config['anchor_max_age_sec'])
    accept(h, 1)
    assert h.evaluate(10., 1.75, True)['ready']
    assert not h.evaluate(10., 2.000000001, True)['ready']
