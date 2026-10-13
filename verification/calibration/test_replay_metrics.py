"""A scheduling measurement must count output callbacks, not sensor callbacks."""
from .replay_runtime import command_metrics


def test_pp_sensor_callbacks_do_not_dilute_command_latency():
    rows=[dict(event='odometry',phase='RUNNING',callback_s='0.0001'),
          dict(event='command',phase='RUNNING',callback_s='0.025',publish_gap_s='0.03'),
          dict(event='command',phase='FAULT',callback_s='0.0001',publish_gap_s='0.02')]
    count,processing,interval=command_metrics(rows,'pp')
    assert count==1 and processing['p95']==25. and interval['p95']==30.
    assert processing['over_20_ms']==1


def test_calibration_idle_loop_does_not_count_as_active_stage():
    rows=[dict(state='IDLE',callback_ms='0.1',timer_interval_ms='20'),
          dict(state='MEASURE',callback_ms='2',timer_interval_ms='21')]
    count,processing,interval=command_metrics(rows,'calibration')
    assert count==1 and processing['p95']==2. and interval['p95']==21.
