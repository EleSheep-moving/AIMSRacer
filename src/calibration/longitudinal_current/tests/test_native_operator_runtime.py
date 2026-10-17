"""Operator transport tests use a synthetic service, never vehicle drivers."""
import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import SetBool


@pytest.fixture(scope="module")
def peer():
    rclpy.init()
    node = rclpy.create_node("native_operator_test_peer")
    yield node
    node.destroy_node()
    rclpy.shutdown()


def executable(name):
    root = Path(os.environ.get("CALIB_NATIVE_OPERATOR_DIR", "/tmp/calib-native-install/lib/aims_calibration"))
    path = root / name
    assert path.is_file(), f"Native operator is missing: {path}"
    assert path.read_bytes()[:4] == b"\x7fELF", f"Operator still uses an interpreter: {path}"
    return str(path)


def drive(peer, child, timeout, action=lambda: None):
    deadline = time.monotonic() + timeout
    while child.poll() is None and time.monotonic() < deadline:
        action()
        rclpy.spin_once(peer, timeout_sec=.015)
    if child.poll() is None:
        child.kill()
        child.wait()
        pytest.fail("Native operator did not exit within its transport timeout")
    return child.communicate()[0]


def ready_snapshot():
    return dict(state="IDLE", enabled=False, enable_count=0, logging_error="", logging_dropped=0,
                freshness=.2, ages={name + suffix: 0. for name in ("wheel", "telemetry", "final", "rc", "authority")
                                    for suffix in ("_source", "_receipt")},
                wheel_speed=0., telemetry_valid=True, current_signal="avg_iq", authority=True,
                rc_valid=True, final_zero=True, publishers=dict(calibration=1, final=1, drive=0),
                driver_startup_disabled=True, driver_parameter_age=0., artifact_error="", pp=False,
                require_acceleration=False)


def test_inspect_missing_status_times_out_without_enable(peer):
    child = subprocess.Popen([executable("inspect"), "--timeout", ".2"], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True)
    output = drive(peer, child, 4.)
    assert child.returncode == 2, output
    assert "status" in output.lower(), output


@pytest.mark.parametrize("termination", ["status_loss", "sigint"])
def test_start_disables_once_after_monitor_loss_or_interrupt(peer, termination):
    import json
    requests = []

    def enable(request, response):
        requests.append(request.data)
        response.success = True
        response.message = "synthetic operator service"
        return response

    service = peer.create_service(SetBool, "/calib/enable", enable)
    publisher = peer.create_publisher(DiagnosticArray, "/calib/status", 10)
    message = DiagnosticArray()
    status = DiagnosticStatus(name="longitudinal_calibration")
    values = dict(stage="accel_interval", state="IDLE", enable_count="0",
                  preflight_json=json.dumps(ready_snapshot()))
    status.values = [KeyValue(key=k, value=v) for k, v in values.items()]
    message.status = [status]
    child = subprocess.Popen([executable("start"), "--timeout", "2", "--max-seconds", "8"],
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    interrupted = False

    def publish_or_interrupt():
        nonlocal interrupted
        if not requests:
            publisher.publish(message)
        elif termination == "sigint" and not interrupted:
            child.send_signal(signal.SIGINT)
            interrupted = True

    try:
        output = drive(peer, child, 10., publish_or_interrupt)
        assert requests == [True, False], output
        assert child.returncode == 3, output
    finally:
        peer.destroy_service(service)
        peer.destroy_publisher(publisher)
