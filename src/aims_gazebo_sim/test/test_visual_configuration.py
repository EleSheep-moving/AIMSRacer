import ast
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[3]


def test_rviz_odometry_display_accepts_sensor_data_qos():
    config = (REPOSITORY / 'src/aims_gazebo_sim/rviz/mpcc_gazebo.rviz').read_text()
    odometry = config.split('Name: Vehicle odometry', 1)[1].split('- Alpha:', 1)[0]

    assert 'Reliability Policy: Best Effort' in odometry


def test_startup_warnings_are_throttled():
    source_path = REPOSITORY / 'src/ackermann_mux/scripts/joystick_control_v2.py'
    tree = ast.parse(source_path.read_text())
    messages = {
        'Waiting for RC input...',
        'No nav message received yet',
    }
    warnings = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != 'warn' or not node.args:
            continue
        message = node.args[0]
        if isinstance(message, ast.Constant) and message.value in messages:
            warnings.append(node)

    assert len(warnings) == 4
    for warning in warnings:
        assert any(
            keyword.arg == 'throttle_duration_sec'
            and isinstance(keyword.value, ast.Constant)
            and keyword.value.value == 1.0
            for keyword in warning.keywords
        )
