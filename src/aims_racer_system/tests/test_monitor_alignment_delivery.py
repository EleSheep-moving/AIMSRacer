"""No-drive ROS integration of the deployed C++ monitor's atomic status record."""
import os
import subprocess
import time

import pytest


def test_cpp_monitor_publishes_accepted_alignment_and_clears_epoch_and_rewind(tmp_path):
    rclpy = pytest.importorskip('rclpy')
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import QoSProfile, DurabilityPolicy
    executable = os.environ.get('MONITOR_TEST_EXECUTABLE')
    if not executable:
        pytest.skip('MONITOR_TEST_EXECUTABLE must identify the built C++ monitor')
    points = '\n'.join(f'{i * .01} 0 0' for i in range(100))
    map_file = tmp_path / 'map.pcd'
    map_file.write_text('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
                        'WIDTH 100\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 100\nDATA ascii\n' + points)
    process = subprocess.Popen([executable, '--ros-args', '-p', f'map_file:={map_file}',
                                '-p', 'use_sim_time:=true', '-p', 'ekf_max_age_sec:=3.0',
                                '-p', 'cloud_max_age_sec:=3.0', '-p', 'anchor_max_age_sec:=3.0'], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('monitor_alignment_test')
    delivered = []
    status_stamps = []

    def receive(message):
        delivered.append({v.key: v.value for v in message.status[0].values})
        status_stamps.append(message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec)

    node.create_subscription(DiagnosticArray, '/localization/status', receive, 10)
    anchor_pub = node.create_publisher(DiagnosticArray, '/localization/anchor_status',
        QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    odom_pub = node.create_publisher(Odometry, '/odometry/filtered', 10)
    cloud_pub = node.create_publisher(PointCloud2, '/fastlio2/body_cloud', 10)
    clock_pub = node.create_publisher(Clock, '/clock', 10)

    def spin_until(predicate, publish=None):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            if publish is not None:
                publish()
            rclpy.spin_once(node, timeout_sec=.02)
            if predicate():
                return
            if process.poll() is not None:
                pytest.fail('monitor exited: ' + process.stdout.read())
        pytest.fail('status not delivered; latest=' + str(delivered[-1:] ))

    def inputs(second):
        clock = Clock(); clock.clock.sec = second
        # Reader discovery alone does not prove that the node processed its
        # first /clock. Wait for its own stamped status before input/anchor data.
        spin_until(lambda: status_stamps and status_stamps[-1] == second * 1_000_000_000,
                   lambda: clock_pub.publish(clock))
        before = int(delivered[-1].get('ekf_callback_count', '0'))
        odom = Odometry(); odom.header.stamp.sec = second
        odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.pose.pose.orientation.w = 1.
        cloud = PointCloud2(); cloud.header.stamp.sec = second; cloud.header.frame_id = 'livox_frame'

        def publish_inputs():
            odom_pub.publish(odom); cloud_pub.publish(cloud)

        # Volatile sensor packets may drop around discovery on the NX. Repeat
        # this stage's qualified inputs until actual monitor callbacks accept
        # both; unchanged source stamps still cannot refresh its watchdog.
        spin_until(lambda: int(delivered[-1].get('ekf_callback_count', '0')) > before
                   and float(delivered[-1].get('ekf_age_sec', 'inf')) == 0.
                   and float(delivered[-1].get('body_cloud_age_sec', 'inf')) == 0., publish_inputs)

    def anchor(epoch, sequence, second, committed=True, **extra):
        fields = dict(protocol_version='1', epoch=epoch, event_sequence=str(sequence),
                      anchor_sequence=str(sequence if committed else 0),
                      last_anchor_stamp_ns=str(second * 1_000_000_000 if committed else 0),
                      anchor_committed=str(committed).lower(), ready=str(committed).lower(), reason='ok',
                      map_odom_x='1.25', map_odom_y='-2', map_odom_z='3',
                      map_odom_qx='0', map_odom_qy='0', map_odom_qz='0', map_odom_qw='1')
        fields.update(extra)
        message = DiagnosticArray(); message.header.stamp.sec = second
        message.status = [DiagnosticStatus(name='lidar_localization/anchor',
            values=[KeyValue(key=k, value=v) for k, v in fields.items()])]
        anchor_pub.publish(message)

    try:
        spin_until(lambda: all(p.get_subscription_count() > 0 for p in
                              (anchor_pub, clock_pub, odom_pub, cloud_pub)))
        inputs(10); anchor('one', 1, 10)
        spin_until(lambda: any(s.get('epoch') == 'one' and s.get('ready') == 'true' for s in delivered))
        accepted = next(s for s in reversed(delivered) if s['epoch'] == 'one' and s['ready'] == 'true')
        assert accepted.get('alignment_valid') == 'true'
        assert accepted['alignment_epoch'] == accepted['epoch']
        assert accepted['alignment_anchor_sequence'] == accepted['anchor_sequence']
        assert accepted['alignment_stamp_ns'] == accepted['last_anchor_stamp_ns']
        assert accepted['map_odom_x'] == '1.25'
        assert len(accepted['map_sha256']) == 64
        anchor('one', 2, 10, committed=False, anchor_sequence='1',
               last_anchor_stamp_ns='10000000000', ready='true')
        spin_until(lambda: delivered[-1].get('state') == 'hold')
        assert delivered[-1]['alignment_valid'] == 'true'
        assert delivered[-1]['alignment_stamp_ns'] == '10000000000'
        assert delivered[-1]['map_odom_x'] == '1.25'
        # Duplicate sequence with a changed payload is never accepted as the same alignment.
        anchor('one', 1, 10, map_odom_x='99')
        spin_until(lambda: delivered[-1].get('ready') == 'false')
        assert delivered[-1]['alignment_valid'] == 'false'
        inputs(11); anchor('two', 0, 11, committed=False)
        spin_until(lambda: delivered[-1].get('epoch') == 'two')
        assert delivered[-1]['alignment_valid'] == 'false'
        assert 'map_odom_x' not in delivered[-1]
        inputs(12); anchor('two', 1, 12)
        spin_until(lambda: delivered[-1].get('epoch') == 'two' and delivered[-1].get('alignment_valid') == 'true')
        inputs(9)
        spin_until(lambda: delivered[-1].get('alignment_valid') == 'false')
        assert delivered[-1]['ready'] == 'false'
        assert 'map_odom_x' not in delivered[-1]
        anchor('bad', 1, 9, map_odom_qw='nan')
        spin_until(lambda: delivered[-1].get('epoch') == 'bad')
        assert delivered[-1]['ready'] == 'false'
        assert delivered[-1]['alignment_valid'] == 'false'
        assert 'map_odom_qw' not in delivered[-1]
        # A new authoritative epoch plus fresh pose/cloud restores readiness;
        # a rewind must not strand health behind the old ROS source watermark.
        inputs(10); anchor('recovered', 1, 10)
        spin_until(lambda: delivered[-1].get('epoch') == 'recovered'
                   and delivered[-1].get('ready') == 'true')
        recovered = delivered[-1]
        assert recovered['alignment_valid'] == 'true'
        assert recovered['alignment_epoch'] == recovered['epoch'] == 'recovered'
        assert recovered['alignment_anchor_sequence'] == recovered['anchor_sequence'] == '1'
        assert recovered['alignment_stamp_ns'] == recovered['last_anchor_stamp_ns'] == '10000000000'
        assert recovered['map_valid'] == 'true'
        assert recovered['state'] == 'tracking'
        assert recovered['map_sha256'] == accepted['map_sha256']
        assert [float(recovered['map_odom_' + key]) for key in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')] == [
            1.25, -2., 3., 0., 0., 0., 1.]
    finally:
        node.destroy_node(); rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
