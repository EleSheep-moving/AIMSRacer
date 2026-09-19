"""ROS delivery regression, with stale IMU rejection and real node serialization."""
import importlib.util
from pathlib import Path
import time
import pytest
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


@pytest.mark.parametrize('publish_tf', [False, True])
def test_adapter_ros_delivery_and_stale_gyro_rejection(publish_tf):
    path = Path(__file__).parents[1] / 'scripts/lio_to_rear_axle.py'
    spec = importlib.util.spec_from_file_location('adapter', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    rclpy.init(args=['--ros-args', '-p', 'imu_translation:=[0.3,0.0,0.0]',
                     '-p', f'publish_tf:={str(publish_tf).lower()}'])
    node = module.LioToRearAxle()
    probe = rclpy.create_node('rear_axle_test_probe')
    received = []
    transforms = []
    probe.create_subscription(TFMessage, '/tf', transforms.append, 10)
    probe.create_subscription(Odometry, '/fastlio2/base_odom', received.append, 10)
    imu_pub = probe.create_publisher(Imu, '/livox/imu', 10)
    odom_pub = probe.create_publisher(Odometry, '/fastlio2/lio_odom', 10)

    def spin_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.005)
            rclpy.spin_once(probe, timeout_sec=.005)
    try:
        spin_for(.5)
        imu = Imu(); imu.header.stamp.sec = 10; imu.angular_velocity.z = 2.
        odom = Odometry(); odom.header.stamp.sec = 10
        odom.header.frame_id = 'odom'; odom.child_frame_id = 'livox_imu'
        odom.pose.pose.orientation.w = 1.; odom.pose.pose.position.x = .3
        odom.twist.twist.linear.x = 1.; odom.twist.twist.linear.y = .6
        imu_pub.publish(imu); spin_for(.1)
        odom_pub.publish(odom); spin_for(.2)
        assert len(received) == 1
        assert bool(transforms) == publish_tf
        if publish_tf:
            assert transforms[0].transforms[0].child_frame_id == 'base_link'
            assert transforms[0].transforms[0].transform.translation.x == pytest.approx(0.)
        assert received[0].child_frame_id == 'base_link'
        assert received[0].pose.pose.position.x == pytest.approx(0.)
        assert received[0].twist.twist.linear.y == pytest.approx(0.)
        odom.header.stamp.sec = 11
        odom_pub.publish(odom); spin_for(.2)
        assert len(received) == 1
        # A fresh gyro with a future stamp must not rescue this stale measurement.
        imu.header.stamp.sec = 12; imu_pub.publish(imu); spin_for(.1)
        odom_pub.publish(odom); spin_for(.2)
        assert len(received) == 1
    finally:
        node.destroy_node(); probe.destroy_node(); rclpy.shutdown()


@pytest.mark.parametrize('publish_tf', [True, None, False])
def test_launch_requires_exclusive_tf_ownership(tmp_path, publish_tf):
    import yaml
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext

    share = Path(get_package_share_directory('aims_racer_system'))
    spec = importlib.util.spec_from_file_location(
        'rear_frames_launch', share/'launch/rear_axle_frames.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    config = yaml.safe_load((share/'params/fastlio_rear.yaml').read_text())
    if publish_tf is None:
        config.pop('publish_tf', None)
    else:
        config['publish_tf'] = publish_tf
    config_path = tmp_path/'lio.yaml'
    config_path.write_text(yaml.safe_dump(config))
    context = LaunchContext()
    context.launch_configurations.update({
        'lio_config': str(config_path),
        'geometry_config': str(share/'params/rear_axle_geometry.yaml'),
        'publish_odom_tf': 'false',
    })
    if publish_tf is False:
        assert len(module.build_nodes(context)) == 6
    else:
        with pytest.raises(ValueError, match='publish_tf: false'):
            module.build_nodes(context)
