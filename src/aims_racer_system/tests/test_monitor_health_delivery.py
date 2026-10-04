"""Exercise real monitor publication with ROS message stubs on non-ROS hosts."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace


def monitor_module(monkeypatch, node_type=object):
    def module(name, **attributes):
        result = ModuleType(name)
        result.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, result)
        return result
    class Message(SimpleNamespace):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
    class DiagnosticStatus(Message):
        OK, ERROR = 0, 2
    class DiagnosticArray(Message):
        def __init__(self, **kwargs):
            super().__init__(header=SimpleNamespace(stamp=None), **kwargs)
    module('rclpy')
    module('rclpy.node', Node=node_type)
    module('rclpy.clock', Clock=object, ClockType=SimpleNamespace(STEADY_TIME=1))
    module('rclpy.qos', QoSProfile=object, DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1),
           qos_profile_sensor_data=None)
    module('diagnostic_msgs'); module('diagnostic_msgs.msg', DiagnosticArray=DiagnosticArray,
                                     DiagnosticStatus=DiagnosticStatus, KeyValue=Message)
    module('nav_msgs'); module('nav_msgs.msg', Odometry=Message)
    module('sensor_msgs'); module('sensor_msgs.msg', PointCloud2=Message)
    module('sensor_msgs_py', point_cloud2=SimpleNamespace())
    module('std_msgs'); module('std_msgs.msg', Bool=Message, String=Message)
    module('tf2_msgs'); module('tf2_msgs.msg', TFMessage=Message)
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    monkeypatch.syspath_prepend(str(scripts))
    spec = importlib.util.spec_from_file_location('tested_monitor', scripts / 'localization_monitor.py')
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def test_watchdog_heartbeats_increment_health_sequence_without_anchor_events(monkeypatch):
    module = monitor_module(monkeypatch)
    from localization_policy import AnchorHealth
    node = module.LocalizationMonitor.__new__(module.LocalizationMonitor)
    node.health = AnchorHealth()
    node.health_publish_epoch = ''
    node.health_publish_sequence = 0
    node.ekf_max_age = .1
    node.cloud_max_age = .5
    node.sha = 'map'
    node.quality_error = ''
    node.quality = {}
    node.inputs = {'ekf': (10_000_000_000, 1.), 'body_cloud': (10_000_000_000, 1.)}
    node.status_pub = SimpleNamespace(messages=[])
    node.status_pub.publish = node.status_pub.messages.append
    node.valid_pub = SimpleNamespace(messages=[])
    node.valid_pub.publish = node.valid_pub.messages.append
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))
    clock = [1.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: clock[0])
    event = dict(protocol_version='1', epoch='process:1', event_sequence='1', anchor_sequence='1',
                 anchor_committed='true', ready='true', reason='ok', last_anchor_stamp_ns='10000000000')
    node.health.observe(event, 10_000_000_000, 10_000_000_000, 1.)
    node.publish_status(10_000_000_000)
    clock[0] = 1.6
    node.publish_status(10_600_000_000)
    delivered = [{v.key: v.value for v in message.status[0].values} for message in node.status_pub.messages]
    assert [v['ready'] for v in delivered] == ['true', 'false']
    assert [v['health_sequence'] for v in delivered] == ['1', '2']
    assert [v['anchor_sequence'] for v in delivered] == ['1', '1']
    assert [v['event_sequence'] for v in delivered] == ['1', '1']
    # A new native epoch owns a new health sequence domain.
    event.update(epoch='process:2', last_anchor_stamp_ns='10600000000')
    node.health.observe(event, 10_600_000_000, 10_600_000_000, 1.6)
    node.inputs = {'ekf': (10_600_000_000, 1.6), 'body_cloud': (10_600_000_000, 1.6)}
    node.publish_status(10_600_000_000)
    latest = {v.key: v.value for v in node.status_pub.messages[-1].status[0].values}
    assert latest['epoch'] == 'process:2' and latest['health_sequence'] == '1'


def test_static_subscription_keeps_multiple_latched_publishers(monkeypatch, tmp_path):
    class StubNode:
        def __init__(self, name):
            pass
        def declare_parameter(self, name, default):
            return SimpleNamespace(value=str(tmp_path / 'map.pcd') if name == 'map_file' else default)
        def create_publisher(self, *args):
            return SimpleNamespace(publish=lambda message: None)
        def create_subscription(self, message_type, topic, callback, qos):
            return SimpleNamespace(topic=topic, callback=callback, qos=qos)
        def create_timer(self, *args, **kwargs):
            return None
    points = '\n'.join(f'{i} 0 0' for i in range(100))
    (tmp_path / 'map.pcd').write_text('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 100\nDATA ascii\n'+points)
    module = monitor_module(monkeypatch, StubNode)
    monkeypatch.setattr(module, 'QoSProfile', SimpleNamespace)
    monkeypatch.setattr(module, 'Clock', lambda **kwargs: None)
    node = module.LocalizationMonitor()
    try:
        static = next(s for s in node.subscriptions_owned if s.topic == '/tf_static')
        assert static.qos.depth >= 2
        assert static.qos.durability == module.DurabilityPolicy.TRANSIENT_LOCAL
        # Separate static messages preserve the Livox mount when footprint arrives.
        def packet(child):
            return SimpleNamespace(transforms=[SimpleNamespace(
                header=SimpleNamespace(frame_id='base_link'), child_frame_id=child,
                transform=SimpleNamespace(translation=SimpleNamespace(x=.3,y=0.,z=.03),
                                          rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))])
        static.callback(packet('livox_frame'))
        static.callback(packet('base_footprint'))
        assert node.mount[0,3] == .3
    finally:
        node.worker.shutdown(wait=True)


def test_diagnostic_missing_input_reasons_are_specific(monkeypatch):
    import numpy as np
    module = monitor_module(monkeypatch)
    node = module.LocalizationMonitor.__new__(module.LocalizationMonitor)
    node.future = None
    node.cloud = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
    node.last_quality_stamp = None
    node.history = [(10.,np.eye(4))]
    node.map_packets = module.MapOdomPackets()
    node.map_packets.add(10_000_000_000,np.eye(4))
    node.mount = None
    node.launch_quality()
    assert node.quality_error == 'quality_mount_unavailable'
    node.mount = np.eye(4)
    node.history = []
    node.launch_quality()
    assert node.quality_error == 'quality_source_pose_unavailable'
    node.history = [(10.,np.eye(4))]
    node.map_packets.clear()
    node.launch_quality()
    assert node.quality_error == 'quality_correction_unavailable'


def test_quality_bounded_query_preserves_exact_boundary_and_inlier_rmse(monkeypatch):
    import math
    import numpy as np
    from scipy.spatial import cKDTree
    module = monitor_module(monkeypatch)
    monkeypatch.setattr(module.point_cloud2, 'read_points',
                        lambda *args, **kwargs: [(.25,0.,0.), (.1,0.,0.), (2.,0.,0.)], raising=False)
    class QueryAudit:
        def __init__(self):
            self.tree = cKDTree([[0.,0.,0.]])
        def query(self, points, **kwargs):
            result = self.tree.query(points, **kwargs)
            self.distances = result[0]
            return result
    tree = QueryAudit()
    cloud = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10,nanosec=0)))
    result = module.quality_check(cloud,np.eye(4),np.eye(4),np.eye(4),tree)
    assert result['quality_points'] == 3
    assert result['inlier_fraction'] == 2/3
    assert result['inlier_rmse_m'] == math.sqrt((.25**2+.1**2)/2)
    assert tree.distances[0] == .25
    assert math.isinf(tree.distances[2])
    # An all-outlier scan retains zero inlier fraction and undefined inlier RMSE.
    monkeypatch.setattr(module.point_cloud2, 'read_points', lambda *args, **kwargs: [(2.,0.,0.)])
    result = module.quality_check(cloud,np.eye(4),np.eye(4),np.eye(4),tree)
    assert result['inlier_fraction'] == 0.
    assert math.isnan(result['inlier_rmse_m'])
