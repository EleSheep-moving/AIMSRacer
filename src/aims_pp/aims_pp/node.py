"""50 Hz ROS adapter for independent enhanced PP; starts disabled."""
import csv
import math
from pathlib import Path
import queue
import signal
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from vesc_msgs.msg import VescCommand

from .core import PPConfig
from .reference import NumericReference
from .tracking import PPTrackingRuntime, TrackingInputs


class RuntimeLog:
    """Bounded callback queue; filesystem operations run on a dedicated thread."""
    columns = ['event','ros_ns','steady_s','callback_s','publish_gap_s','source_age_s','phase','speed','steering','progress','traveled','reason']
    def __init__(self, directory):
        self.queue = queue.Queue(maxsize=1024)
        self.dropped = 0
        self.error = ''
        self.file = None
        self.stop = threading.Event()
        self.thread = None
        if directory:
            path = Path(directory)
            path.mkdir(parents=True,exist_ok=True)
            self.file = (path/'runtime.csv').open('x',newline='')
            self.writer = csv.DictWriter(self.file,fieldnames=self.columns)
            self.writer.writeheader()
            self.file.flush()
            self.thread = threading.Thread(target=self._run,name='pp_csv',daemon=True)
            self.thread.start()

    def submit(self, **values):
        if self.file is None:
            return
        try:
            self.queue.put_nowait({k:values.get(k,'') for k in self.columns})
        except queue.Full:
            self.dropped += 1

    def _run(self):
        try:
            while not self.stop.is_set() or not self.queue.empty():
                try:
                    self.writer.writerow(self.queue.get(timeout=.05))
                except queue.Empty:
                    self.file.flush()
            self.file.flush()
        except OSError as error:
            self.error = str(error)

    def close(self):
        self.stop.set()
        if self.thread:
            self.thread.join(timeout=2.)
        if self.file and not (self.thread and self.thread.is_alive()):
            self.file.close()


def stamp_ns(stamp):
    if stamp.sec<0 or not 0<=stamp.nanosec<1_000_000_000:
        raise ValueError('invalid ROS header stamp')
    return stamp.sec*1_000_000_000+stamp.nanosec


class PPNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__('aims_pp',parameter_overrides=parameter_overrides)
        reference_directory = self.declare_parameter('reference_directory','').value
        vehicle = self.declare_parameter('vehicle_config','').value
        tuning = self.declare_parameter('pp_config','').value
        log_directory = self.declare_parameter('log_directory','').value
        speed_limit = self.declare_parameter('speed_limit',1.).value
        repeat_laps = self.declare_parameter('repeat_laps',False).value
        auto_start = self.declare_parameter('auto_start',False).value
        if type(auto_start) is not bool or auto_start:
            raise ValueError('PP requires explicit /pp/enable; auto_start must be false')
        if not reference_directory or not vehicle:
            raise ValueError('reference_directory and vehicle_config are required')
        self.config = PPConfig.from_files(vehicle,tuning)
        self.reference = NumericReference.load(reference_directory).validate_config(self.config)
        self.inputs = TrackingInputs(self.config,self.reference)
        self.runtime = PPTrackingRuntime(self.inputs,self.reference,speed_limit,repeat_laps)
        self.log = RuntimeLog(log_directory)
        self.sequence = 0
        self.last_publish = None
        self.callback_count = 0
        self.callback_max_s = 0.
        reliable = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,'/drive',reliable)
        self.status_pub = self.create_publisher(DiagnosticArray,'/pp/status',10)
        self.reference_pub = self.create_publisher(RosPath,'/pp/reference',latched)
        self.lookahead_pub = self.create_publisher(PointStamped,'/pp/lookahead_point',reliable)
        self.create_subscription(Odometry,'/odometry/filtered',self._odom,QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(DiagnosticArray,'/localization/status',self._health,10)
        self.create_subscription(String,'/localization/map_sha256',self._map,latched)
        self.create_subscription(Bool,'/localization/map_valid',self._valid,latched)
        self.create_subscription(Bool,'/control/autonomy_speed_enabled',self._authority,10)
        self.create_subscription(VescCommand,'/vesc/command',self._command,reliable)
        self.create_service(SetBool,'/pp/enable',self._enable)
        self.publication_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.02,self._tick,clock=self.publication_clock)
        self.create_timer(.1,self._status,clock=self.publication_clock)
        self.add_on_set_parameters_callback(self._guard_parameters)
        self._publish_reference()
        self.get_logger().info('Enhanced PP ready: 50 Hz, disabled, trusted map alignment, explicit enable required')

    def _guard_parameters(self, parameters):
        valid = all(p.name=='use_sim_time' for p in parameters)
        return SetParametersResult(successful=valid,reason='' if valid else 'PP configuration is fixed; restart with matching reference and vehicle')

    def _sample_clocks(self):
        ros,mono = self.get_clock().now().nanoseconds,time.monotonic()
        self.inputs.observe_clock(ros)
        return ros,mono

    def _accept(self, event, action):
        started = time.monotonic()
        ros,mono = self._sample_clocks()
        try:
            action(ros,mono)
        except (ValueError,TypeError,ArithmeticError) as error:
            self.inputs.invalidate(f'{event}: {error}')
        if self.runtime.enabled and self.inputs.fault_generation!=self.runtime.generation:
            self.runtime.fault(self.inputs.fault_reason)
            self._publish(dict(speed=0.,steering=0.),ros,mono)
        elapsed = time.monotonic()-started
        self.callback_count += 1
        self.callback_max_s = max(self.callback_max_s,elapsed)
        self.log.submit(event=event,ros_ns=ros,steady_s=mono,callback_s=elapsed,phase=self.runtime.phase,reason=self.inputs.fault_reason)

    def _odom(self, message):
        def accept(ros,mono):
            q = message.pose.pose.orientation
            values = [q.x,q.y,q.z,q.w]
            norm = math.sqrt(sum(value*value for value in values))
            if not math.isfinite(norm) or abs(norm-1.)>.01:
                self.inputs.pose = None
                raise ValueError('nonunit odometry quaternion')
            qx,qy,qz,qw = [value/norm for value in values]
            yaw = math.atan2(2*(qw*qz+qx*qy),1-2*(qy*qy+qz*qz))
            p = message.pose.pose.position
            self.inputs.observe_odometry(p.x,p.y,yaw,message.twist.twist.linear.x,stamp_ns(message.header.stamp),ros,mono,message.header.frame_id,message.child_frame_id)
        self._accept('odometry',accept)

    def _health(self, message):
        def accept(ros,mono):
            age = (ros-stamp_ns(message.header.stamp))*1e-9
            records = [record for record in message.status if record.name=='aims_racer_system/localization']
            values = {}
            if len(records)==1 and 0<=age<=.3:
                for item in records[0].values:
                    if item.key in values:
                        values = {}
                        break
                    values[item.key] = item.value
            self.inputs.observe_health(values,ros,mono)
        self._accept('health',accept)

    def _map(self, message):
        self._accept('map',lambda ros,mono:self.inputs.observe_map(message.data))

    def _valid(self, message):
        self._accept('map_valid',lambda ros,mono:self.inputs.observe_map_valid(message.data))

    def _authority(self, message):
        self._accept('authority',lambda ros,mono:self.inputs.observe_authority(message.data,mono))

    def _command(self, message, info=None):
        def accept(ros,mono):
            if info is not None and info.source_timestamp>0:
                dds_age = (time.time_ns()-info.source_timestamp)*1e-9
                if not 0<=dds_age<=.1:
                    self.inputs.command_valid = False
                    raise ValueError('stale final VESC DDS source')
            targets = (message.target_speed_mps,message.target_current,message.target_duty,message.target_brake_current,message.target_acceleration_mps2)
            self.inputs.observe_command(message.control_mode,message.steering_angle,message.target_speed_mps,stamp_ns(message.header.stamp),ros,mono,message.header.frame_id,targets)
        self._accept('final_command',accept)

    def _enable(self, request, response):
        ros,mono = self._sample_clocks()
        if request.data:
            response.success,response.message = self.runtime.enable(ros,mono,self.count_publishers('/drive'))
        else:
            self.runtime.disable()
            response.success,response.message = True,self.runtime.reason
        if not self.runtime.enabled:
            self._publish(dict(speed=0.,steering=0.),ros,mono)
        self.log.submit(event='enable' if request.data else 'disable',ros_ns=ros,steady_s=mono,phase=self.runtime.phase,reason=response.message)
        return response

    def _tick(self):
        started = time.monotonic()
        ros,mono = self._sample_clocks()
        result = self.runtime.tick(ros,mono,self.count_publishers('/drive'))
        self._publish(result,ros,mono)
        target = result.get('lookahead')
        if target:
            point = PointStamped()
            point.header.stamp = self.get_clock().now().to_msg()
            point.header.frame_id = 'map'
            point.point.x,point.point.y = target['x'],target['y']
            self.lookahead_pub.publish(point)
        self.log.submit(event='command',ros_ns=ros,steady_s=mono,callback_s=time.monotonic()-started,
                        publish_gap_s='' if self.previous_publish is None else mono-self.previous_publish,
                        source_age_s=mono-self.inputs.source,phase=self.runtime.phase,speed=result['speed'],steering=result['steering'],
                        progress=result.get('progress',0.),traveled=result.get('traveled',0.),reason=self.runtime.reason)

    def _publish(self, result, ros, mono):
        command = AckermannDriveStamped()
        command.header.stamp.sec,command.header.stamp.nanosec = divmod(ros,1_000_000_000)
        command.header.frame_id = 'base_link'
        command.drive.speed,command.drive.steering_angle = float(result['speed']),float(result['steering'])
        self.drive_pub.publish(command)
        self.previous_publish,self.last_publish = self.last_publish,mono
        self.sequence += 1

    def _status(self):
        ros,mono = self._sample_clocks()
        gate = self.inputs.reason(ros,mono,self.count_publishers('/drive'))
        values = dict(controller='enhanced_pp',phase=self.runtime.phase,enabled=str(self.runtime.enabled).lower(),reason=self.runtime.reason,
                      gate=gate,sequence=self.sequence,progress=self.runtime.progress,traveled=self.runtime.traveled,speed_limit=self.runtime.speed_limit,
                      map_sha256=self.reference.map_sha256,anchor_epoch=self.inputs.health.epoch,anchor_sequence=self.inputs.health.sequence,
                      callback_count=self.callback_count,callback_max_s=self.callback_max_s,log_dropped=self.log.dropped,log_error=self.log.error)
        status = DiagnosticStatus(name='aims_pp/controller',hardware_id='AIMSRacer',level=DiagnosticStatus.ERROR if self.runtime.phase=='FAULT' else DiagnosticStatus.OK,message=self.runtime.reason,
                                  values=[KeyValue(key=k,value=str(v)) for k,v in values.items()])
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.status_pub.publish(array)

    def _publish_reference(self):
        path = RosPath()
        path.header.stamp,path.header.frame_id = self.get_clock().now().to_msg(),'map'
        for s in self.reference.sample_positions[::max(1,len(self.reference.sample_positions)//2000)]:
            at = self.reference.at(s)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x,pose.pose.position.y = at['x'],at['y']
            pose.pose.orientation.z,pose.pose.orientation.w = math.sin(at['yaw']/2),math.cos(at['yaw']/2)
            path.poses.append(pose)
        if path.poses:
            path.poses.append(path.poses[0])
        self.reference_pub.publish(path)

    def destroy_node(self):
        if hasattr(self,'runtime'):
            self.runtime.disable()
        if hasattr(self,'drive_pub') and rclpy.ok():
            ros,mono = self._sample_clocks()
            self._publish(dict(speed=0.,steering=0.),ros,mono)
        if hasattr(self,'log'):
            self.log.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PPNode()
        rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        # ROS launch may forward SIGINT after the process group already received
        # it. Finish flushing the bounded log once, without interrupting cleanup.
        previous = signal.signal(signal.SIGINT,signal.SIG_IGN)
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            signal.signal(signal.SIGINT,previous)
