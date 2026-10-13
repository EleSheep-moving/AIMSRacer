"""50 Hz calibration producer. All requests pass through joystick_control_v2."""
from dataclasses import fields, replace, asdict
import hashlib
import json
import math
import signal
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import AccelWithCovarianceStamped
from crsf_receiver_msg.msg import CRSFChannels16
from vesc_msgs.msg import VescCommand, VescStateStamped
import yaml

from .engine import Config, Engine, Observation, Request, RCProfile, Regions
from .io import motor_telemetry, apply_request, AsyncOutputLog, health_values, _json_safe
from .operator import check_start
from .configuration import prepare_engine,tracking_settings_from_parameters


def seconds(stamp): return float(stamp.sec)+float(stamp.nanosec)*1e-9


class CalibrationNode(Node):
    def __init__(self,parameter_overrides=None):
        super().__init__('longitudinal_calibration',parameter_overrides=parameter_overrides or [])
        default_root=Path.home()/'aimsracer-data'/'calibration'
        for name,value in dict(reference_directory='',vehicle_config='',pp_config='',log_directory=str(default_root),
                               auto_start=False,rc_profile='steering_ch1_throttle_ch3_aux_ch5_to_ch10',config_file='').items():
            self.declare_parameter(name,value)
        if self.get_parameter('auto_start').value:
            raise ValueError('auto_start must remain false; use explicit /calib/enable service')
        defaults=Config()
        values={}
        for field in fields(Config):
            value=getattr(defaults,field.name)
            if isinstance(value,tuple):
                parameter=self.declare_parameter(field.name,list(value) if value else Parameter.Type.DOUBLE_ARRAY)
                values[field.name]=tuple(parameter.value or ())
            else: values[field.name]=self.declare_parameter(field.name,value).value
        vehicle_path=self.get_parameter('vehicle_config').value
        if not vehicle_path: raise ValueError('vehicle_config is required')
        vehicle=yaml.safe_load(Path(vehicle_path).read_text())
        values['max_speed']=min(values['max_speed'],float(vehicle['max_speed']))
        extra_defaults=dict(plan_mode='manual',base_current_file='',current_step=5.,
            current_settle_seconds=.25,current_window_seconds=.5,current_min_samples=10,
            current_absolute_tolerance=1.,current_relative_tolerance=.1,current_failure_fraction=.9,max_attempts=3)
        extra={name:self.declare_parameter(name,value).value for name,value in extra_defaults.items()}
        settings=tracking_settings_from_parameters(extra)
        self.engine,sweep_metadata,manifest=prepare_engine(values,plan_mode=extra['plan_mode'],
            base_current_file=extra['base_current_file'],current_step=extra['current_step'],tracking_settings=settings)
        self.obs=Observation()
        self.require_acceleration=self.declare_parameter('require_acceleration',False).value
        self.enable_count=0; self.driver_startup_disabled=False; self.driver_parameter_received=-math.inf
        self.driver_query=None; self.acceleration_status={}
        self.driver_parameters=self.create_client(GetParameters,'/vesc_driver_node/get_parameters')
        self.profile=RCProfile(name=self.get_parameter('rc_profile').value)
        self.receipts={}; self.last_request=Request(0.); self.tracking=None
        self.evidence_error=''; self.results_dirty=False
        self.tracking_generation=0
        self.last_ros=None; self.last_ros_changed=time.monotonic(); self.last_timer=None
        metadata=dict(vehicle_config=str(Path(vehicle_path).resolve()),
            vehicle_sha256=hashlib.sha256(Path(vehicle_path).read_bytes()).hexdigest(),
            rc_profile=self.profile.name,
            configuration_sha256=hashlib.sha256(json.dumps(asdict(self.engine.config),sort_keys=True,separators=(',',':')).encode()).hexdigest())
        config_file=self.get_parameter('config_file').value
        if config_file:
            metadata.update(calibration_config=config_file,calibration_config_sha256=hashlib.sha256(Path(config_file).read_bytes()).hexdigest())
        metadata.update(sweep_metadata)
        if self.engine.config.steering_source=='pp':
            from aims_pp.core import PPConfig, EnhancedPP
            from aims_pp.reference import NumericReference
            from aims_pp.tracking import TrackingInputs, PPTrackingRuntime
            directory=self.get_parameter('reference_directory').value
            self.pp_config=PPConfig.from_files(vehicle_path,self.get_parameter('pp_config').value)
            self.reference=NumericReference.load(directory)
            self.tracking=TrackingInputs(self.pp_config,self.reference)
            self.association_runtime=PPTrackingRuntime(self.tracking,self.reference,repeat_laps=True)
            self.pp=EnhancedPP(self.pp_config)
            self.regions=Regions.load(directory,self.reference.length)
            metadata.update(reference_directory=directory,
                reference_sha256=hashlib.sha256((Path(directory)/'reference.json').read_bytes()).hexdigest(),
                calibration_regions_sha256=hashlib.sha256((Path(directory)/'calibration_regions.json').read_bytes()).hexdigest(),
                map_sha256=self.reference.map_sha256,pp_config=self.get_parameter('pp_config').value,
                effective_pp_configuration=asdict(self.pp_config))
            pp_path=self.get_parameter('pp_config').value
            if pp_path: metadata['pp_config_sha256']=hashlib.sha256(Path(pp_path).read_bytes()).hexdigest()
        if manifest is not None and manifest.get('baseline') is not None:
            source=manifest['baseline']['source']['context']
            keys=['vehicle_sha256']
            if self.tracking:keys+=['map_sha256','reference_sha256','calibration_regions_sha256']
            for key in keys:
                if source.get(key) is not None and source[key]!=metadata.get(key):
                    raise ValueError('baseline '+key+' differs from current calibration')
        self.output=AsyncOutputLog(self.get_parameter('log_directory').value,self.engine.config,metadata,manifest=manifest)
        latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,lifespan=Duration(seconds=.1))
        self.command_pub=self.create_publisher(VescCommand,'/calib/vesc_command',latest)
        self.status_pub=self.create_publisher(DiagnosticArray,'/calib/status',10)
        sensor_latest=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry,'/rear_axle/wheel_odom',self._wheel,qos_profile_sensor_data)
        self.create_subscription(VescStateStamped,'/sensors/core',self._telemetry,qos_profile_sensor_data)
        self.create_subscription(CRSFChannels16,'/rc/channels',self._rc,sensor_latest)
        self.create_subscription(Bool,'/control/calibration_enabled',self._authority,latest)
        self.create_subscription(VescCommand,'/vesc/command',self._final,latest)
        self.create_subscription(AccelWithCovarianceStamped,'/state_estimation/longitudinal_acceleration',self._acceleration,qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray,'/state_estimation/longitudinal_acceleration/status',self._acceleration_status,10)
        if self.tracking:
            durable=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
            self.create_subscription(Odometry,'/odometry/filtered',self._odometry,qos_profile_sensor_data)
            self.create_subscription(DiagnosticArray,'/localization/status',self._health,10)
            self.create_subscription(String,'/localization/map_sha256',self._map_hash,durable)
            self.create_subscription(Bool,'/localization/map_valid',self._map_valid,durable)
        self.create_service(SetBool,'/calib/enable',self._enable)
        self.safety_clock=Clock(clock_type=ClockType.STEADY_TIME)
        self.command_timer=self.create_timer(.02,self._tick,clock=self.safety_clock)
        self.status_timer=self.create_timer(.2,self._status,clock=self.safety_clock)
        if self.tracking:
            self.reference_pub=self.create_publisher(RosPath,'/calib/reference',durable)
            self._publish_reference()
        self.get_logger().info(f'Calibration disabled by default; evidence: {self.output.directory}')

    def _now(self): return self.get_clock().now().nanoseconds*1e-9

    def _receipt(self,name): self.receipts[name]=time.monotonic()

    def _fault(self,reason):
        if self.engine.enabled and self.engine.selected: self.engine.fault(reason)

    def _wheel(self,msg):
        self._receipt('wheel')
        value=float(msg.twist.twist.linear.x)
        if msg.child_frame_id!='base_link': value=math.nan
        self.obs=replace(self.obs,wheel_stamp=seconds(msg.header.stamp),velocity=value)

    def _telemetry(self,msg):
        self._receipt('telemetry')
        try: current,voltage,erpm=motor_telemetry(msg)
        except ValueError: current=voltage=erpm=math.nan
        self.obs=replace(self.obs,telemetry_stamp=seconds(msg.header.stamp),
                         current_motor=current,voltage_input=voltage,erpm=erpm,fault_code=int(msg.state.fault_code))

    def _rc(self,msg):
        self._receipt('rc')
        valid,steering,straight=self.profile.evaluate([getattr(msg,f'ch{i}') for i in range(1,17)])
        self.obs=replace(self.obs,rc_stamp=self._now(),rc_valid=valid,
                         steering=steering if self.tracking is None else self.obs.steering,
                         straight=straight if self.tracking is None else self.obs.straight)
        if not valid: self._fault('rc_invalid_or_locked')

    def _authority(self,msg):
        self._receipt('authority')
        self.obs=replace(self.obs,authority=msg.data,authority_stamp=self._now())
        if not msg.data: self._fault('authority_withdrawn')

    def _final(self,msg):
        self._receipt('final')
        mode=int(msg.control_mode)
        targets=(msg.target_speed_mps,msg.target_current,msg.target_duty,msg.target_brake_current,msg.target_acceleration_mps2)
        valid=msg.header.frame_id=='base_link' and mode in (0,1) and all(math.isfinite(v) for v in (*targets,msg.steering_angle))
        valid=valid and all(v==0. for i,v in enumerate(targets) if i!=mode)
        command=Request(seconds(msg.header.stamp),mode,float(msg.steering_angle),float(targets[0]),float(targets[1])) if valid else None
        self.obs=replace(self.obs,final_request=command,final_received=self._now())
        if not valid: self._fault('invalid_final_command')

    def _acceleration(self,msg):
        self._receipt('acceleration')
        value=float(msg.accel.accel.linear.x)
        self.obs=replace(self.obs,acceleration=value,acceleration_stamp=seconds(msg.header.stamp))

    def _acceleration_status(self,msg):
        self._receipt('acceleration_quality')
        self.acceleration_status={v.key:v.value for status in msg.status for v in status.values}
        quality='; '.join(f'{s.level}:{s.message}' for s in msg.status)
        self.obs=replace(self.obs,acceleration_quality=quality or 'unavailable')

    def _odometry(self,msg):
        self._receipt('odometry')
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        norm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
        try:
            if not math.isfinite(norm) or abs(norm-1.)>.01: raise ValueError('invalid odometry quaternion')
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            self.tracking.observe_odometry(p.x,p.y,yaw,msg.twist.twist.linear.x,
                int(msg.header.stamp.sec)*10**9+msg.header.stamp.nanosec,
                self.get_clock().now().nanoseconds,time.monotonic(),msg.header.frame_id,msg.child_frame_id)
        except (ValueError,TypeError) as exc: self._fault(str(exc))

    def _health(self,msg):
        values=health_values(msg,self.get_clock().now().nanoseconds)
        self.tracking.observe_health(values,self.get_clock().now().nanoseconds,time.monotonic())
        if self.tracking.fault_generation!=self.tracking_generation: self._fault(self.tracking.fault_reason)

    def _map_hash(self,msg):
        self.tracking.observe_map(msg.data)
        if msg.data!=self.reference.map_sha256: self._fault('map_identity_mismatch')

    def _map_valid(self,msg):
        self.tracking.observe_map_valid(msg.data)
        if not msg.data: self._fault('map_validity_withdrawn')

    def _poll_driver_parameters(self):
        if self.driver_query is not None:
            if not self.driver_query.done(): return
            try:
                values=self.driver_query.result().values
                self.driver_startup_disabled=len(values)==1 and values[0].type==1 and not values[0].bool_value
                self.driver_parameter_received=time.monotonic()
            except Exception:
                self.driver_startup_disabled=False
            self.driver_query=None
        if time.monotonic()-self.driver_parameter_received>=.5 and self.driver_parameters.service_is_ready():
            request=GetParameters.Request(); request.names=['current_startup_enabled']
            self.driver_query=self.driver_parameters.call_async(request)

    def preflight_snapshot(self,verify_artifacts=False):
        now=self._now(); steady=time.monotonic();obs=self.obs
        ages={name+'_source':now-stamp for name,stamp in (
            ('wheel',obs.wheel_stamp),('telemetry',obs.telemetry_stamp),
            ('final',obs.final_request.stamp if obs.final_request else -math.inf),
            ('rc',obs.rc_stamp),('authority',obs.authority_stamp))}
        ages.update({name+'_receipt':steady-self.receipts.get(name,-math.inf)
                     for name in ('wheel','telemetry','final','rc','authority')})
        snapshot=dict(state=self.engine.state,enabled=self.engine.enabled,enable_count=self.enable_count,
            logging_error=str(self.output.error or self.evidence_error),logging_dropped=self.output.dropped_count,
            freshness=self.engine.config.freshness,ages=ages,wheel_speed=obs.velocity,
            telemetry_valid=all(math.isfinite(v) for v in (obs.current_motor,obs.voltage_input,obs.erpm)) and obs.voltage_input>0
                and (self.engine.config.stage=='speed_hold' or obs.fault_code==0),
            authority=obs.authority,rc_valid=obs.rc_valid,
            final_zero=obs.final_request is not None and obs.final_request.mode==0 and obs.final_request.speed==0 and obs.final_request.current==0,
            publishers=dict(calibration=self.count_publishers('/calib/vesc_command'),final=self.count_publishers('/vesc/command'),drive=self.count_publishers('/drive')),
            driver_startup_disabled=self.driver_startup_disabled,driver_parameter_age=steady-self.driver_parameter_received,
            pp=self.tracking is not None,localization_reason='',cross_track=None,heading_error=None,
            require_acceleration=self.require_acceleration,artifact_error='')
        try:
            force_ages=[float(self.acceleration_status[key]) for key in ('wheel_age_s','imu_age_s','force_age_s')]
        except (KeyError,ValueError,TypeError): force_ages=[math.inf]
        snapshot['acceleration_valid']=(self.acceleration_status.get('initialized')=='true' and
            self.acceleration_status.get('force_valid')=='true' and all(math.isfinite(age) and 0<=age<=.1 for age in force_ages) and
            math.isfinite(obs.acceleration) and 0<=now-obs.acceleration_stamp<=.1 and
            0<=steady-self.receipts.get('acceleration',-math.inf)<=.1 and
            0<=steady-self.receipts.get('acceleration_quality',-math.inf)<=2.)
        if self.tracking:
            self.tracking.observe_clock(int(now*1e9))
            snapshot['localization_reason']=self.tracking.localization_reason(int(now*1e9),steady)
            if not snapshot['localization_reason']:
                try:
                    from aims_pp.core import wrap_angle
                    pose,s,lateral=self.association_runtime.association()
                    snapshot.update(cross_track=lateral,heading_error=wrap_angle(pose[2]-self.reference.at(s)['yaw']),progress=s)
                except ValueError as error: snapshot['localization_reason']=str(error)
        if verify_artifacts:
            metadata=self.output.writer.metadata
            def check_file(path,expected,label):
                try:
                    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=expected:
                        snapshot['artifact_error']=label+' changed after launch'
                except (OSError,TypeError) as error:
                    snapshot['artifact_error']=label+' unavailable: '+str(error)
            for key,hash_key in (('vehicle_config','vehicle_sha256'),('calibration_config','calibration_config_sha256'),
                                ('pp_config','pp_config_sha256'),('base_current_file','base_current_file_sha256')):
                path=metadata.get(key)
                if path: check_file(path,metadata.get(hash_key),key)
            if self.tracking:
                for name,key in (('reference.json','reference_sha256'),('calibration_regions.json','calibration_regions_sha256')):
                    path=Path(metadata['reference_directory'])/name
                    check_file(path,metadata[key],name)
        return _json_safe(snapshot)

    def _publish_reference(self):
        path=RosPath(); path.header.frame_id='map';path.header.stamp=self.get_clock().now().to_msg()
        for position in self.reference.sample_positions[::max(1,len(self.reference.sample_positions)//2000)]:
            at=self.reference.at(position);point=PoseStamped();point.header=path.header
            point.pose.position.x=at['x'];point.pose.position.y=at['y']
            point.pose.orientation.z=math.sin(at['yaw']/2);point.pose.orientation.w=math.cos(at['yaw']/2)
            path.poses.append(point)
        self.reference_pub.publish(path)

    def _begin(self):
        self.engine.enable(True,now=self._now())
        self.obs=Observation(); self.receipts.clear()
        self.last_ros=None; self.last_ros_changed=time.monotonic(); self.last_timer=None
        self.output.reset_observations()
        if self.tracking: self.tracking_generation=self.tracking.fault_generation
        self.output.trials(self.engine.trials,self.evidence_error)

    def _enable(self,request,response):
        try:
            if request.data:
                snapshot=self.preflight_snapshot(verify_artifacts=True)
                reasons=check_start(snapshot)
                if reasons:
                    response.success=False; response.message='Start refused: '+', '.join(reasons)
                    return response
                marker=self.output.directory/'enable-attempt.json'
                with marker.open('x') as output:
                    json.dump(dict(requested_at_ns=time.time_ns(),preflight=snapshot),output,allow_nan=False)
                self.enable_count+=1
                self._begin()
            else: self.engine.enable(False); self.output.trials(self.engine.trials,self.evidence_error)
        except (RuntimeError,OSError,ValueError) as exc:
            self.engine.fault(str(exc)); response.success=False; response.message=str(exc)
            return response
        response.success=True
        response.message='PREPARE; awaiting fresh selected calibration authority' if request.data else 'Stopped; incomplete trial invalidated'
        return response

    def _tracking_observation(self,obs,now,steady):
        if self.tracking is None: return obs
        self.tracking.observe_clock(int(now*1e9))
        reason=self.tracking.localization_reason(int(now*1e9),steady)
        if self.engine.enabled and self.engine.selected and self.tracking.fault_generation!=self.tracking_generation:
            reason=self.tracking.fault_reason or 'localization_validity_withdrawn'
        if reason: return replace(obs,straight=False,tracking_reason=reason)
        try:
            pose,s,_=self.association_runtime.association()
            # Nearest-point progress can change abruptly on a curve even
            # with continuous localization; it is not a localization fault.
            target=self.reference.at(s+self.pp.compute_lookahead(pose[3]))
            steering,_=self.pp.compute_steering(pose[:2],pose[2],(target['x'],target['y']),target['yaw'],target['curvature'],pose[3])
            region=self.regions.region(s)
            return replace(obs,steering=steering,straight=region is not None,region=region or '',tracking_reason='')
        except ValueError as exc: return replace(obs,straight=False,tracking_reason=str(exc))

    def _tick(self):
        callback_start=time.monotonic()
        first_new_trial=len(self.engine.trials)
        ros_stamp=self.get_clock().now(); now=ros_stamp.nanoseconds*1e-9; steady=time.monotonic()
        obs=self.obs
        if self.engine.enabled and self.engine.selected:
            if self.last_timer is not None and steady-self.last_timer>.2: self.engine.fault('publication_timer_stale')
            if self.count_publishers('/calib/vesc_command')!=1: self.engine.fault('multiple_calibration_publishers')
            for name in ('wheel','telemetry','rc','authority','final'):
                if not 0<=steady-self.receipts.get(name,-math.inf)<=self.engine.config.freshness:
                    self.engine.fault(name+'_receipt_stale'); break
        timer_interval=steady-self.last_timer if self.last_timer is not None else 0.
        self.last_timer=steady
        if now!=self.last_ros: self.last_ros_changed=steady
        elif self.engine.enabled and self.engine.selected and steady-self.last_ros_changed>.2:
            self.engine.fault('ros_clock_stalled')
        self.last_ros=now
        obs=self._tracking_observation(obs,now,steady)
        if steady-self.receipts.get('acceleration',-math.inf)>.2 or not 0<=now-obs.acceleration_stamp<=.2:
            obs=replace(obs,acceleration=math.nan,acceleration_quality='estimate_stale_or_unavailable')
        elif steady-self.receipts.get('acceleration_quality',-math.inf)>2.:
            obs=replace(obs,acceleration_quality='quality_stale_or_unavailable')
        if self.output.error is not None:
            reason='logging_writer_failure: '+str(self.output.error)
            self.engine.fault(reason)
            self.engine.invalidate_evidence(min(first_new_trial,self.output.failed_after_trial_count),reason)
            self.evidence_error=reason; self.results_dirty=True
        request=self.engine.tick(now,obs)
        # The request is recorded only after serialization has succeeded.
        message=apply_request(VescCommand(),request,ros_stamp.to_msg())
        self.command_pub.publish(message); self.engine.sent(request); self.last_request=request
        try:
            self.output.observation(obs,request,self.engine.state)
            if self.results_dirty or self.output.trial_count!=len(self.engine.trials) or self.output.skipped_count!=len(self.engine.skipped):
                self.output.trials(self.engine.trials,self.evidence_error,skipped=self.engine.skipped); self.results_dirty=False
            self.output.runtime(dict(ros_stamp=now,callback_ms=(time.monotonic()-callback_start)*1000.,
                timer_interval_ms=timer_interval*1000.,pending=self.output.pending_count,
                dropped=self.output.dropped_count,state=self.engine.state))
        except RuntimeError as exc:
            self.engine.fault(str(exc))
            self.engine.invalidate_evidence(first_new_trial,str(exc))
            self.evidence_error=str(exc); self.results_dirty=True
            # Keep the writer's failure snapshot current even when its queue
            # refuses all further jobs. This only copies data; disk I/O stays
            # on the background writer.
            try:self.output.trials(self.engine.trials,self.evidence_error,skipped=self.engine.skipped)
            except RuntimeError:pass
            stop=self.engine.tick(now,obs)
            self.command_pub.publish(apply_request(VescCommand(),stop,ros_stamp.to_msg()))
            self.engine.sent(stop); self.last_request=stop

    def status_message(self):
        msg=DiagnosticArray(); msg.header.stamp=self.get_clock().now().to_msg()
        status=DiagnosticStatus(); status.name='longitudinal_calibration'; status.hardware_id='vesc_calibration'
        status.level=DiagnosticStatus.ERROR if self.engine.state=='FAULT' else DiagnosticStatus.OK
        status.message=self.engine.reason
        self._poll_driver_parameters()
        snapshot=self.preflight_snapshot()
        reasons=check_start(snapshot)
        values=dict(state=self.engine.state,enabled=str(self.engine.enabled).lower(),
            enable_count=str(self.enable_count),start_ready=str(not reasons).lower(),
            start_refusals=json.dumps(reasons),preflight_json=json.dumps(snapshot,allow_nan=False),
            reason=self.engine.reason,vehicle_config=self.get_parameter('vehicle_config').value,
            reference_directory=self.get_parameter('reference_directory').value,
            calibration_config=self.get_parameter('config_file').value,
            stage=self.engine.config.stage,steering_source=self.engine.config.steering_source,
            trial_index=str(self.engine.index),trials=str(len(self.engine.trials)),
            planned_trials=str(len(self.engine.plan)),hold_sampling=self.engine.config.hold_sampling,
            sampled_seconds=str(self.engine.hold_elapsed),measurement_seconds=str(self.engine.config.measurement_seconds),
            segment_count=str(len(self.engine.hold_segments)),
            target_speed=str(self.engine.plan[self.engine.index] if self.engine.config.stage=='speed_hold'
                and self.engine.index<len(self.engine.plan) else ''),
            output_directory=str(self.output.directory),current_motor=str(self.obs.current_motor),
            voltage_input=str(self.obs.voltage_input),erpm=str(self.obs.erpm),velocity=str(self.obs.velocity),
            request_mode=str(self.last_request.mode),request_current=str(self.last_request.current),
            request_speed=str(self.last_request.speed),authority=str(self.obs.authority).lower(),
            logging_pending=str(self.output.pending_count),logging_dropped=str(self.output.dropped_count),
            logging_committed_trials=str(self.output.writer.trial_count),
            logging_committed_skips=str(self.output.writer.skipped_count),
            logging_error=str(self.output.error or ''))
        if self.engine.config.stage!='speed_hold':
            item=self.engine.trial_plan[self.engine.index] if self.engine.index<len(self.engine.trial_plan) else None
            values.update(plan_mode=self.get_parameter('plan_mode').value,trial_kind=item.kind if item else '',
                target_speed=str(item.start_speed) if item else '',target_end_speed=str(item.end_speed) if item else '',
                target_current=str(item.current) if item else '',skipped_trials=str(len(self.engine.skipped)),
                current_tracking_status=self.engine.tracking_status,
                current_tracking_evidence=json.dumps(self.engine.tracking_evidence,allow_nan=False),
                base_current_file=self.output.writer.metadata.get('base_current_file',''))
        status.values=[KeyValue(key=k,value=v) for k,v in values.items()]; msg.status=[status]
        return msg

    def _status(self): self.status_pub.publish(self.status_message())

    def destroy_node(self):
        if hasattr(self,'engine'):
            self.engine.enable(False)
            try: self.output.trials(self.engine.trials,self.evidence_error,skipped=self.engine.skipped)
            except RuntimeError as exc: self.get_logger().error(str(exc))
            finally: self.output.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args); node=None
    try:
        node=CalibrationNode(); rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException): pass
    finally:
        # launch may relay an interrupt already delivered to the process group.
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
