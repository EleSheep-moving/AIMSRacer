"""Message-independent trusted anchor, source freshness and PP run ownership.

All clocks passed by the adapter are sampled inside its serialized callback.
ROS stamps identify physical source time; monotonic receipts detect frozen ROS
time. Alignment comes exclusively from the qualified localization record.
"""
import math

from .core import EnhancedPP, wrap_angle


def finite(*values):
    if not all(math.isfinite(v) for v in values):
        raise ValueError('finite inputs required')


class LocalizationHealth:
    """Protocol-v1 semantics matching aims_mpcc_rt/health.hpp."""
    def __init__(self):
        self.epoch = ''
        self.reason = 'Localization health unavailable'
        self.retired = set()
        self.sequence = self.health_sequence = self.loss_sequence = self.stamp_ns = -1
        self.anchor_received = self.received = float('nan')
        self.ready = self.qualified = self.committed = False
        self.alignment = (0., 0., 0.)
        self.transform = None

    def observe(self, values, ros_now_ns, steady_now):
        changed = False
        try:
            finite(ros_now_ns, steady_now)
            if values['protocol_version'] != '1':
                raise ValueError('protocol')
            epoch = values['epoch']
            hs, seq, stamp = (int(values[k]) for k in ['health_sequence', 'anchor_sequence', 'last_anchor_stamp_ns'])
            ready = values['ready']
            if not epoch or min(hs, seq, stamp) < 0 or ready not in ('true', 'false'):
                raise ValueError('health identity')
            if epoch in self.retired:
                return False
            if self.epoch and epoch != self.epoch:
                if len(self.retired)>=64:
                    raise ValueError('epoch history exhausted; restart required')
                self.retired.add(self.epoch)
                changed = True
            if epoch != self.epoch:
                self.epoch = epoch
                self.sequence = self.health_sequence = self.loss_sequence = self.stamp_ns = -1
                self.anchor_received = float('nan')
                self.ready = self.qualified = self.committed = False
            if hs <= self.health_sequence:
                return changed
            self.health_sequence = hs
            if seq < self.sequence:
                return changed
            if seq == self.sequence and self.stamp_ns >= 0 and stamp != self.stamp_ns:
                raise ValueError('anchor stamp changed without sequence')
            transform = None
            if values.get('alignment_valid') == 'true':
                if values['alignment_epoch'] != epoch or int(values['alignment_anchor_sequence']) != seq or int(values['alignment_stamp_ns']) != stamp:
                    raise ValueError('alignment identity mismatch')
                transform = tuple(float(values['map_odom_'+k]) for k in ['x','y','z','qx','qy','qz','qw'])
                finite(*transform)
                norm = math.sqrt(sum(q*q for q in transform[3:]))
                if abs(norm-1.)>.01:
                    raise ValueError('alignment quaternion')
                if seq == self.sequence and self.committed and transform != self.transform:
                    raise ValueError('alignment changed without sequence')
            if seq>self.sequence:
                self.committed = False
            self.qualified = transform is not None
            if transform is not None:
                self.committed = True
                self.transform = transform
                qx,qy,qz,qw = (q/norm for q in transform[3:])
                self.alignment = (transform[0], transform[1], math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz)))
            if seq>self.sequence:
                self.anchor_received = steady_now-max(0., (ros_now_ns-stamp)*1e-9)
            self.sequence, self.stamp_ns, self.received = seq, stamp, steady_now
            if ready == 'false':
                self.loss_sequence = max(self.loss_sequence, seq)
            self.ready = ready == 'true' and seq>self.loss_sequence
            self.reason = values.get('state', 'Localization not ready')
        except (ValueError, KeyError, TypeError, OverflowError):
            self.ready = self.qualified = False
            self.reason = 'Malformed localization health'
        return changed

    def usable(self, ros_now_ns, steady_now):
        ros_age = (ros_now_ns-self.stamp_ns)*1e-9
        anchor_age, heartbeat_age = steady_now-self.anchor_received, steady_now-self.received
        return self.ready and self.stamp_ns>=0 and all(math.isfinite(v) for v in [ros_age,anchor_age,heartbeat_age]) and 0<=ros_age<=1. and 0<=anchor_age<=1. and 0<=heartbeat_age<=.3


class TrackingInputs:
    """Reusable localization and actuator authority gate for PP/calibration."""
    def __init__(self, config, reference):
        self.config, self.reference = config.validate(), reference.validate_config(config)
        self.fault_generation = 0
        self.fault_reason = ''
        self.last_ros_ns = None
        self._clear()

    def _clear(self):
        self.health = LocalizationHealth()
        self.map_identity = ''
        self.map_valid = False
        self.authority = False
        self.authority_received = float('nan')
        self.command_source = self.command_received = float('nan')
        self.command_ros_ns = -1
        self.command_valid = False
        self.applied_steering = self.applied_speed = 0.
        self.pose = None
        self.pose_epoch = ''
        self.source = self.received = float('nan')
        self.source_ros_ns = -1

    def invalidate(self, reason):
        self.fault_generation += 1
        self.fault_reason = str(reason)

    def observe_clock(self, ros_now_ns):
        finite(ros_now_ns)
        if self.last_ros_ns is not None and ros_now_ns<self.last_ros_ns-1_000_000:
            self._clear()
            self.invalidate('ROS clock moved backwards; fresh inputs and explicit re-enable required')
        self.last_ros_ns = ros_now_ns

    def observe_map(self, hash_value):
        self.map_identity = hash_value
        if hash_value != self.reference.map_sha256:
            self.invalidate('Map identity mismatch')

    def observe_map_valid(self, valid):
        self.map_valid = valid is True
        if not self.map_valid:
            self.invalidate('Map validity withdrawn')

    def observe_health(self, values, ros_now_ns, steady_now):
        self.observe_clock(ros_now_ns)
        if values.get('map_sha256') != self.reference.map_sha256 or values.get('map_valid') != 'true':
            self.health.observe({}, ros_now_ns, steady_now)
            self.invalidate('Localization alignment map identity or validity mismatch')
            return False
        changed = self.health.observe(values, ros_now_ns, steady_now)
        if changed:
            self.pose = None
            self.invalidate('Localization epoch changed; explicit re-enable required')
        if not self.health.qualified or not self.health.usable(ros_now_ns, steady_now):
            self.invalidate('Trusted localization unavailable')
        return changed

    def observe_authority(self, enabled, steady_now):
        finite(steady_now)
        self.authority, self.authority_received = enabled is True, steady_now
        if not self.authority:
            self.invalidate('Autonomy speed authority withdrawn; explicit re-enable required')

    @staticmethod
    def _source_age(stamp_ns, ros_now_ns):
        finite(stamp_ns, ros_now_ns)
        age = (ros_now_ns-stamp_ns)*1e-9
        if stamp_ns<0 or not 0<=age<=.1:
            raise ValueError('stale or future source stamp')
        return age

    def observe_command(self, control_mode, steering, speed, stamp_ns, ros_now_ns, steady_now, frame_id='base_link', targets=None):
        self.observe_clock(ros_now_ns)
        try:
            finite(steering, speed, steady_now)
            age = self._source_age(stamp_ns, ros_now_ns)
            targets = targets if targets is not None else (speed, 0., 0., 0., 0.)
            if len(targets)!=5 or frame_id!='base_link' or control_mode!=0:
                raise ValueError('VESC speed authority withdrawn or command frame invalid')
            finite(*targets)
            if any(value != 0 for value in targets[1:]) or targets[0]!=speed or abs(steering)>self.config.steer_limit+1e-6 or speed<-.05 or speed>self.config.max_speed+1e-6:
                raise ValueError('invalid final VESC command')
            if stamp_ns<=self.command_ros_ns:
                return
            self.command_ros_ns = stamp_ns
            self.command_source, self.command_received = steady_now-age, steady_now
            self.command_valid = True
            self.applied_steering, self.applied_speed = steering, max(0.,speed)
        except (ValueError, TypeError):
            self.command_valid = False
            self.invalidate('Final VESC command invalid or authority withdrawn')
            raise

    def observe_odometry(self, x, y, yaw, velocity, stamp_ns, ros_now_ns, steady_now, frame_id='odom', child_frame_id='base_link'):
        self.observe_clock(ros_now_ns)
        try:
            finite(x,y,yaw,velocity,steady_now)
            age = self._source_age(stamp_ns, ros_now_ns)
            if frame_id!='odom' or child_frame_id!='base_link' or velocity<-.05 or velocity>self.config.max_speed+.5:
                raise ValueError('invalid odom/base_link forward state')
            if stamp_ns<=self.source_ros_ns:
                return
            if self.pose is not None:
                dt = (stamp_ns-self.source_ros_ns)*1e-9
                if math.hypot(x-self.pose[0], y-self.pose[1])>self.config.max_speed*dt+.15 or abs(wrap_angle(yaw-self.pose[2]))>.5:
                    raise ValueError('odometry discontinuity')
            self.pose = (x,y,yaw,max(0.,velocity))
            self.source, self.received, self.source_ros_ns = steady_now-age, steady_now, stamp_ns
            self.pose_epoch = self.health.epoch
        except (ValueError, TypeError):
            self.pose = None
            self.invalidate('Odometry invalid or discontinuous')
            raise

    def map_pose(self):
        if self.pose is None or not self.health.qualified or self.pose_epoch!=self.health.epoch:
            raise ValueError('qualified map state unavailable')
        x,y,yaw,v = self.pose
        x -= self.config.rear_offset*math.cos(yaw)
        y -= self.config.rear_offset*math.sin(yaw)
        ax,ay,angle = self.health.alignment
        c,s = math.cos(angle),math.sin(angle)
        return ax+c*x-s*y, ay+s*x+c*y, wrap_angle(yaw+angle), v

    def reason(self, ros_now_ns, steady_now, publisher_count=1):
        if publisher_count!=1:
            return 'Exclusive /drive publisher required'
        if not self.authority or not 0<=steady_now-self.authority_received<=.1:
            return 'Fresh autonomy speed authority required'
        reason = self.localization_reason(ros_now_ns, steady_now)
        if reason:
            return reason
        if not self.command_valid or not 0<=steady_now-self.command_source<=.1 or not 0<=steady_now-self.command_received<=.1 or not 0<=(ros_now_ns-self.command_ros_ns)*1e-9<=.1:
            return 'Fresh final VESC speed command required'
        return ''

    def localization_reason(self, ros_now_ns, steady_now):
        """Localization gate reusable by current-mode calibration owners."""
        if self.pose is None or not 0<=steady_now-self.source<=.1 or not 0<=steady_now-self.received<=.1 or not 0<=(ros_now_ns-self.source_ros_ns)*1e-9<=.1:
            return 'Fresh source odometry required'
        if self.map_identity != self.reference.map_sha256 or not self.map_valid:
            return 'Matching valid map required'
        if not self.health.qualified or not self.health.usable(ros_now_ns,steady_now) or self.pose_epoch!=self.health.epoch:
            return 'Fresh trusted qualified localization required'
        return ''


class PPTrackingRuntime:
    def __init__(self, inputs, reference, speed_limit=1., repeat_laps=False):
        finite(speed_limit)
        if speed_limit<=0 or type(repeat_laps) is not bool:
            raise ValueError('positive speed cap and boolean repeat_laps required')
        self.inputs, self.reference = inputs, reference
        self.pp = EnhancedPP(inputs.config)
        self.speed_limit = min(speed_limit, inputs.config.max_speed)
        self.repeat_laps = repeat_laps
        self.enabled = False
        self.phase, self.reason = 'DISABLED', 'Explicit enable required'
        self.progress = self.traveled = 0.
        self.last_tick = None
        self.generation = inputs.fault_generation

    def fault(self, reason):
        self.enabled = False
        self.phase, self.reason = 'FAULT', str(reason)

    def association(self):
        x,y,yaw,v = self.inputs.map_pose()
        s,lateral = self.reference.project((x,y))
        at = self.reference.at(s)
        if abs(wrap_angle(yaw-at['yaw']))>1.:
            raise ValueError('Forward reference heading mismatch')
        cfg = self.inputs.config
        if cfg.enforce_corridor:
            c,h = math.cos(yaw),math.sin(yaw)
            nx,ny = -math.sin(at['yaw']),math.cos(at['yaw'])
            for longitudinal in [-cfg.rear_extent,cfg.front_extent]:
                for side in [-cfg.half_width,cfg.half_width]:
                    cross = (x+c*longitudinal-h*side-at['x'])*nx + (y+h*longitudinal+c*side-at['y'])*ny
                    if cross>self.reference.left_width or cross<-self.reference.right_width:
                        raise ValueError('Vehicle footprint outside reference corridor')
        return (x,y,yaw,v),s,lateral

    def enable(self, ros_now_ns, steady_now, publisher_count=1):
        self.inputs.observe_clock(ros_now_ns)
        reason = self.inputs.reason(ros_now_ns,steady_now,publisher_count)
        if self.enabled:
            return False, 'Already enabled; disable before resetting lap association'
        if reason:
            return False, reason
        try:
            _, self.progress, _ = self.association()
        except ValueError as error:
            return False,str(error)
        self.traveled = 0.
        self.last_tick = steady_now
        self.generation = self.inputs.fault_generation
        self.enabled = True
        self.phase,self.reason = 'RUNNING','Enabled'
        return True,self.reason

    def disable(self):
        self.enabled = False
        self.phase,self.reason = 'DISABLED','Operator stop requested; explicit re-enable required'

    def tick(self, ros_now_ns, steady_now, publisher_count=1):
        self.inputs.observe_clock(ros_now_ns)
        output = dict(speed=0.,steering=0.,progress=self.progress,traveled=self.traveled)
        if not self.enabled:
            return output
        reason = self.inputs.reason(ros_now_ns,steady_now,publisher_count)
        if self.inputs.fault_generation != self.generation:
            reason = self.inputs.fault_reason or 'Input validity withdrawn; explicit re-enable required'
        if reason:
            self.fault(reason)
            return output
        try:
            pose,s,lateral = self.association()
            delta = (s-self.progress+self.reference.length/2)%self.reference.length-self.reference.length/2
            elapsed = steady_now-self.last_tick
            if elapsed<0 or elapsed>.1:
                raise ValueError('PP publication timer stale or clock reset')
            if delta<-.05 or abs(delta)>self.inputs.config.max_speed*elapsed+.15:
                raise ValueError('Reference progress discontinuity or reverse traversal')
            self.traveled += delta
            self.progress,self.last_tick = s,steady_now
            output.update(progress=s,traveled=self.traveled,lateral_error=lateral)
            if not self.repeat_laps and self.traveled>=self.reference.length-1e-6:
                self.enabled = False
                self.phase,self.reason = 'COMPLETE','One lap complete; explicit re-enable required'
                return output
            target = self.reference.at(s+self.pp.compute_lookahead(pose[3]))
            steering,debug = self.pp.compute_steering(pose[:2],pose[2],(target['x'],target['y']),target['yaw'],target['curvature'],pose[3])
            output.update(speed=min(self.reference.speed_at(s),self.speed_limit),steering=steering,lookahead=target,debug=debug)
        except (ValueError, TypeError, ArithmeticError) as error:
            self.fault(error)
        return output
