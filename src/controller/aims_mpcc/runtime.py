"""Clock-explicit command supervision; no ROS or optimizer dependencies."""
from dataclasses import dataclass, asdict
import math
from .config import REVERSE_SPEED_TOLERANCE


def clip(value, lower, upper):
    return max(lower, min(upper, value))


def angle_difference(a, b):
    return (a-b+math.pi) % (2*math.pi)-math.pi


def speed_envelope(distance, brake, jerk, latency):
    """Conservative braking envelope including jerk ramp and execution delay."""
    delay = brake/jerk + latency
    return brake*(math.sqrt(delay*delay+2*max(0., distance)/brake)-delay)


@dataclass(frozen=True)
class State:
    x: float
    y: float
    yaw: float
    speed: float
    steering: float
    timestamp: float  # ROS time, used only for measurement ordering/alignment.


@dataclass(frozen=True)
class Command:
    speed: float
    steering: float


class Supervisor:
    STATE_TIMEOUT = .10
    MODE_TIMEOUT = .10
    PLAN_TTL = .25

    def __init__(self, config, length):
        self.config, self.length = config, float(length)
        self.status, self.reason = 'READY', 'Waiting for fresh inputs'
        self.state = None
        self.state_received = self.mode_received = -math.inf
        self.mode = False
        self.generation = 0
        self.plan = None
        self.started = self.last_tick = None
        self.progress = self.wrapped_progress = 0.
        self.cross_track = self.heading_error = 0.
        self.last_command = Command(0., 0.)
        self.last_acceleration = 0.
        self.last_steering_rate = 0.
        self.stationary_since = None

    @property
    def active(self):
        return self.status in ('RUNNING', 'STOPPING')

    def fault(self, reason):
        if self.status != 'FAULT':
            self.generation += 1
        self.status, self.reason, self.plan = 'FAULT', str(reason), None
        self.last_command = Command(0., self.last_command.steering)
        self.last_acceleration = 0.
        self.last_steering_rate = 0.

    def observe(self, state, received, progress, cross_track, heading_error=0.):
        if not all(math.isfinite(v) for v in asdict(state).values()):
            self.fault('Nonfinite state'); return
        if self.state is not None:
            dt = state.timestamp-self.state.timestamp
            if dt < 0:
                self.fault('Measurement clock moved backwards')
            if self.active and (math.hypot(state.x-self.state.x, state.y-self.state.y) >
                                self.config.max_speed*max(dt, 0)+.15 or
                                abs(angle_difference(state.yaw, self.state.yaw)) > .5):
                self.fault('Localization discontinuity')
        if state.speed < -REVERSE_SPEED_TOLERANCE or state.speed > self.config.max_speed+.2:
            if self.active:
                self.fault('Measured speed outside operating range')
        if self.active:
            delta = (progress-self.wrapped_progress+self.length/2) % self.length-self.length/2
            self.progress += delta
        self.wrapped_progress = progress
        self.cross_track, self.heading_error = cross_track, heading_error
        self.state, self.state_received = state, received

    def set_mode(self, enabled, received):
        self.mode, self.mode_received = bool(enabled), received
        if self.active and not enabled:
            self.fault('Manual takeover or autonomy not selected')

    def fresh(self, now):
        return (self.state is not None and 0 <= now-self.state_received <= self.STATE_TIMEOUT
                and self.mode and 0 <= now-self.mode_received <= self.MODE_TIMEOUT)

    def start(self, now):
        if not self.fresh(now):
            raise ValueError('Fresh odometry and autonomous speed-mode selection required')
        if self.state.speed < -REVERSE_SPEED_TOLERANCE:
            raise ValueError('reverse motion unsupported')
        if abs(self.state.speed) > .1:
            raise ValueError('Start requires a stationary vehicle')
        if (min(self.wrapped_progress, self.length-self.wrapped_progress) > .2
                or abs(self.cross_track) > .15 or abs(self.heading_error) > math.radians(15)):
            raise ValueError('Start requires alignment within 0.2 m of the reference start')
        self.generation += 1
        self.status, self.reason, self.plan = 'RUNNING', '', None
        self.started = self.last_tick = now
        self.progress = self.wrapped_progress if self.wrapped_progress < self.length/2 else self.wrapped_progress-self.length
        self.stationary_since = None
        self.last_command = Command(0., self.state.steering)
        self.last_acceleration = 0.
        self.last_steering_rate = 0.

    def stop(self):
        if self.active:
            self.status, self.reason = 'STOPPING', 'Operator stop requested'

    def accept(self, result, now):
        if not self.active or result.get('generation') != self.generation:
            return False
        try:
            finite = all(math.isfinite(v) for row in result['states']+result['controls'] for v in row)
            valid = (result['success'] and finite and len(result['states']) >= 2
                     and len(result['controls']) == len(result['states'])-1
                     and all(len(row)==6 for row in result['states'])
                     and all(len(row)==3 for row in result['controls'])
                     and math.isfinite(result['constraint_violation'])
                     and result['constraint_violation'] < 1e-4
                     and 0 <= now-result['stamp'] <= self.PLAN_TTL)
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.fault('Invalid, failed, or expired solver result'); return False
        self.plan = result
        return True

    def refs(self, n, dt):
        if self.status == 'STOPPING':
            return [0.]*(n+1)
        remaining = max(0., self.length-self.progress)
        speed = max(0., self.state.speed)
        return [min(self.config.cruise_speed, speed_envelope(
            remaining-i*dt*speed, self.config.brake_limit,
            self.config.jerk_limit, self.PLAN_TTL)) for i in range(n+1)]

    def command(self, now):
        if not self.active:
            return Command(0., self.last_command.steering)
        if not self.fresh(now):
            self.fault('Odometry or mode status expired'); return self.last_command
        if self.last_tick is not None and (now < self.last_tick or now-self.last_tick > .10):
            self.fault('Command scheduling discontinuity'); return self.last_command
        dt = clip(now-(self.last_tick if self.last_tick is not None else now), 0., .05)
        self.last_tick = now
        if self.plan is None:
            if now-self.started > self.PLAN_TTL:
                self.fault('Initial plan deadline expired')
            return Command(0., self.last_command.steering)
        age = now-self.plan['stamp']
        if not 0 <= age <= self.PLAN_TTL:
            self.fault('Plan expired'); return self.last_command
        index = min(len(self.plan['controls'])-1, int(age/.1))
        fraction = clip((age-index*.1)/.1, 0., 1.)
        states, controls = self.plan['states'], self.plan['controls']
        target_speed = states[index][3]*(1-fraction)+states[index+1][3]*fraction
        previous_steer = self.plan.get('previous_steering', self.last_command.steering) if index==0 else controls[index-1][1]
        target_steer = previous_steer+(controls[index][1]-previous_steer)*fraction
        remaining = self.length-self.progress
        if self.status=='STOPPING' or remaining <= .05:
            target_speed = 0.
        # Enforce the finish envelope on executed targets as well as the OCP's
        # soft speed references. The command still passes through smooth limits.
        finish_speed = speed_envelope(remaining, self.config.brake_limit,
                                      self.config.jerk_limit, self.PLAN_TTL)
        target_speed = clip(target_speed, 0., min(self.config.max_speed,
                                                self.config.cruise_speed, finish_speed))
        if dt > 0:
            desired_accel = clip((target_speed-self.last_command.speed)/dt,
                                 -self.config.brake_limit, self.config.accel_limit)
            # Reserve speed change to ramp acceleration to zero before a hard
            # speed bound: a*dt + a*a/(2*jerk) <= remaining speed change.
            jerk = self.config.jerk_limit
            def safe_acceleration(distance):
                return math.sqrt((jerk*dt)**2+2*jerk*max(0., distance))-jerk*dt
            accel_lower = max(-self.config.brake_limit, self.last_acceleration-jerk*dt,
                              -safe_acceleration(self.last_command.speed))
            accel_upper = min(self.config.accel_limit, self.last_acceleration+jerk*dt,
                              safe_acceleration(self.config.max_speed-self.last_command.speed))
            if accel_lower > accel_upper+1e-10:
                self.fault('Acceleration cannot stop within speed limits')
                return self.last_command
            accel = clip(desired_accel, accel_lower, accel_upper)
            speed = clip(self.last_command.speed+accel*dt, 0., self.config.max_speed)
            accel = (speed-self.last_command.speed)/dt
            desired_rate = clip((target_steer-self.last_command.steering)/dt,
                                -self.config.steer_rate,self.config.steer_rate)
            # Reserve enough angular distance to brake the steering rate before
            # a hard angle limit. r*dt + r*r/(2*a) <= remaining angle.
            angular_accel = self.config.steer_acceleration
            def safe_rate(distance):
                return math.sqrt((angular_accel*dt)**2 +
                                 2*angular_accel*max(0., distance))-angular_accel*dt
            upper = min(self.config.steer_rate,
                        self.last_steering_rate+angular_accel*dt,
                        safe_rate(self.config.steer_limit-self.last_command.steering))
            lower = max(-self.config.steer_rate,
                        self.last_steering_rate-angular_accel*dt,
                        -safe_rate(self.config.steer_limit+self.last_command.steering))
            if lower > upper+1e-10:
                self.fault('Steering rate cannot stop within angle limits')
                return self.last_command
            rate = clip(desired_rate, lower, upper)
            steer = clip(self.last_command.steering+rate*dt,
                         -self.config.steer_limit, self.config.steer_limit)
            self.last_steering_rate = (steer-self.last_command.steering)/dt
            self.last_acceleration = accel
        else:
            speed, steer = self.last_command.speed, self.last_command.steering
        self.last_command = Command(speed, clip(steer, -self.config.steer_limit, self.config.steer_limit))
        finish = abs(remaining) <= .2 and self.progress >= self.length-.2
        if (finish or self.status=='STOPPING') and abs(self.state.speed)<.05 and speed<1e-6:
            if self.stationary_since is None:
                self.stationary_since = now
            if now-self.stationary_since >= .5:
                self.status = 'COMPLETE' if finish else 'READY'
                self.reason = 'One lap complete' if finish else 'Stopped'
                self.plan = None
                self.last_command = Command(0., self.last_command.steering)
        else:
            self.stationary_since = None
        if remaining < -.2:
            self.fault('Finish overshoot')
        return self.last_command
