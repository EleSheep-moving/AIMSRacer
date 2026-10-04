"""Applied inputs and bounded kinematic prediction between measurement epochs."""
from collections import deque
from dataclasses import replace
import math


class AppliedHistory:
    def __init__(self,tau):
        self.tau=tau
        self.records=deque(maxlen=1024)

    def at(self,stamp):
        if not self.records or stamp < self.records[0][0]:
            return 0.,dict(acceleration=0.,steering=0.,steering_rate=0.)
        record=self.records[0]
        for candidate in reversed(self.records):
            if candidate[0]<=stamp:
                record=candidate;break
        time,estimate,target,speed,acceleration,rate=record
        estimate=target+(estimate-target)*math.exp(-max(0.,stamp-time)/self.tau)
        return estimate,dict(acceleration=acceleration,steering=target,steering_rate=rate)

    def record(self,stamp,steering,speed,acceleration,rate):
        if not all(math.isfinite(v) for v in (stamp,steering,speed,acceleration,rate)):
            raise ValueError('Nonfinite applied command')
        if self.records and stamp<self.records[-1][0]:
            raise ValueError('Applied command history moved backwards')
        estimate,_=self.at(stamp)
        self.records.append((stamp,estimate,steering,speed,acceleration,rate))

    def command_at(self,stamp):
        """Return a real selector output, never a proposed /drive command."""
        for record in reversed(self.records):
            if record[0] <= stamp:
                return dict(speed=record[3],steering=record[2],
                            acceleration=record[4],steering_rate=record[5])
        raise ValueError('Applied command history does not cover measurement epoch')

    def predict(self,state,start,end,config,known_until=None,future_command=None):
        """Replay actual ZOH inputs, then optionally forecast the old controller.

        The longitudinal bridge tracks the forwarded speed target with the
        configured acceleration bounds. It is a low-speed approximation, not
        an identified motor model. Steering uses the existing measured lumped
        time constant. No proposed input is inserted into real history.
        """
        if not all(math.isfinite(v) for v in (start,end)) or end < start:
            raise ValueError('Invalid prediction interval')
        known_until=end if known_until is None else known_until
        if not start <= known_until <= end:
            raise ValueError('Invalid known-input interval')
        command=self.command_at(start)
        actual=[r for r in self.records if start < r[0] <= known_until]
        index=0;stamp=start;value=state
        next_future=known_until
        while stamp < end-1e-10:
            while index<len(actual) and actual[index][0]<=stamp+1e-10:
                r=actual[index]
                command=dict(speed=r[3],steering=r[2],acceleration=r[4],steering_rate=r[5])
                index+=1
            if future_command is not None and stamp>=next_future-1e-10 and stamp>=known_until-1e-10:
                command=future_command(stamp,value)
                next_future=stamp+.02
            boundary=min(end,stamp+.005)
            if stamp < known_until-1e-10: boundary=min(boundary,known_until)
            if index<len(actual): boundary=min(boundary,actual[index][0])
            if future_command is not None and next_future>stamp+1e-10:
                boundary=min(boundary,next_future)
            dt=boundary-stamp
            value=self._step(value,command,dt,config)
            stamp=boundary
        return value,command

    def _step(self,state,command,dt,config):
        target=max(0.,min(config.max_speed,command['speed']))
        if target < config.minimum_drive_speed:
            target = 0.
        acceleration=max(-config.brake_limit,min(config.accel_limit,
                         (target-max(0.,state.speed))/dt))
        speed=max(0.,state.speed)
        target_steering=command['steering']
        def steering(t):
            return target_steering+(state.steering-target_steering)*math.exp(-t/self.tau)
        def rhs(t,yaw):
            v=max(0.,speed+acceleration*t)
            return (v*math.cos(yaw),v*math.sin(yaw),
                    v*math.tan(steering(t))/(config.wheelbase*(1+config.understeer_coefficient*v*v)))
        k1=rhs(0.,state.yaw)
        k2=rhs(dt/2,state.yaw+dt*k1[2]/2)
        k3=rhs(dt/2,state.yaw+dt*k2[2]/2)
        k4=rhs(dt,state.yaw+dt*k3[2])
        increments=[dt*(a+2*b+2*c+d)/6 for a,b,c,d in zip(k1,k2,k3,k4)]
        return replace(state,x=state.x+increments[0],y=state.y+increments[1],
                       yaw=state.yaw+increments[2],speed=max(0.,speed+dt*acceleration),
                       steering=steering(dt),timestamp=state.timestamp+dt)
