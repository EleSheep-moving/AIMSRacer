"""Causal wheel-primary pulse exit and measured-current handoff helpers.

Response limits are empirical guard assumptions, never actuator guarantees.
No IMU/KF quantity replaces the wheel speed in this experiment's feedback.
"""
from collections import deque
from dataclasses import dataclass, asdict
import hashlib
import json
import math
from pathlib import Path


@dataclass(frozen=True)
class ProtectionSettings:
    exit_horizon: float = .14  # command opportunity + command/IQ release budget; add source age
    derivative_window: float = .04
    source_gap: float = .08
    iq_tolerance: float = 2.
    iq_confirm_samples: int = 2
    unload_timeout: float = .6
    hold_timeout: float = 6.
    quiet_acceleration: float = .5
    overspeed_margin: float = .2
    reverse_guard: float = .25
    reverse_guard_enabled: bool = True

    def __post_init__(self):
        for name,value in asdict(self).items():
            if name=='reverse_guard_enabled':
                if type(value) is not bool:raise ValueError('invalid protection '+name)
                continue
            if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
                raise ValueError('invalid protection '+name)
        if type(self.iq_confirm_samples) is not int or self.iq_confirm_samples<2:
            raise ValueError('IQ confirmation requires two or more fresh samples')
        if self.exit_horizon<.02 or self.source_gap>.2 or self.unload_timeout<=self.exit_horizon:
            raise ValueError('invalid protection timing budget')


class PulseProtection:
    def __init__(self,holding_current,response_limit,settings,full_range=False):
        self.holding_current=holding_current;self.response_limit=response_limit;self.settings=settings
        self.full_range=full_range
        self.wheel=deque();self.acceleration=None;self.instant_acceleration=None

    def reset(self):
        self.wheel.clear();self.acceleration=self.instant_acceleration=None

    def observe(self,stamp,velocity):
        if self.wheel and stamp<=self.wheel[-1][0]:return
        if self.wheel and stamp-self.wheel[-1][0]>self.settings.source_gap:self.reset()
        if self.wheel:
            self.instant_acceleration=(velocity-self.wheel[-1][1])/(stamp-self.wheel[-1][0])
        self.wheel.append((stamp,velocity))
        while len(self.wheel)>2 and self.wheel[1][0]<stamp-self.settings.derivative_window:self.wheel.popleft()
        if len(self.wheel)>1:
            t0,v0=self.wheel[0];self.acceleration=(velocity-v0)/(stamp-t0)

    def prediction(self,item,now,obs):
        bound=self.response_limit(item,obs.velocity) if self.full_range else self.response_limit(item)
        if isinstance(bound,bool) or not isinstance(bound,(int,float)) or not math.isfinite(bound) or bound<=0:
            raise ValueError('response_limit_unavailable')
        sign=1 if item.stage=='accel_interval' else -1
        rate=max([bound]+[sign*a for a in (self.acceleration,self.instant_acceleration) if a is not None])
        horizon=self.settings.exit_horizon+max(0.,now-obs.wheel_stamp)
        predicted=obs.velocity+sign*rate*horizon
        result=dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity,wheel_acceleration=self.acceleration,
            instantaneous_acceleration=self.instant_acceleration,response_limit=bound,directional_rate=rate,
            horizon_s=horizon,predicted_velocity=predicted,target_speed=item.end_speed,
            observer_acceleration=obs.acceleration,observer_quality=obs.acceleration_quality,
            exits=sign*(predicted-item.end_speed)>=0)
        if self.full_range and hasattr(self.response_limit,'provenance'):
            result['response_limit_provenance']=self.response_limit.provenance(item,obs.velocity)
        return result


def load_response_limits(path,baseline_sha256,raw=None,full_range=False):
    raw=Path(path).read_bytes() if raw is None else raw;value=json.loads(raw)
    if not isinstance(value,dict) or value.get('schema_version')!=1 or value.get('current_signal')!='avg_iq' or value.get('baseline_sha256')!=baseline_sha256:
        raise ValueError('response limits baseline/schema mismatch')
    rows=value.get('limits')
    if not isinstance(rows,list) or not rows:raise ValueError('missing response limits')
    for row in rows:
        if not isinstance(row,dict):raise ValueError('invalid response limit row')
        for key in ('start_speed','current','acceleration_bound'):
            v=row.get(key)
            if isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v):raise ValueError('invalid response limit')
        if row.get('stage') not in ('accel_interval','decel_current') or not 0<row['start_speed']<=5 or abs(row['current'])>100 or row['acceleration_bound']<=0:
            raise ValueError('invalid response limit bounds')
    def lookup(item,velocity=None):
        if full_range:return envelope(item,velocity)['acceleration_bound']
        candidates=[r['acceleration_bound'] for r in rows if r['stage']==item.stage and
            math.isclose(r['start_speed'],item.start_speed,abs_tol=1e-6) and math.isclose(r['current'],item.current,abs_tol=1e-4)]
        if not candidates:raise ValueError('response_limit_unavailable')
        return max(candidates)
    if full_range:
        digest=hashlib.sha256(raw).hexdigest()
        def envelope(item,velocity=None):
            """Require every traversed half-m/s cell; never infer lower-speed coverage.

            Schema 1 records the nominal start, not the speed range of its wheel
            samples. Assign only its adjacent directional cell, including old C
            braking/coast trials whose requested endpoint happened to be 1 m/s.
            A stronger signed current within 5 A is an empirical guard assumption,
            not a physical ceiling. Include *all* eligible bounds, never an average.
            """
            speeds=[item.start_speed,item.end_speed]
            if velocity is not None:speeds.append(velocity)
            if any(isinstance(v,bool) or not isinstance(v,(int,float)) or
                    not math.isfinite(v) or not 1.<=v<=5. for v in speeds):
                raise ValueError('response_limit_unavailable')
            low,high=min(speeds),max(speeds)
            cells=[]
            for index in range(8):
                left=1.+.5*index;right=left+.5
                if right<=low+1e-9 or left>=high-1e-9:continue
                selected=[]
                for row_index,r in enumerate(rows):
                    if r['stage']!=item.stage:continue
                    reference=r['current'];command=item.current
                    if item.stage=='accel_interval':
                        eligible=reference>0 and 0<=reference-command<=5.+1e-6
                        start=left
                    else:
                        same_sign=(reference>0 and command>0) or (reference<0 and command<0) or reference==command==0
                        eligible=same_sign and 0<=command-reference<=5.+1e-6
                        start=right
                    if eligible and math.isclose(r['start_speed'],start,rel_tol=0.,abs_tol=1e-6):
                        selected.append(dict(row_index=row_index,current=reference,
                            acceleration_bound=r['acceleration_bound']))
                if not selected:raise ValueError('response_limit_unavailable')
                cells.append(dict(speed_min=left,speed_max=right,references=selected,
                    acceleration_bound=max(r['acceleration_bound'] for r in selected)))
            if not cells:raise ValueError('response_limit_unavailable')
            return dict(method='whole_path_max_empirical_half_mps_cells',
                baseline_sha256=baseline_sha256,response_limits_sha256=digest,
                source_events_sha256=value.get('source_events_sha256'),
                stage=item.stage,current=item.current,speed_min=low,speed_max=high,
                speed_cells=cells,acceleration_bound=max(c['acceleration_bound'] for c in cells))
        lookup.provenance=envelope
    lookup.full_range=full_range
    return lookup,value,raw
