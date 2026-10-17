"""Full-range constant-current acquisition flow, hosted by the shared Engine.

The Engine owns authority, clocks and source freshness. This mixin owns only
full-step trial progression; legacy interval and Stage A paths stay separate.
"""
from dataclasses import asdict
import math

from .steady import SteadySettings


def steady_settings(config, tracking):
    return SteadySettings(window_seconds=config.steady_window_seconds,
        confirm_seconds=config.steady_confirm_seconds,
        max_abs_acceleration=config.steady_max_abs_acceleration,
        max_speed_span=config.steady_max_speed_span,min_samples=config.steady_min_samples,
        current_settle_seconds=tracking.settle_seconds,current_window_seconds=tracking.window_seconds,
        current_min_samples=tracking.min_samples,current_absolute_tolerance=tracking.abs_tolerance,
        current_relative_tolerance=tracking.rel_tolerance)


class FullStepFlow:
    def _full_step_record(self, result, success, reason, last):
        if self.config.stage=='decel_current':
            result['start_trigger_observation']=self.start_trigger_observation
        normal=reason in ('speed_boundary','steady_state')
        observed=result['samples']>=2 and self.phase_ack_stamp is not None
        velocities=[r['velocity'] for r in self.active_samples]
        if last.get('velocity') is not None:velocities.append(last['velocity'])
        result.update(stop_reason=reason,data_status='complete' if success and normal and observed else
            ('partial' if result['samples'] else 'uncollected'),
            endpoint_reached=success and reason=='speed_boundary' and observed,
            interval_completed=success and reason=='speed_boundary' and observed,
            pulse_valid=success and normal and observed,steady_evidence=dict(self.steady_evidence),
            attempts_exhausted=False,wheel_speed_min=min(velocities) if velocities else None,
            wheel_speed_max=max(velocities) if velocities else None)

    def _full_step_tail(self, obs):
        row=self.trials[-1]
        previous=(row.get('wheel_speed_min'),row.get('wheel_speed_max'))
        if row.get('wheel_speed_min') is None:
            row['wheel_speed_min']=row['wheel_speed_max']=obs.velocity
        else:
            row['wheel_speed_min']=min(row['wheel_speed_min'],obs.velocity)
            row['wheel_speed_max']=max(row['wheel_speed_max'],obs.velocity)
        if previous!=(row['wheel_speed_min'],row['wheel_speed_max']):self.results_revision+=1

    def _full_step_reject(self, now, obs, reason):
        self._finish(False,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
        self.trials[-1].update(attempts_exhausted=True,preflight_rejected=True)
        self._advance_current(reason)
        return self._next_current_request(now,obs)

    def _full_step_stop(self, now, obs, reason, response_verified=False):
        normal=reason in ('speed_boundary','steady_state')
        observed=response_verified and len(self.active_samples)>=2 and self.phase_ack_stamp is not None
        target=obs.velocity if reason=='steady_state' else self._item().end_speed
        if not normal:target=self.config.recovery_speed
        holding_error=''
        try:current=self.protection.holding_current(target) if normal else 0.
        except (ValueError,KeyError) as exc:
            # A valid measured steady speed may lie outside Stage A's baseline
            # domain. Do not extrapolate that curve or leave the pulse active.
            holding_error=str(exc);target=self.config.recovery_speed;current=0.
        if isinstance(current,bool) or not isinstance(current,(int,float)) or not math.isfinite(current) or abs(current)>self.config.max_current:
            self.fault('invalid_holding_current');return self._request(now,obs,1)
        self._finish(normal and observed,reason,dict(wheel_stamp=obs.wheel_stamp,velocity=obs.velocity))
        retry=reason in ('trial_timeout','straight_headroom','straight_region_exit')
        exhausted=not retry or self.attempt_number>=self.tracking_config.max_attempts
        self.trials[-1]['attempts_exhausted']=exhausted
        if retry and not exhausted:
            self.retry_region=self.region;self.retry_left=False
        self.handoff=dict(started=now,target_speed=target,target_current=current,
            advance=exhausted,turn=not normal or bool(holding_error),ack_stamp=None,last_iq_stamp=None,iq_samples=0,
            quiet_since=None,last_hold_wheel=None,hold_confirmed=False,reason=reason)
        if holding_error:self.handoff.update(holding_current_unavailable=True,holding_current_error=holding_error)
        self.trials[-1]['handoff']=dict(self.handoff)
        self.started=None;self.active_samples=[]
        self.state='UNLOAD';self.reason=reason
        return self._request(now,obs,1,current=current)

    def _full_step_distance(self, obs):
        # A spatial region margin is independent of the measured speed endpoint.
        # No fixed response-delay budget is used to shorten the current plateau.
        return math.isfinite(obs.straight_remaining_m) and obs.straight_remaining_m>self.config.straight_exit_margin_m

    def _full_step_tick(self, now, obs, echo, pair, unique):
        c=self.config
        if self.state in ('UNLOAD','HOLD'):self._full_step_tail(obs)
        if c.stage=='decel_current' and self.protection.settings.reverse_guard_enabled and self.state in ('APPLY','MEASURE','UNLOAD','HOLD') and obs.velocity<=self.protection.settings.reverse_guard:
            self.fault('near_reverse');return self._request(now,obs,1)
        if self.state in ('UNLOAD','HOLD'):
            return self._handoff_tick(now,obs,echo,pair)
        if self.state=='RECOVER':
            if not obs.straight or obs.region!=self.retry_region:self.retry_left=True
            if self.retry_region is not None and not (self.retry_left and obs.straight):
                return self._request(now,obs,speed=c.recovery_speed)
            self.retry_region=None;self.retry_left=False;self.state='PREPARE'
        item=self._item()
        if self.state=='PREPARE':
            if not obs.straight:
                self.stable_since=None
                self.last_prepare_wheel=None
                return self._request(now,obs,speed=c.recovery_speed)
            self.region=obs.region
            start_ready=False
            if c.stage=='decel_current':
                fresh_wheel=self.last_prepare_wheel is None or obs.wheel_stamp>self.last_prepare_wheel
                if fresh_wheel:self.last_prepare_wheel=obs.wheel_stamp
                # Start braking on the first fresh wheel sample at/above the
                # target. Do not spend the straight waiting for a speed plateau
                # or for a sampled overshoot to return into the baseline domain.
                start_ready=(fresh_wheel and obs.velocity>=item.start_speed and
                    echo.mode==0 and math.isclose(echo.speed,item.start_speed,abs_tol=1e-5))
            elif unique:
                self.last_pair=pair
                if self.last_prepare_wheel is not None and obs.wheel_stamp-self.last_prepare_wheel>self.protection.settings.source_gap:
                    self.stable_since=None
                self.last_prepare_wheel=obs.wheel_stamp
                stable=(abs(obs.velocity-item.start_speed)<=c.stable_tolerance and
                    self.protection.acceleration is not None and
                    abs(self.protection.acceleration)<=self.protection.settings.quiet_acceleration and
                    echo.mode==0 and math.isclose(echo.speed,item.start_speed,abs_tol=1e-5))
                if stable:
                    if self.stable_since is None:self.stable_since=obs.wheel_stamp
                else:self.stable_since=None
                # Tolerance may include speeds outside the measured domain. Keep
                # preparing there; emit a step only on an in-domain fresh sample.
                start_ready=(self.stable_since is not None and
                        obs.wheel_stamp-self.stable_since>=c.stable_seconds and
                        min(item.start_speed,item.end_speed)<=obs.velocity<=max(item.start_speed,item.end_speed))
            if start_ready:
                self.state='APPLY';self.phase_started=obs.wheel_stamp;self._begin_attempt()
                if not math.isfinite(obs.straight_remaining_m):
                    return self._full_step_reject(now,obs,'straight_distance_unavailable')
                if not self._full_step_distance(obs):
                    # No pulse yet: wait for another straight, without
                    # consuming an acquisition attempt or rejecting a current.
                    self.attempt_counts[self.index]-=1
                    self._reset_trial();self.retry_region=obs.region;self.retry_left=False
                    self.state='RECOVER'
                    return self._request(now,obs,speed=c.recovery_speed)
                if c.stage=='decel_current':
                    self.start_trigger_observation=dict(wheel_stamp=obs.wheel_stamp,
                        telemetry_stamp=obs.telemetry_stamp,velocity=obs.velocity,request_stamp=now)
                return self._request(now,obs,1,current=item.current)
            return self._request(now,obs,speed=item.start_speed)
        if self.state not in ('APPLY','MEASURE'):
            self.fault('invalid_full_step_state');return self._request(now,obs,1)
        matching=(echo.mode==1 and math.isclose(echo.current,item.current,abs_tol=1e-5) and echo.speed==0 and
            self.first_current_request_stamp is not None and echo.stamp>=self.first_current_request_stamp)
        if matching and self.phase_ack_stamp is None:self.phase_ack_stamp=echo.stamp
        acknowledged=self.phase_ack_stamp is not None and min(pair)>=self.phase_ack_stamp
        verified=unique and matching and acknowledged
        if unique and matching and acknowledged:
            self.last_pair=pair
            row=asdict(obs);row.pop('final_request')
            row.update(trial_id=len(self.trials)+1,plan_index=self.index,stage=c.stage,
                requested_mode=1,requested_speed=0.,requested_current=item.current,
                attempt_id=self.attempt_id,attempt_number=self.attempt_number,repeat=item.repeat,kind=item.kind)
            if self.starting_wheel_observation is None:
                self.starting_wheel_observation=dict(wheel_stamp=obs.wheel_stamp,telemetry_stamp=obs.telemetry_stamp,velocity=obs.velocity)
            self.samples.append(row);self.active_samples.append(row)
            if self.started is None:self.started=obs.wheel_stamp;self.start_velocity=obs.velocity
            self.state='MEASURE'
        self.steady_evidence=self.steady.observe(obs.wheel_stamp,obs.velocity,obs.telemetry_stamp,
            obs.avg_iq,item.current,self.phase_ack_stamp,current_mode=matching and acknowledged)
        boundary=obs.velocity>=item.end_speed if c.stage=='accel_interval' else obs.velocity<=item.end_speed
        if not obs.straight or obs.region!=self.region:
            return self._full_step_stop(now,obs,'straight_region_exit')
        if boundary:return self._full_step_stop(now,obs,'speed_boundary',verified)
        if not self._full_step_distance(obs):
            return self._full_step_stop(now,obs,'straight_headroom')
        if self.first_current_request_stamp is not None and now-self.first_current_request_stamp>=c.trial_timeout:
            return self._full_step_stop(now,obs,'trial_timeout')
        # Use a symmetric actual-Iq error window in full-step mode, including
        # coast. Legacy C magnitude/group failure rules do not apply here.
        q=self.steady_evidence;first=q.get('current_window_start_stamp')
        if (first is not None and q['current_sample_count']>=self.tracking_config.min_samples and
                obs.telemetry_stamp-first+2*math.ulp(obs.telemetry_stamp)>=self.tracking_config.window_seconds):
            fraction=q['current_good_fraction']
            self.tracking_evidence=dict(q)
            self.tracking_status='verified' if fraction>=.9 else 'unverified'
            if 1.-fraction+1e-12>=self.tracking_config.failure_fraction:
                self.tracking_status='outside_tolerance'
                return self._full_step_stop(now,obs,'current_tracking_not_achieved')
        if q['steady']:return self._full_step_stop(now,obs,'steady_state',verified)
        return self._request(now,obs,1,current=item.current)
