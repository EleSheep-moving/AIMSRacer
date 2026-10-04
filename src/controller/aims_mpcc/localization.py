"""Navigation authorization from final trusted anchors, independent of TF refresh."""
import math


class LocalizationHealth:
    def __init__(self, max_age=.5, heartbeat_timeout=.3):
        self.max_age=max_age
        self.heartbeat_timeout=heartbeat_timeout
        self.epoch=None
        self.sequence=-1
        self.health_sequence=-1
        self.loss_sequence=-1
        self.retired_epochs=set()
        self.stamp_ns=None
        self.received=None
        self.anchor_received=None
        self.ready=False
        self.reason='Localization health unavailable'

    def observe(self, values, ros_now_ns, monotonic_now):
        changed=False
        try:
            if values['protocol_version']!='1':
                raise ValueError('Unsupported localization health protocol')
            epoch=values['epoch']
            health_sequence=int(values['health_sequence'])
            sequence=int(values['anchor_sequence'])
            stamp=int(values['last_anchor_stamp_ns'])
            if not epoch or health_sequence<0 or sequence<0 or stamp<0 or values['ready'] not in ('true','false'):
                raise ValueError('Invalid localization health')
            if epoch in self.retired_epochs:
                return False
            if self.epoch is not None and epoch!=self.epoch:
                self.retired_epochs.add(self.epoch)
                changed=True
            if epoch!=self.epoch:
                self.sequence=-1;self.health_sequence=-1;self.loss_sequence=-1
                self.stamp_ns=None;self.anchor_received=None
                self.epoch=epoch
            if health_sequence<=self.health_sequence:
                return changed
            self.health_sequence=health_sequence
            if sequence<self.sequence:
                return changed
            if sequence==self.sequence and self.stamp_ns is not None and stamp!=self.stamp_ns:
                raise ValueError('Anchor changed without new sequence')
            if sequence>self.sequence:
                age=(ros_now_ns-stamp)*1e-9
                self.anchor_received=monotonic_now-max(0.,age)
            self.sequence=sequence;self.stamp_ns=stamp;self.received=monotonic_now
            if values['ready']=='false':
                self.loss_sequence=max(self.loss_sequence,sequence)
            self.ready=values['ready']=='true' and sequence>self.loss_sequence
            self.reason=values.get('state','Localization not ready')
        except (ValueError,TypeError,KeyError,OverflowError):
            self.ready=False;self.reason='Malformed localization health'
        return changed

    def usable(self, ros_now_ns, monotonic_now):
        if not self.ready or self.stamp_ns is None or self.anchor_received is None or self.received is None:
            return False
        ages=((ros_now_ns-self.stamp_ns)*1e-9,
              monotonic_now-self.anchor_received,monotonic_now-self.received)
        return all(math.isfinite(age) for age in ages) and (
            0<=ages[0]<=self.max_age and 0<=ages[1]<=self.max_age and
            0<=ages[2]<=self.heartbeat_timeout)
