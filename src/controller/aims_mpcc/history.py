"""Timestamped applied-command history for source-time steering estimation."""
from collections import deque
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
