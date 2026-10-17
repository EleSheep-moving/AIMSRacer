"""Reference implementation: nine-state source-time velocity observer.

LIO twist, wheel speed, EKF and command values are deliberately absent from API.
Covariance describes a conditional effective model, not calibrated accuracy.
"""
from dataclasses import dataclass, field
import copy
import bisect
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Config:
    gravity: float = 9.80665
    position_std: float = .03
    bias_psd: float = 1e-4
    force_variance: float = .01
    tilt_std: float = .00872664626
    initial_bias_std: float = .2
    initialization_span: float = .2
    position_gate: float = 16.266
    angle_gate: float = .35
    max_gap: float = .03
    degraded_age: float = .3
    max_age: float = .5
    history: float = 2.
    max_events: int = 4096
    lever: np.ndarray = field(default_factory=lambda: np.array([.30,0.,.03]))
    mounting: np.ndarray = field(default_factory=lambda: np.eye(3))


def exp(v):
    return Rotation.from_rotvec(v).as_matrix()


class State:
    def __init__(self):
        self.previous = None
        self.R = np.eye(3)
        self.p = np.zeros(3); self.v = np.zeros(3); self.b = np.zeros(3)
        self.P = np.eye(9)
        self.oriented = False; self.initialized = False
        self.anchor = 0; self.init = []
        self.innovation = np.zeros(3); self.rejected = 0; self.reacquisitions = 0
        self.status = 'no_imu'


class Observer:
    def __init__(self, config=None):
        self.c = config or Config()
        for key in ['gravity','position_std','bias_psd','force_variance','tilt_std','initial_bias_std',
                    'initialization_span','position_gate','angle_gate','max_gap','degraded_age','max_age','history']:
            if not np.isfinite(getattr(self.c,key)) or getattr(self.c,key)<=0:raise ValueError(key)
        if self.c.history<self.c.max_age or self.c.degraded_age>self.c.max_age or self.c.max_events<8:
            raise ValueError('history/age limits')
        if np.shape(self.c.lever)!=(3,) or not np.isfinite(self.c.lever).all():raise ValueError('lever')
        R=np.asarray(self.c.mounting)
        if R.shape!=(3,3) or not np.isfinite(R).all() or not np.allclose(R.T@R,np.eye(3),atol=1e-9) or abs(np.linalg.det(R)-1)>1e-9:raise ValueError('mounting')
        self.imus=[]; self.states=[]; self.anchors={}; self.base=State(); self.s=State()
        self.replayed_imus=0

    @property
    def history_size(self):return len(self.imus)+len(self.anchors)

    def _old(self,t):
        return (self.base.previous is not None and t<=self.base.previous[0]) or (self.imus and (self.imus[-1][0]-t)*1e-9>self.c.history) or (not self.imus and self.anchors and (max(self.anchors)-t)*1e-9>self.c.history)

    def _propagate(self,s,dt,f,w):
        if dt<=0 or not s.oriented:return
        R=s.R@exp(w*dt*.5)
        acceleration=R@(f*self.c.gravity-s.b)+np.array([0.,0.,-self.c.gravity])
        s.p+=s.v*dt+.5*acceleration*dt*dt;s.v+=acceleration*dt
        s.R=s.R@exp(w*dt)
        if not s.initialized:return
        F=np.eye(9);F[:3,3:6]=np.eye(3)*dt;F[:3,6:]=-.5*dt*dt*R;F[3:6,6:]=-dt*R
        G=np.zeros((9,3));G[:3]=.5*dt*dt*R;G[3:6]=dt*R
        Q=(self.c.force_variance+(self.c.gravity*self.c.tilt_std)**2)*(G@G.T)
        bq=self.c.bias_psd
        Q[:3,:3]+=np.eye(3)*bq*dt**5/20;Q[:3,3:6]+=np.eye(3)*bq*dt**4/8
        Q[3:6,:3]=Q[:3,3:6].T;Q[3:6,3:6]+=np.eye(3)*bq*dt**3/3
        Q[:3,6:]=-R*bq*dt**3/6;Q[6:,:3]=Q[:3,6:].T
        Q[3:6,6:]=-R*bq*dt*dt/2;Q[6:,3:6]=Q[3:6,6:].T;Q[6:,6:]=np.eye(3)*bq*dt
        s.P=F@s.P@F.T+Q;s.P=(s.P+s.P.T)*.5

    def _lio(self,s,t,p,R):
        if s.initialized and (t-s.anchor)*1e-9>self.c.max_age+1e-12:
            previous=s.previous;rejected=s.rejected;count=s.reacquisitions+1
            s.__dict__.update(State().__dict__);s.previous=previous;s.rejected=rejected;s.reacquisitions=count
        if not s.oriented:
            s.R=R.copy();s.p=p.copy();s.v[:]=0;s.b[:]=0;s.oriented=True
        if not s.initialized:
            s.init.append((t,p-s.p))
            while len(s.init)>3 and (t-s.init[0][0])*1e-9>1.:s.init.pop(0)
            while len(s.init)>64:s.init.pop(1)
            if len(s.init)>=3 and (t-s.init[0][0])*1e-9+1e-12>=self.c.initialization_span:
                ts=np.array([(item[0]-s.init[0][0])*1e-9 for item in s.init]);ys=np.array([item[1] for item in s.init]);tc=ts-ts.mean()
                denominator=tc@tc;slope=tc@ys/denominator
                residual=ys-(ys.mean(axis=0)+tc[:,None]*slope)
                if np.max(np.linalg.norm(residual,axis=1))<=max(.10,5*self.c.position_std):
                    s.v+=slope;s.p=p.copy();s.initialized=True;s.init=[]
                    var=self.c.position_std**2;s.P=np.zeros((9,9));s.P[:3,:3]=np.eye(3)*var
                    s.P[3:6,3:6]=np.eye(3)*var/denominator
                    s.P[:3,3:6]=np.eye(3)*var*tc[-1]/denominator;s.P[3:6,:3]=s.P[:3,3:6].T
                    s.P[6:,6:]=np.eye(3)*self.c.initial_bias_std**2
            s.R=R.copy();s.anchor=t
            return
        innovation=p-s.p;S=s.P[:3,:3]+np.eye(3)*self.c.position_std**2
        angle=Rotation.from_matrix(s.R.T@R).magnitude()
        if innovation@np.linalg.solve(S,innovation)>self.c.position_gate or angle>self.c.angle_gate:
            s.rejected+=1;return
        s.innovation=innovation.copy();K=np.linalg.solve(S,s.P[:3,:]).T
        dx=K@innovation;s.p+=dx[:3];s.v+=dx[3:6];s.b+=dx[6:]
        A=np.eye(9);A[:,:3]-=K
        s.P=A@s.P@A.T+self.c.position_std**2*(K@K.T);s.P=(s.P+s.P.T)*.5
        s.R=R.copy();s.anchor=t

    def _advance(self,state,m):
        s=copy.deepcopy(state);t,f,w=m;previous=s.previous
        gap=previous is not None and (t-previous[0])*1e-9>self.c.max_gap+1e-12
        if gap:
            rejected=s.rejected;count=s.reacquisitions;s=State();s.rejected=rejected;s.reacquisitions=count;s.previous=m;s.status='imu_gap';return s
        start=previous[0] if previous is not None else t-1
        cursor=previous[0] if previous is not None else t
        def values(at):
            if previous is None:return f,w
            fraction=(at-previous[0])/(t-previous[0])
            return previous[1]+fraction*(f-previous[1]),previous[2]+fraction*(w-previous[2])
        def advance(until):
            nonlocal cursor
            fa,wa=values(cursor);fb,wb=values(until)
            self._propagate(s,(until-cursor)*1e-9,.5*(fa+fb),.5*(wa+wb));cursor=until
        for stamp,(position,R) in sorted(self.anchors.items()):
            if start<stamp<=t:
                advance(stamp);self._lio(s,stamp,position,R)
        advance(t);s.previous=m
        age=(t-s.anchor)*1e-9
        s.status='initializing' if not s.initialized else ('lio_stale' if age>self.c.max_age+1e-12 else ('degraded' if age>self.c.degraded_age+1e-12 else 'valid'))
        projected=self.c.mounting@s.R.T@s.v-np.cross(self.c.mounting@w,self.c.lever)
        row=self.c.mounting[0]@s.R.T
        variance=row@s.P[3:6,3:6]@row
        if not all(np.isfinite(x).all() for x in [s.p,s.v,s.b,s.P,s.R,projected]) or not np.isfinite(variance) or variance<0:raise ArithmeticError('nonfinite filter/output')
        return s

    def _replay(self,index):
        state=self.base if index==0 else self.states[index-1]
        changed=[]
        for m in self.imus[index:]:state=self._advance(state,m);changed.append(state)
        self.states[index:]=changed
        if self.states:self.s=self.states[-1]
        self.replayed_imus+=len(changed)

    def _trim(self):
        if not self.imus and self.anchors:
            newest=max(self.anchors)
            for t in sorted(self.anchors):
                if len(self.anchors)>self.c.max_events or (newest-t)*1e-9>self.c.history:
                    del self.anchors[t]
            return
        while len(self.imus)>1 and (self.history_size>self.c.max_events or (self.imus[-1][0]-self.imus[0][0])*1e-9>self.c.history):
            self.imus.pop(0);self.base=self.states.pop(0)
            self.anchors={t:a for t,a in self.anchors.items() if t>self.base.previous[0]}
        if len(self.imus)==1 and self.anchors:
            newest=max(self.anchors)
            if (newest-self.imus[0][0])*1e-9>self.c.history:
                self.anchors={t:a for t,a in self.anchors.items() if (newest-t)*1e-9<=self.c.history}
        while self.history_size>self.c.max_events:
            del self.anchors[min(self.anchors)]

    def add_imu(self,t,f,w):
        f=np.asarray(f,float);w=np.asarray(w,float)
        if t<=0 or f.shape!=(3,) or w.shape!=(3,) or not np.isfinite(f).all() or not np.isfinite(w).all() or self._old(t):return False
        index=bisect.bisect_left([m[0] for m in self.imus],t)
        if index<len(self.imus) and self.imus[index][0]==t:return False
        m=(t,f.copy(),w.copy())
        try:
            with np.errstate(over='raise',invalid='raise',divide='raise'):
                if index==len(self.imus):
                    s=self._advance(self.s,m);self.imus.append(m);self.states.append(s);self.s=s
                else:
                    self.imus.insert(index,m)
                    try:self._replay(index)
                    except (ArithmeticError,ValueError):self.imus.pop(index);raise
        except (ArithmeticError,ValueError):return False
        self._trim();return True

    def add_lio(self,t,p,q):
        p=np.asarray(p,float);q=np.asarray(q,float)
        if t<=0 or p.shape!=(3,) or q.shape!=(4,) or not np.isfinite(p).all() or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>1e-3 or self._old(t) or t in self.anchors:return False
        self.anchors[t]=(p.copy(),Rotation.from_quat(q).as_matrix())
        try:
            with np.errstate(over='raise',invalid='raise',divide='raise'):
                if self.imus and t<=self.imus[-1][0]:self._replay(bisect.bisect_left([m[0] for m in self.imus],t))
        except (ArithmeticError,ValueError):del self.anchors[t];return False
        self._trim();return True

    def latest(self):
        s=self.s;t=s.previous[0] if s.previous is not None else 0
        body=self.c.mounting@s.R.T@s.v
        omega=self.c.mounting@s.previous[2] if s.previous is not None else np.zeros(3)
        velocity=body-np.cross(omega,self.c.lever)
        row=self.c.mounting[0]@s.R.T
        variance=max(float(row@s.P[3:6,3:6]@row),1e-4)
        return dict(stamp_ns=t,valid=s.initialized and s.status in ['valid','degraded'],status=s.status,
                    anchor_stamp_ns=s.anchor,vx=float(velocity[0]),variance=variance,
                    position_world=s.p.copy(),velocity_world=s.v.copy(),effective_bias=s.b.copy(),
                    innovation=s.innovation.copy(),rejected_lio=s.rejected,lio_reinitializations=s.reacquisitions)
