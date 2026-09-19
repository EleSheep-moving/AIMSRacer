"""Rear-point specific force and causal gyro differentiation/attitude propagation."""
from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
from .rear_axle_odometry import skew


def compensate_force(force, omega, alpha, lever, force_cov, omega_cov, alpha_cov):
    """All inputs in base axes; lever is rear -> IMU. Gravity stays in force.

    Covariance uses a conservative factor-two bound for the correlated gyro
    and gyro-derivative contributions; force/gyro correlation is not modelled.
    Mounting error and gyro bias require physical characterization.
    """
    f,w,a,r = (np.asarray(v,dtype=float) for v in (force,omega,alpha,lever))
    corrected = f-np.cross(a,r)-np.cross(w,np.cross(w,r))
    jw = 2*np.outer(r,w)-np.outer(w,r)-np.dot(w,r)*np.eye(3)
    ja = skew(r)
    covariance = np.asarray(force_cov)+2*(jw @ omega_cov @ jw.T+ja @ alpha_cov @ ja.T)
    return corrected, (covariance+covariance.T)/2


class AngularHistory:
    def __init__(self, window=.025, max_gap=.03):
        if not 0 < window <= .2 or not 0 < max_gap <= .2:
            raise ValueError('Invalid gyro window/gap')
        self.window,self.max_gap=window,max_gap
        self.samples=deque(maxlen=400)

    def add(self, stamp, omega, covariance):
        if not np.isfinite([stamp,*omega]).all() or not np.isfinite(covariance).all():
            raise ValueError('Nonfinite gyro input')
        reset=bool(self.samples and (stamp<=self.samples[-1][0] or stamp-self.samples[-1][0]>self.max_gap))
        if reset: self.samples.clear()
        self.samples.append((stamp,np.asarray(omega),np.asarray(covariance)))
        return reset

    def derivative(self):
        if not self.samples: return None
        now=self.samples[-1][0]
        samples=[s for s in self.samples if now-s[0]<=self.window+1e-9]
        if len(samples)<3: return None
        times=np.array([s[0]-now for s in samples]); times-=times.mean()
        denominator=times@times
        if denominator<1e-10: return None
        weights=times/denominator
        alpha=sum(w*s[1] for w,s in zip(weights,samples))
        covariance=sum(w*w*s[2] for w,s in zip(weights,samples))
        # Window smoothing can lag changing angular acceleration: explicit model floor.
        return alpha,covariance+np.eye(3)*.25

    def orientation(self, stamp, quaternion, target):
        """Propagate a LIO attitude with timestamped gyro, using trapezoidal integration."""
        if target<stamp or not self.samples or self.samples[0][0]>stamp or self.samples[-1][0]<target:
            return None
        times=np.array([s[0] for s in self.samples])
        rates=np.array([s[1] for s in self.samples])
        knots=[stamp]+[t for t in times if stamp<t<target]+([target] if target>stamp else [])
        rotation=Rotation.from_quat(quaternion)
        previous=stamp
        omega=np.array([np.interp(stamp,times,rates[:,i]) for i in range(3)])
        for t in knots[1:]:
            next_omega=np.array([np.interp(t,times,rates[:,i]) for i in range(3)])
            rotation=rotation*Rotation.from_rotvec(.5*(omega+next_omega)*(t-previous))
            previous,omega=t,next_omega
        return rotation.as_quat()
