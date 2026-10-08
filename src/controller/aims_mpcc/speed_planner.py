"""Periodic curvature speed preview with forward/backward acceleration limits."""
import math
import numpy as np


class SpeedPlanner:
    def __init__(self,path,config,spacing=.05):
        config.validate()
        if not math.isfinite(spacing) or spacing<=0:
            raise ValueError('Positive finite speed-profile spacing required')
        self.path,self.config=path,config
        count=max(32,math.ceil(path.length/spacing))
        self.positions=np.arange(count)*path.length/count
        curvature=np.array([abs(path.at(s)['curvature']) for s in self.positions])
        caps=np.sqrt(config.lateral_accel_limit/np.maximum(curvature,1e-9))
        speeds=np.minimum(caps,min(config.cruise_speed,config.max_speed))
        ds=path.length/count
        # The spline parameter is cumulative recorded chord distance. Integrate
        # its tangent norm so acceleration limits use physical arc length.
        tangent=np.array([np.linalg.norm(path.curve.numpy(s,1)) for s in self.positions])
        middle=np.array([np.linalg.norm(path.curve.numpy(s+ds/2,1)) for s in self.positions])
        self.arc_distances=ds*(tangent+4*middle+np.roll(tangent,-1))/6
        # Repeated periodic passes propagate the tightest corner through the
        # lap boundary without inventing a stop at the recorded start point.
        for _ in range(count):
            old=speeds.copy()
            for i in range(count):
                j=(i+1)%count
                speeds[j]=min(speeds[j],math.sqrt(speeds[i]**2+2*config.accel_limit*self.arc_distances[i]))
            for i in range(count-1,-1,-1):
                j=(i+1)%count
                speeds[i]=min(speeds[i],math.sqrt(speeds[j]**2+2*config.brake_limit*self.arc_distances[i]))
            if np.max(np.abs(old-speeds))<1e-9:
                break
        self.speeds=speeds

    def at(self,progress):
        return float(np.interp(progress%self.path.length,
            np.r_[self.positions,self.path.length],np.r_[self.speeds,self.speeds[0]]))

    def refs(self,progress,n,dt):
        result=[]
        for _ in range(n+1):
            speed=self.at(progress)
            result.append(speed)
            tangent=max(1e-9,float(np.linalg.norm(self.path.curve.numpy(progress,1))))
            midpoint=progress+.5*speed*dt/tangent
            midpoint_tangent=max(1e-9,float(np.linalg.norm(self.path.curve.numpy(midpoint,1))))
            progress+=self.at(midpoint)*dt/midpoint_tangent
        return result
