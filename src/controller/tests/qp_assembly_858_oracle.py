"""Verbatim frozen858 dense assembler as a differential oracle.

qp_backend.py SHA256: 40738db34b89029b844e43ef7f7356578b3b45a1d981a45b7c361720c6958ce1 .
Do not modify this oracle when changing the production assembler.
"""
import numpy as np
from aims_mpcc.envelope import jerk_limits
from aims_mpcc.vendor.normalized_cost import RATIOS
ELLIPSE_FACETS=tuple((np.cos(angle),np.sin(angle)) for angle in np.arange(16)*2*np.pi/16)

def original_assemble(self,initial,applied,refs,alignment,seed):
        cfg=self.config;D=self.dimension;P=np.eye(D)*1e-10;q=np.zeros(D)
        rows=[];structures=[];low=[];high=[];P_structure=np.eye(D,dtype=bool)
        def constraint(values,lo,hi):
            row=np.zeros(D)
            for i,v in values.items():row[i]+=v
            structure=np.zeros(D,dtype=bool);structure[list(values)]=True
            rows.append(row);structures.append(structure);low.append(lo);high.append(hi)
        def cost(values,target,weight):
            # Each objective term involves at most three variables. Preserve
            # its accumulation order while updating only those active entries.
            for i,vi in values.items():
                q[i]-=2*weight*target*vi
                for j,vj in values.items():
                    P[i,j]+=2*weight*(vi*vj)
                    P_structure[i,j]=True
        state=lambda k,j:4*k+j
        control=lambda k,j:4*(self.n+1)+2*k+j
        geometries=self.geometries(seed[0][:,4],alignment)
        g=geometries[0];normal=np.array([-np.sin(g[2]),np.cos(g[2])])
        lateral=float(np.dot(initial[:2]-g[:2],normal));heading=(initial[2]-g[2]+np.pi)%(2*np.pi)-np.pi
        for j,value in enumerate((lateral,heading,initial[5],initial[3])):
            constraint({state(0,j):1.},value,value)
        for k in range(self.n):
            coeff,bias=self._transition(float(seed[0][k,3]),geometries[k][3]);coeff=np.asarray(coeff);bias=np.asarray(bias).ravel()
            for j in range(3):
                equation={state(k+1,j):1.,control(k,1):-coeff[j,3]}
                for m in range(3):equation[state(k,m)]=-coeff[j,m]
                if k:equation[control(k-1,1)]=-coeff[j,4];offset=bias[j]
                else:offset=bias[j]+coeff[j,4]*applied[1]
                constraint(equation,offset,offset)
            constraint({state(k+1,3):1.,state(k,3):-1.,control(k,0):-self.dt},0.,0.)
        offsets=cfg.longitudinal_offsets() if cfg.enforce_corridor else ()
        for k in range(self.n+1):
            terminal=cfg.terminal_weight if k==self.n else 1.
            cost({state(k,0):1.},0.,terminal*cfg.contour_weight/cfg.contour_scale**2)
            cost({state(k,1):1.},0.,terminal*cfg.heading_weight/cfg.heading_scale**2)
            cost({state(k,3):1.},refs[k],terminal*cfg.speed_weight/cfg.speed_scale**2)
            constraint({state(k,3):1.},0.,cfg.max_speed)
            constraint({state(k,2):1.},-cfg.steer_limit,cfg.steer_limit)
            if cfg.enforce_corridor:
                for along in offsets:
                    constraint({state(k,0):1.,state(k,1):along},-self.path.right_width+cfg.half_width,self.path.left_width-cfg.half_width)
        jerk=jerk_limits(initial,applied,cfg,self.dt,self.n)
        accel_axis=min(cfg.envelope_halfaxes()[:2]);lateral_axis=cfg.envelope_halfaxes()[2]
        envelope_radius=np.cos(np.pi/16)*np.sqrt(1.-cfg.optimization_envelope_margin)
        for k in range(self.n):
            accel,steer=control(k,0),control(k,1)
            constraint({accel:1.},-cfg.brake_limit,cfg.accel_limit)
            constraint({steer:1.},-cfg.steer_limit,cfg.steer_limit)
            delta={steer:1.}
            if k:delta[control(k-1,1)]=-1.;previous=0.
            else:previous=applied[1]
            constraint(delta,previous-cfg.steer_rate*self.dt,previous+cfg.steer_rate*self.dt)
            acceleration={accel:1.}
            if k:acceleration[control(k-1,0)]=-1.;pa=0.
            else:pa=applied[0]
            constraint(acceleration,pa-jerk[k]*self.dt,pa+jerk[k]*self.dt)
            second=dict(delta);bound_offset=0.
            if k==0:bound_offset=applied[1]+applied[2]*self.dt
            elif k==1:
                second[control(k-1,1)]=-2.;bound_offset=-applied[1]
            else:
                second[control(k-1,1)]=-2.;second[control(k-2,1)]=1.
            constraint(second,bound_offset-cfg.steer_acceleration*self.dt**2,bound_offset+cfg.steer_acceleration*self.dt**2)
            ff=np.arctan(cfg.wheelbase*(1+cfg.understeer_coefficient*seed[0][k,3]**2)*geometries[k][3])
            cost({steer:1.},ff,cfg.steering_weight/cfg.steering_scale**2)
            cost({accel:1.},0.,RATIOS['accel']/cfg.acceleration_scale**2)
            cost(delta,previous,cfg.steering_rate_weight/(self.dt*cfg.steer_rate)**2)
            cost(second,bound_offset,cfg.steering_acceleration_weight/(cfg.steer_acceleration*self.dt**2)**2)
            # Inscribed 16-sided ellipse approximation; exact nonlinear
            # utilization is checked again on the independent output rollout.
            for node in (k,k+1):
                v=float(seed[0][node,3]);d=float(seed[0][node,5])
                factor=v*v/(cfg.wheelbase*(1+cfg.understeer_coefficient*v*v)*lateral_axis)
                slope=factor/(np.cos(d)**2);offset=factor*np.tan(d)-slope*d
                for a,b in ELLIPSE_FACETS:
                    constraint({accel:a/accel_axis,state(node,2):b*slope},-np.inf,envelope_radius-b*offset)
        self._p_structure=P_structure;self._a_structure=np.asarray(structures)
        return P,q,np.asarray(rows),np.asarray(low),np.asarray(high)
