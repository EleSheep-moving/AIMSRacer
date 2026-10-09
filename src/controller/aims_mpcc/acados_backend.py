"""Generated C discrete MPCC with acados SQP_RTI and HPIPM.

Nine internal states retain the previous acceleration, steering endpoint and
steering ramp rate. Geometry is frozen from shifted progress at each request;
physical motion always remains in odom. Preparation is exclusively offline.
"""
from dataclasses import asdict
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import tempfile
import casadi as ca
import numpy as np
from .backend_models import NumericalBackend,interval_symbolic
from .envelope import jerk_limits
from .vendor.normalized_cost import RATIOS


def _dependency_versions():
    versions={'python':platform.python_version(),'casadi':ca.__version__,'numpy':np.__version__}
    try:versions['acados_template']=importlib.metadata.version('acados_template')
    except importlib.metadata.PackageNotFoundError:versions['acados_template']='unavailable'
    source=os.environ.get('ACADOS_SOURCE_DIR')
    if source is None:
        try:
            import acados_template
            source=str(Path(acados_template.__file__).resolve().parents[3])
        except ImportError:source=None
    versions['acados_source']=None
    if source:
        try:versions['acados_source']=subprocess.check_output(['git','-C',source,'rev-parse','HEAD'],text=True,stderr=subprocess.DEVNULL).strip()
        except (subprocess.CalledProcessError,FileNotFoundError):versions['acados_source']='unknown'
        libroot=Path(source)/'install-x86'/'lib'
        if not libroot.exists():libroot=Path(source)/'lib'
        versions['acados_lib_path']=str(libroot.resolve())
        versions['libraries']={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in (libroot/'libacados.so',libroot/'libhpipm.so',libroot/'libblasfeo.so') if p.is_file()}
    return versions


def artifact_fingerprint(path,config,horizon,dt):
    root=Path(__file__).parent
    source_files=['acados_backend.py','backend_models.py','config.py','envelope.py','path.py',
                  'rollout_native.py','kernels/rollout.c','solver_diagnostics.py',
                  'vendor/track.py','vendor/global_kinematic_model.py','vendor/normalized_cost.py']
    payload=dict(schema_version=1,backend='acados_sqp_rti_hpipm',config=asdict(config),
        horizon=int(horizon),dt=float(dt),path=dict(points=path.points.tolist(),left_width=path.left_width,
        right_width=path.right_width,frame_id=path.frame_id,metadata=path.metadata),
        platform=dict(system=platform.system(),machine=platform.machine(),python_cache_tag=sys.implementation.cache_tag),
        dependencies=_dependency_versions(),sources={f:hashlib.sha256((root/f).read_bytes()).hexdigest()
                                                    for f in source_files if (root/f).is_file()})
    fingerprint=hashlib.sha256(json.dumps(payload,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()
    return fingerprint,payload


class AcadosSolver(NumericalBackend):
    def __init__(self,path,config,horizon=10,dt=.1,prepare=False,artifact_directory=None):
        super().__init__(path,config,horizon,dt)
        config.require_legacy_command_profile()
        fingerprint,payload=artifact_fingerprint(path,config,horizon,dt)
        root=Path(artifact_directory or os.environ.get('AIMS_MPCC_SOLVER_DIR',Path.home()/'.cache'/'aims_mpcc'/'solvers'))
        self.artifact_directory=root.expanduser().resolve()/'acados'/fingerprint
        self._expected_lib_path=payload['dependencies'].get('acados_lib_path')
        self._verified_json_bytes=None
        self.fingerprint=fingerprint;self._manifest=self.artifact_directory/'manifest.json'
        self.json_file=self.artifact_directory/'ocp.json'
        self.model_name='aims_'+fingerprint[:16]
        self.shared_library=self.artifact_directory/'c_generated_code'/f'libacados_ocp_solver_{self.model_name}.so'
        if not prepare and not self._artifact_ready(payload):
            raise FileNotFoundError(f'acados artifact missing or mismatched; prepare offline first: {self.artifact_directory}')
        try:from acados_template import AcadosOcpSolver
        except ImportError as exc:raise RuntimeError('acados native backend requires acados_template and installed acados/HPIPM libraries') from exc
        ocp=self._build_ocp()
        self._ocp=ocp
        ocp.code_export_directory=str(self.shared_library.parent)
        if prepare and not self._artifact_ready(payload):
            self.artifact_directory.mkdir(parents=True,exist_ok=True)
            AcadosOcpSolver.generate(ocp,json_file=str(self.json_file),verbose=False)
            AcadosOcpSolver.build(ocp.code_export_directory,verbose=False)
            self._manifest.write_text(json.dumps(dict(fingerprint=fingerprint,payload=payload,
                shared_library_sha256=hashlib.sha256(self.shared_library.read_bytes()).hexdigest(),
                ocp_json_sha256=hashlib.sha256(self.json_file.read_bytes()).hexdigest()),sort_keys=True,indent=2)+'\n')
            if not self._artifact_ready(payload):
                raise RuntimeError('Prepared acados artifact failed manifest or loader metadata validation')
        # Both generate and build are false online. This call only opens the
        # prepared shared library and allocates the native solver workspace.
        try:
            # Load the exact verified JSON snapshot rather than reopening the
            # mutable cache metadata after its hash and routing checks.
            with tempfile.TemporaryDirectory(prefix='aims-acados-loader-') as directory:
                verified_json=Path(directory)/'ocp.json'
                verified_json.write_bytes(self._verified_json_bytes)
                # SDK 0.5.5 can rewrite generate/build=True after its own reuse
                # comparison. Our verified hashes/routes are authoritative;
                # forbid that automatic online generation/rebuild branch.
                self._native=AcadosOcpSolver(ocp,json_file=str(verified_json),generate=False,build=False,
                    check_reuse_possible=False,verbose=False)
        except OSError as exc:
            raise RuntimeError('Cannot load prepared acados artifact; check ACADOS_SOURCE_DIR and LD_LIBRARY_PATH for acados/HPIPM/BLASFEO libraries') from exc

    def _artifact_ready(self,payload):
        self._verified_json_bytes=None
        if not self._manifest.is_file() or not self.json_file.is_file() or not self.shared_library.is_file():return False
        try:
            manifest=json.loads(self._manifest.read_text());json_bytes=self.json_file.read_bytes()
            if not (manifest['fingerprint']==self.fingerprint and manifest['payload']==payload and
                    manifest['ocp_json_sha256']==hashlib.sha256(json_bytes).hexdigest() and
                    manifest['shared_library_sha256']==hashlib.sha256(self.shared_library.read_bytes()).hexdigest()):
                return False
            document=json.loads(json_bytes)
            if not self._loader_metadata_matches(document):return False
            self._verified_json_bytes=json_bytes
            return True
        except (ValueError,KeyError,OSError,TypeError,AttributeError):return False

    def _loader_metadata_matches(self,document):
        # Older SDKs place loader routes at the JSON root; pinned 0.5.5 places
        # them under code_gen_options and loads from the supplied OCP object.
        # Validate the generated metadata against the object routes we set,
        # while retaining the binary and manifest hashes above.
        dims=document['dims'];options=document['solver_options']
        routes=document.get('code_gen_options',document)
        route_keys=('shared_lib_ext','code_export_directory','acados_lib_path','acados_include_path')
        if any(key not in routes for key in route_keys):return False
        if routes is not document and any(key in document and document[key]!=routes[key] for key in route_keys):
            return False
        if self._expected_lib_path is None:return False
        library_directory=Path(self._expected_lib_path)
        sample_count=round(self.dt/.02)+1
        corner_probes=len({0,(sample_count+1)//2,sample_count-1})
        nonlinear_rows=(1 if self.config.command_profile=='rate_bounded_v2' else 3)+sample_count+(4*corner_probes if self.config.enforce_corridor else 0)+int(self.config.envelope_soft_enabled)
        return bool(
            document['name']==self.model_name and document['model']['name']==self.model_name and
            document['problem_class']=='OCP' and routes['shared_lib_ext']=='.so' and
            Path(routes['code_export_directory']).resolve()==self.shared_library.parent.resolve() and
            Path(routes['acados_lib_path']).resolve()==library_directory.resolve() and
            Path(routes['acados_include_path']).resolve()==(library_directory.parent/'include').resolve() and
            type(dims['N']) is int and dims['N']==self.n and dims['nx']==9 and dims['np']==10 and
            dims['nu']==(4 if self.config.envelope_soft_enabled else 3) and
            dims['nh_0']==nonlinear_rows and dims['nh']==nonlinear_rows and
            dims['nh_e']==1+(4 if self.config.enforce_corridor else 0) and dims['nbx_0']==9 and
            dims['nbxe_0']==9 and document['constraints']['has_x0'] is True and
            type(options['N_horizon']) is int and options['N_horizon']==self.n and
            options['nlp_solver_type']=='SQP_RTI' and options['qp_solver']=='PARTIAL_CONDENSING_HPIPM' and
            options['integrator_type']=='DISCRETE' and
            np.isclose(float(options['tf']),self.n*self.dt,rtol=0.,atol=1e-12) and
            np.asarray(options['time_steps']).shape==(self.n,) and
            np.allclose(np.asarray(options['time_steps'],float),self.dt,rtol=0.,atol=1e-12))

    def reset(self):
        super().reset()
        if hasattr(self,'_native'):self._native.reset()

    def _build_ocp(self):
        from acados_template import AcadosOcp,AcadosModel
        cfg=self.config;ocp=AcadosOcp();model=AcadosModel();model.name=self.model_name
        x=ca.SX.sym('x',9);u=ca.SX.sym('u',4 if cfg.envelope_soft_enabled else 3)
        # Frozen odom reference x,y,yaw,curvature,progress,tangent-norm;
        # desired speed, active jerk bound, recovery time remaining, immutable initial flag.
        p=ca.SX.sym('p',10)
        end,samples=interval_symbolic(x[:6],u[:3],x[7],cfg,self.dt)
        rate=(u[1]-x[7])/self.dt;rate_change=rate-x[8]
        discrete=ca.vertcat(end,u[0],u[1],rate)
        model.x=x;model.u=u;model.p=p;model.disc_dyn_expr=discrete
        heading=ca.vertcat(ca.cos(p[2]),ca.sin(p[2]));normal=ca.vertcat(-heading[1],heading[0])
        def errors(v):
            target=p[:2]+heading*p[5]*(v[4]-p[4]);offset=v[:2]-target
            return ca.dot(offset,normal),ca.dot(offset,heading),v[2]-p[2]
        ec,el,angle=errors(x)
        ff=ca.atan(cfg.wheelbase*(1+cfg.understeer_coefficient*x[3]**2)*p[3])
        residual=ca.vertcat(ec/cfg.contour_scale,el/cfg.lag_scale,
            ca.sin(angle)/cfg.heading_scale,(1-ca.cos(angle))/cfg.heading_scale,
            (x[3]-p[6])/cfg.speed_scale,(u[2]-p[6])/cfg.speed_scale,
            (u[1]-ff)/cfg.steering_scale,u[0]/cfg.acceleration_scale,
            rate/cfg.steer_rate,rate_change/(cfg.steering_rate_change_scale()*self.dt))
        weights=[cfg.contour_weight,RATIOS['lag'],cfg.heading_weight,cfg.heading_weight,
                 cfg.speed_weight,RATIOS['progress'],cfg.steering_weight,RATIOS['accel'],cfg.steering_rate_weight,cfg.steering_acceleration_weight]
        if cfg.envelope_soft_enabled:
            residual=ca.vertcat(residual,u[3]);weights.append(cfg.envelope_slack_weight)
        terminal=ca.vertcat(ec/cfg.contour_scale,el/cfg.lag_scale,
            ca.sin(angle)/cfg.heading_scale,(1-ca.cos(angle))/cfg.heading_scale,(x[3]-p[6])/cfg.speed_scale)
        model.cost_y_expr=residual;model.cost_y_expr_e=terminal
        ocp.cost.cost_type='NONLINEAR_LS';ocp.cost.cost_type_e='NONLINEAR_LS'
        ocp.cost.W=2*np.diag(weights);ocp.cost.W_e=2*cfg.terminal_weight*np.diag(weights[:5])
        ocp.cost.yref=np.zeros(len(weights));ocp.cost.yref_e=np.zeros(5)
        h=[];lo=[];hi=[];groups=[]
        def constraint(expr,lower,upper,group):h.append(expr);lo.append(lower);hi.append(upper);groups.append(group)
        if cfg.command_profile=='legacy_bounded_v1':
            constraint((u[0]-x[6])/(p[7]*self.dt),-1.,1.,'jerk')
        constraint(rate,-cfg.steer_rate,cfg.steer_rate,'steering_rate')
        if cfg.command_profile=='legacy_bounded_v1':
            constraint(rate_change,-cfg.steer_acceleration*self.dt,cfg.steer_acceleration*self.dt,'steering_acceleration')
        def corridor(value):
            if not cfg.enforce_corridor:return []
            front=ca.vertcat(ca.cos(value[2]),ca.sin(value[2]));left=ca.vertcat(-ca.sin(value[2]),ca.cos(value[2]))
            target=p[:2]+heading*p[5]*(value[4]-p[4])
            return [ca.dot(value[:2]+along*front+sign*cfg.half_width*left-target,normal)
                    for along in cfg.longitudinal_offsets() for sign in (-1,1)]
        def utilization(value,acceleration):
            accel,brake,lateral=cfg.envelope_halfaxes()
            axis=ca.if_else(acceleration>=0,accel,brake)
            ay=value[3]**2*ca.tan(value[5])/(cfg.wheelbase*(1+cfg.understeer_coefficient*value[3]**2))
            return (acceleration/axis)**2+(ay/lateral)**2
        for j,sample in enumerate(samples):
            # Soft mode handles immutable sample separately in numerical
            # diagnostics. Future stages retain their starting sample.
            slack=0.
            if cfg.envelope_soft_enabled:
                slack=ca.if_else(p[8]>j*.02+1e-10,u[3],0.)
            value=utilization(sample,u[0])-slack
            if cfg.envelope_soft_enabled and j==0:
                value=ca.if_else(p[9]>.5,0.,value)
            constraint(value,-1e15,1.,'operating_envelope')
            if j in (0,(len(samples)+1)//2,len(samples)-1):
                for value in corridor(sample):constraint(value,-self.path.right_width,self.path.left_width,'corridor')
        if cfg.envelope_soft_enabled:
            constraint(u[3]-ca.if_else(p[8]>1e-10,cfg.envelope_slack_limit,0.),-1e15,0.,'slack_cap')
        model.con_h_expr=ca.vertcat(*h)
        # acados does not inherit nonlinear path constraints into stage zero.
        # The first input must obey the same jerk/slew/envelope/footprint rows.
        model.con_h_expr_0=model.con_h_expr
        terminal_h=[utilization(x,x[6])]+corridor(x)
        model.con_h_expr_e=ca.vertcat(*terminal_h)
        self.effective_envelope_margin=max(cfg.optimization_envelope_margin,cfg.acados_envelope_margin)
        self._physical_upper=np.asarray(hi)
        native_upper=self._physical_upper.copy()
        native_upper[np.array(groups)=='operating_envelope']-=self.effective_envelope_margin
        ocp.constraints.lh=np.asarray(lo);ocp.constraints.uh=native_upper
        ocp.constraints.lh_0=np.asarray(lo);ocp.constraints.uh_0=native_upper.copy()
        ocp.constraints.lh_e=np.r_[-1e15,[-self.path.right_width]*(len(terminal_h)-1)]
        self._physical_terminal_upper=np.r_[1.,[self.path.left_width]*(len(terminal_h)-1)]
        ocp.constraints.uh_e=self._physical_terminal_upper.copy()
        ocp.constraints.uh_e[0]-=self.effective_envelope_margin
        ocp.constraints.idxbu=np.arange(u.shape[0]);ocp.constraints.lbu=np.array([-cfg.brake_limit,-cfg.steer_limit,0.]+([0.] if cfg.envelope_soft_enabled else []))
        ocp.constraints.ubu=np.array([cfg.accel_limit,cfg.steer_limit,cfg.max_speed]+([cfg.envelope_slack_limit] if cfg.envelope_soft_enabled else []))
        ocp.constraints.idxbx=np.array([3]);ocp.constraints.lbx=np.array([0.]);ocp.constraints.ubx=np.array([cfg.max_speed])
        ocp.constraints.idxbx_e=np.array([3]);ocp.constraints.lbx_e=np.array([0.]);ocp.constraints.ubx_e=np.array([cfg.max_speed])
        ocp.constraints.x0=np.zeros(9)
        ocp.model=model;ocp.parameter_values=np.r_[np.zeros(5),1.,cfg.cruise_speed,cfg.jerk_limit,0.,1.]
        ocp.solver_options.N_horizon=self.n;ocp.solver_options.tf=self.n*self.dt
        ocp.solver_options.integrator_type='DISCRETE'
        ocp.solver_options.nlp_solver_type='SQP_RTI';ocp.solver_options.qp_solver='PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.qp_solver_cond_N=min(self.n,5);ocp.solver_options.hessian_approx='GAUSS_NEWTON'
        ocp.solver_options.qp_solver_iter_max=100;ocp.solver_options.qp_solver_tol_stat=1e-7
        ocp.solver_options.qp_solver_tol_eq=1e-7;ocp.solver_options.qp_solver_tol_ineq=1e-7
        ocp.solver_options.qp_solver_tol_comp=1e-7
        ocp.solver_options.regularize_method='PROJECT';ocp.solver_options.print_level=0
        if self._expected_lib_path is not None:
            ocp.acados_lib_path=self._expected_lib_path
            ocp.acados_include_path=str(Path(self._expected_lib_path).parent/'include')
        self._transition=ca.Function('acados_discrete',[x,u],[discrete])
        self._constraints=ca.Function('acados_constraints',[x,u,p],[model.con_h_expr])
        self._terminal_constraints=ca.Function('acados_terminal_constraints',[x,p],[model.con_h_expr_e])
        self._groups=groups
        self._constraints_batch=self._constraints.map(self.n)
        self._transition_batch=self._transition.map(self.n)
        self._forward_transition=self._transition.mapaccum(self.n)
        group_names=np.asarray(groups)
        self._violation_group_indices={group:np.flatnonzero(group_names==group)
                                       for group in dict.fromkeys(groups)}
        return ocp

    def _trajectory_violations(self,states,controls,params,initial,upper,terminal_upper):
        value=np.asarray(self._constraints_batch(states[:-1].T,controls.T,np.asarray(params)[:-1].T))
        errors=np.maximum(np.maximum(self._ocp.constraints.lh[:,None]-value,value-upper[:,None]),0.)
        # The scalar path uses max(previous, error), ignoring NaN errors while
        # retaining finite violations. Preserve that diagnostic behavior; the
        # separate finite-state gate continues to reject nonfinite candidates.
        violations={group:float(np.fmax.reduce(errors[indices].ravel(),initial=0.))
                    for group,indices in self._violation_group_indices.items()}
        transitions=np.asarray(self._transition_batch(states[:-1].T,controls.T)).T
        stage_defects=np.max(np.abs(states[1:]-transitions),axis=1)
        violations['dynamics']=float(np.fmax.reduce(stage_defects,initial=0.))
        value=np.asarray(self._terminal_constraints(states[-1],params[-1])).ravel()
        violations['terminal_constraints']=float(np.max(np.maximum.reduce((self._ocp.constraints.lh_e-value,
            value-terminal_upper,np.zeros(len(value))))))
        violations['input_bounds']=max(0.,float(np.max(self._ocp.constraints.lbu-controls)),float(np.max(controls-self._ocp.constraints.ubu)))
        violations['speed_bounds']=max(0.,float(np.max(-states[:,3])),float(np.max(states[:,3]-self.config.max_speed)))
        violations['initial_state']=float(np.max(np.abs(states[0]-initial)))
        return violations

    def solve(self,state,previous,speed_refs=None,elapsed=.1,map_alignment=None):
        started=time.perf_counter();initial,applied,refs,alignment,seed=self.inputs(state,previous,speed_refs,elapsed,map_alignment)
        initial9=np.r_[initial,applied]
        geometries=self.geometries(seed[0][:,4],alignment)
        params=[];jerk=jerk_limits(initial,applied,self.config,self.dt,self.n)
        for k in range(self.n+1):
            cap=0.
            if self.config.envelope_soft_enabled and k*self.dt<self.config.envelope_recovery_time-1e-10:
                cap=self.config.envelope_recovery_time-k*self.dt
            p=np.r_[geometries[k],refs[k],jerk[min(k,self.n-1)],cap,float(k==0)]
            params.append(p);self._native.set(k,'p',p)
            memory=applied if k==0 else np.r_[seed[1][k-1,:2],(seed[1][k-1,1]-(applied[1] if k==1 else seed[1][k-2,1]))/self.dt]
            self._native.set(k,'x',np.r_[seed[0][k],memory])
            if k<self.n:
                self._native.set(k,'u',np.r_[seed[1][k],0.] if self.config.envelope_soft_enabled else seed[1][k])
        self._native.set(0,'lbx',initial9);self._native.set(0,'ubx',initial9)
        prepared=time.perf_counter();native_passes=[]
        # Each call performs preparation AND feedback. Keep the same OCP
        # parameters and the preceding native iterate between bounded passes;
        # never re-seed from the successful-candidate cache inside this loop.
        self._native.options_set('rti_phase',0)
        for step in range(self.config.acados_rti_steps):
            pass_started=time.perf_counter();status=self._native.solve()
            record=dict(pass_index=step+1,native_status=int(status),
                native_total_time_s=float(self._native.get_stats('time_tot')),
                wall_time_s=time.perf_counter()-pass_started,
                sqp_iterations=int(self._native.get_stats('sqp_iter')))
            try:
                record['nlp_residuals']=np.asarray(self._native.get_residuals(recompute=True)).tolist()
            except (RuntimeError,ValueError,OverflowError) as exc:
                record['residual_diagnostic_error']=str(exc)
            native_passes.append(record)
        optimized=time.perf_counter()
        raw_states9=np.asarray([self._native.get(k,'x') for k in range(self.n+1)])
        full_u=np.asarray([self._native.get(k,'u') for k in range(self.n)])
        native_violations=self._trajectory_violations(raw_states9,full_u,params,initial9,
            self._ocp.constraints.uh,self._ocp.constraints.uh_e)
        # RTI passes linearize dynamics. Execute the final unchanged inputs
        # through our nonlinear RK4 transition to construct a consistent plan.
        # The separate validator still uses its independent midpoint model.
        states9=np.vstack((initial9,np.asarray(self._forward_transition(initial9,full_u.T)).T))
        violations=self._trajectory_violations(states9,full_u,params,initial9,
            self._physical_upper,self._physical_terminal_upper)
        finite=np.isfinite(states9).all() and np.isfinite(full_u).all()
        violation=max(violations.values()) if finite else np.inf
        success=bool(status==0 and finite and violation<1e-4)
        diagnostics=dict(native_core='acados/HPIPM',native_status=int(status),max_constraint_violation=violation,
            constraint_violations=violations,artifact_fingerprint=self.fingerprint,
            constraint_violation_scope='forward_candidate_physical_bounds_and_dynamics',
            candidate_projection='forward_nonlinear_discrete_model',native_controls_modified=False,
            candidate_finite=bool(finite),
            acados_envelope_margin=self.config.acados_envelope_margin,physical_envelope_upper=1.,
            optimization_envelope_margin=self.config.optimization_envelope_margin,effective_envelope_margin=self.effective_envelope_margin,
            raw_native_constraint_violations=native_violations,raw_native_max_constraint_violation=max(native_violations.values()),
            raw_optimizer_state_dynamics_defect=native_violations['dynamics'],
            frozen_geometry=True,internal_state_dimension=9,nlp_solver='SQP_RTI',
            acados_rti_steps=self.config.acados_rti_steps,native_passes=native_passes,
            native_pass_statuses=[record['native_status'] for record in native_passes],
            native_total_time_s=sum(record['native_total_time_s'] for record in native_passes),
            sqp_iterations=sum(record['sqp_iterations'] for record in native_passes))
        if self.config.envelope_soft_enabled:diagnostics['optimizer_slack_max']=float(np.max(full_u[:,3]))
        result=dict(success=success,status=f'acados_status_{status}' if success else f'acados_candidate_rejected_{status}',
            iterations=diagnostics['sqp_iterations'],constraint_violation=violation,
            states=states9[:,:6].tolist(),controls=full_u[:,:3].tolist(),diagnostics=diagnostics)
        if not success:
            # Worker keeps this rare snapshot on disk, outside the compact
            # parent reply. Preserve rejected native inputs for exact diagnosis.
            result['failure_snapshot']=dict(initial_state=initial9.tolist(),
                applied=applied.tolist(),speed_refs=refs.tolist(),
                map_alignment=alignment.tolist(),raw_native_states=raw_states9.tolist(),
                raw_native_controls=full_u.tolist(),projected_states=states9.tolist(),
                frozen_parameters=np.asarray(params).tolist(),native_passes=native_passes,
                physical_constraint_violations=violations,
                raw_native_constraint_violations=native_violations)
        return self.finish(result,initial,applied,refs,alignment,started,prepared,optimized)
