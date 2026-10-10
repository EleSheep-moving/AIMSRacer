#!/usr/bin/env python3
"""Offline only: freeze the existing Python OCP and exact quintic reference.

The runtime uses the stable capsule ABI in bridge.c, never Python/CasADi.
Generated C sources can be rebuilt on a different target with build_bundle.py.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import casadi as ca
import numpy as np

from aims_mpcc.acados_backend import AcadosSolver, _dependency_versions
from aims_mpcc.backend_models import NumericalBackend, interval_symbolic
from aims_mpcc.io import load_config
from aims_mpcc.path import ReferencePath
from aims_mpcc.speed_planner import SpeedPlanner


PIN = '59d93e17d2985fdd73fc58b8a83ed8f83a024171'


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_runtime_dt(dt):
    if not np.isfinite(dt):
        raise ValueError('isolated evaluation supports only 100 ms and 50 ms stages')
    for value in (.1,.05):
        if np.isclose(dt,value,rtol=0.,atol=1e-12):return value
    raise ValueError('isolated evaluation supports only 100 ms and 50 ms stages')


def export(config_path, reference_dir, output, horizon=15, dt=.1, source_only=False):
    from acados_template import AcadosOcpSolver
    cfg = load_config(config_path)
    dt=canonical_runtime_dt(dt)
    if dt==.05 and cfg.envelope_soft_enabled:
        raise ValueError('50 ms evaluation requires the strict envelope profile')
    path = ReferencePath.load(reference_dir)
    cfg.validate(require_verified=True)
    path.validate_config(cfg)
    dependencies = _dependency_versions()
    if dependencies['acados_source'] != PIN:
        raise ValueError('acados 0.5.5 source must be pinned to 59d93e')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(config_path, output / 'input_config.yaml')
    shutil.copytree(reference_dir, output / 'input_reference')
    package_root=Path(__file__).resolve().parents[1]
    if not (package_root/'NOTICE.md').is_file():
        from ament_index_python.packages import get_package_share_directory
        package_root=Path(get_package_share_directory('aims_mpcc_rt'))
    shutil.copyfile(package_root/'NOTICE.md',output/'NOTICE.md')
    shutil.copytree(package_root/'licenses',output/'licenses')
    (output / 'config.json').write_text(json.dumps(asdict(cfg), sort_keys=True, indent=2) + '\n')
    reference = dict(knots=path.curve.s.tolist(), coefficients=path.curve.coefficients.tolist(),
                     length=path.length, left_width=path.left_width, right_width=path.right_width,
                     frame_id=path.frame_id, metadata=path.metadata)
    planner = SpeedPlanner(path, cfg)
    reference['speed_profile'] = dict(schema_version=1, positions=planner.positions.tolist(), speeds=planner.speeds.tolist())
    (output / 'reference.json').write_text(json.dumps(reference, sort_keys=True, indent=2) + '\n')
    # _build_ocp is the same OCP constructor used by the tested Python backend.
    solver = AcadosSolver.__new__(AcadosSolver)
    # Preserve legacy backend validation. The offline runtime exporter alone
    # opts into the separately evaluated fractional integration interval.
    NumericalBackend.__init__(solver, path, cfg, horizon, .1)
    solver.dt=dt
    solver.model_name = 'aims_runtime'
    solver._expected_lib_path = dependencies['acados_lib_path']
    ocp = solver._build_ocp()
    # Legacy IPOPT sums stage costs at the baseline 100 ms interval. Acados
    # defaults to dt scaling, which would multiply terminal relative weight by
    # ten. Preserve the baseline ratio explicitly, and retain physical-time
    # scaling relative to that baseline for other supported discretizations.
    cost_scaling = np.r_[np.full(horizon, dt/.1), 1.]
    ocp.solver_options.cost_scaling = cost_scaling
    ocp.code_export_directory = str(output / 'generated')
    AcadosOcpSolver.generate(ocp, json_file=str(output / 'ocp.json'), verbose=False)
    x = ocp.model.x; u = ocp.model.u; p = ocp.model.p
    residual = ocp.model.cost_y_expr
    terminal = ocp.model.cost_y_expr_e
    _, samples = interval_symbolic(x[:6], u[:3], x[7], cfg, dt)
    functions = [ca.Function('rt_transition', [x, u], [ocp.model.disc_dyn_expr]),
                 ca.Function('rt_constraints', [x, u, p], [ocp.model.con_h_expr]),
                 ca.Function('rt_terminal_constraints', [x, p], [ocp.model.con_h_expr_e]),
                 ca.Function('rt_cost', [x, u, p], [ca.mtimes([residual.T, ca.DM(ocp.cost.W), residual]) / 2]),
                 ca.Function('rt_terminal_cost', [x, p], [ca.mtimes([terminal.T, ca.DM(ocp.cost.W_e), terminal]) / 2]),
                 ca.Function('rt_candidate', [x, u, p], [ca.vertcat(ocp.model.disc_dyn_expr,
                     ocp.model.con_h_expr, ca.mtimes([residual.T, ca.DM(ocp.cost.W), residual]) / 2,
                     *samples)], {'cse': True}),
                 ca.Function('rt_terminal_candidate', [x, p], [ca.vertcat(ocp.model.con_h_expr_e,
                     ca.mtimes([terminal.T, ca.DM(ocp.cost.W_e), terminal]) / 2)])]
    old = Path.cwd()
    try:
        os.chdir(output)
        generator = ca.CodeGenerator('model_eval.c', {'with_header': True})
        for function in functions:
            generator.add(function)
        generator.generate()
    finally:
        os.chdir(old)
    (output / 'bridge.c').write_text(BRIDGE)
    shutil.copyfile(Path(__file__).with_name('build_bundle.py'), output / 'build_bundle.py')
    shutil.copyfile(__file__,output/'export_bundle.py')
    source_root = Path(sys.modules['aims_mpcc.acados_backend'].__file__).parent
    source_files = sorted(str(p.relative_to(source_root)) for p in source_root.rglob('*')
                          if p.is_file() and p.suffix in ('.py', '.c'))
    source_hashes = {}
    for name in source_files:
        destination = output / 'sources' / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_root / name, destination)
        source_hashes[name] = digest(destination)
    manifest = dict(schema_version=1, command_profile=cfg.command_profile, stage_zero_envelope_bounds=1,
                    combined_accel_constraint_enabled=cfg.combined_accel_constraint_enabled,
                    constraint_groups=solver._groups, capsule_abi_version=1, backend='acados_sqp_rti_hpipm', acados_commit=dependencies['acados_source'],
                    generation_dependencies=dependencies,
                    generation_sources={name:digest(output/name) for name in ('export_bundle.py','build_bundle.py')},
                    horizon=int(horizon), dt=float(dt), nx=9, nu=int(u.numel()), np=10,
                    cost_scaling=cost_scaling.tolist(),
                    candidate_sample_count=len(samples),
                    integration_sample_times=[min(j*.02,dt) for j in range(len(samples))],
                    nh=int(ocp.model.con_h_expr.numel()), nh_e=int(ocp.model.con_h_expr_e.numel()),
                    lower=ocp.constraints.lh.tolist(), physical_upper=solver._physical_upper.tolist(),
                    terminal_lower=ocp.constraints.lh_e.tolist(), terminal_upper=solver._physical_terminal_upper.tolist(),
                    input_lower=ocp.constraints.lbu.tolist(), input_upper=ocp.constraints.ubu.tolist(),
                    source_hashes=source_hashes,
                    input_config_sha256=digest(config_path),
                    input_reference_hashes={name: digest(Path(reference_dir) / name) for name in ('path.csv', 'metadata.json')})
    # Independent Python evaluation fixtures are consumed by the C++ parity test.
    states = [np.array([1., -.2, .3, .7, .8, .12, .1, .15, -.05]), np.zeros(9)]
    controls = [np.array([.18, .17, .65] + ([.03] if cfg.envelope_soft_enabled else [])),
                np.array([0., 0., 0.] + ([0.] if cfg.envelope_soft_enabled else []))]
    parameters = np.r_[solver.geometry(.8, np.zeros(3)), cfg.cruise_speed, cfg.jerk_limit, 0., 1.]
    fixtures = []
    for state, control in zip(states, controls):
        heading=np.array([np.cos(parameters[2]), np.sin(parameters[2])])
        offset=state[:2]-(parameters[:2]+heading*parameters[5]*(state[4]-parameters[4]))
        ec=float(offset@np.array([-heading[1],heading[0]]));el=float(offset@heading)
        angle=state[2]-parameters[2]
        common=(cfg.contour_weight*(ec/cfg.contour_scale)**2 + (800/30)*(el/cfg.lag_scale)**2
                + cfg.heading_weight*(np.sin(angle)**2+(1-np.cos(angle))**2)/cfg.heading_scale**2
                + cfg.speed_weight*((state[3]-parameters[6])/cfg.speed_scale)**2)
        rate=(control[1]-state[7])/dt
        feedforward=np.arctan(cfg.wheelbase*(1+cfg.understeer_coefficient*state[3]**2)*parameters[3])
        analytic=(common+(40/30)*((control[2]-parameters[6])/cfg.speed_scale)**2
                  +cfg.steering_weight*((control[1]-feedforward)/cfg.steering_scale)**2
                  +(10/30)*(control[0]/cfg.acceleration_scale)**2
                  +cfg.steering_rate_weight*(rate/cfg.steer_rate)**2
                  +cfg.steering_acceleration_weight*((rate-state[8])/(cfg.steering_rate_change_scale()*dt))**2)
        if cfg.envelope_soft_enabled:analytic+=cfg.envelope_slack_weight*control[3]**2
        fixtures.append(dict(state=state.tolist(), control=control.tolist(), parameters=parameters.tolist(),
                             samples=np.asarray(ca.Function('fixture_samples',[x,u],[ca.horzcat(*samples)])(state,control)).T.tolist(),
                             transition=np.asarray(functions[0](state, control)).ravel().tolist(),
                             constraints=np.asarray(functions[1](state, control, parameters)).ravel().tolist(),
                             terminal_constraints=np.asarray(functions[2](state, parameters)).ravel().tolist(),
                             terminal_cost=float(functions[4](state, parameters)),
                             analytic_stage_cost=float(analytic), analytic_terminal_cost=float(cfg.terminal_weight*common),
                             cost=float(functions[3](state, control, parameters))))
    probes = np.linspace(-path.length, 2 * path.length, 103)
    (output / 'parity.json').write_text(json.dumps(dict(model=fixtures, reference=[dict(theta=float(t),
        speed=planner.at(t), speed_refs=planner.refs(t,horizon,dt), **path.at(t)) for t in probes]), indent=2)+'\n')
    manifest['files'] = {str(f.relative_to(output)): digest(f) for f in output.rglob('*')
                         if f.is_file() and f.name != 'manifest.json'}
    canonical = json.dumps(manifest, sort_keys=True, separators=(',', ':'))
    manifest['fingerprint'] = hashlib.sha256(canonical.encode()).hexdigest()
    (output / 'manifest.json').write_text(json.dumps(manifest, sort_keys=True, indent=2)+'\n')
    if not source_only:
        subprocess.run([sys.executable, str(output / 'build_bundle.py'), '--acados-install',
                        str(Path(dependencies['acados_lib_path']).parent)], check=True)
    return output


BRIDGE = r'''
#include <stdlib.h>
#include "generated/acados_solver_aims_runtime.h"
#include "model_eval.h"
int aims_rt_abi_version(void) {return 1;}
void *aims_rt_create(void) {
  aims_runtime_solver_capsule *c=aims_runtime_acados_create_capsule();
  if (!c) return NULL;
  if (aims_runtime_acados_create(c)) {aims_runtime_acados_free_capsule(c); return NULL;}
  return c;
}
void aims_rt_free(void *c) {if(c){aims_runtime_acados_free(c);aims_runtime_acados_free_capsule(c);}}
int aims_rt_reset(void *c) {return aims_runtime_acados_reset(c,1,1,0,0);}
void aims_rt_set(void *c,int stage,const char *field,const double *value) {
  ocp_nlp_out_set(aims_runtime_acados_get_nlp_config(c),aims_runtime_acados_get_nlp_dims(c),
                 aims_runtime_acados_get_nlp_out(c),aims_runtime_acados_get_nlp_in(c),stage,field,(void*)value);
}
void aims_rt_initial(void *c,const double *x) {
  ocp_nlp_constraints_model_set(aims_runtime_acados_get_nlp_config(c),aims_runtime_acados_get_nlp_dims(c),
    aims_runtime_acados_get_nlp_in(c),aims_runtime_acados_get_nlp_out(c),0,"lbx",(void*)x);
  ocp_nlp_constraints_model_set(aims_runtime_acados_get_nlp_config(c),aims_runtime_acados_get_nlp_dims(c),
    aims_runtime_acados_get_nlp_in(c),aims_runtime_acados_get_nlp_out(c),0,"ubx",(void*)x);
}
void aims_rt_input_bounds(void *c,int stage,const double *lower,const double *upper) {
  ocp_nlp_constraints_model_set(aims_runtime_acados_get_nlp_config(c),aims_runtime_acados_get_nlp_dims(c),
    aims_runtime_acados_get_nlp_in(c),aims_runtime_acados_get_nlp_out(c),stage,"lbu",(void*)lower);
  ocp_nlp_constraints_model_set(aims_runtime_acados_get_nlp_config(c),aims_runtime_acados_get_nlp_dims(c),
    aims_runtime_acados_get_nlp_in(c),aims_runtime_acados_get_nlp_out(c),stage,"ubu",(void*)upper);
}
int aims_rt_parameters(void *c,int stage,double *p) {return aims_runtime_acados_update_params(c,stage,p,10);}
int aims_rt_solve(void *c) {return aims_runtime_acados_solve(c);}
double aims_rt_cost(void *c) {
  double value=0.;ocp_nlp_eval_cost(aims_runtime_acados_get_nlp_solver(c),
    aims_runtime_acados_get_nlp_in(c),aims_runtime_acados_get_nlp_out(c));
  ocp_nlp_get(aims_runtime_acados_get_nlp_solver(c),"cost_value",&value);
  return value;
}
void aims_rt_get(void *c,int stage,const char *field,double *value) {
  ocp_nlp_out_get(aims_runtime_acados_get_nlp_config(c),aims_runtime_acados_get_nlp_dims(c),
                 aims_runtime_acados_get_nlp_out(c),stage,field,value);
}
int aims_rt_eval(int kind,const double *x,const double *u,const double *p,double *result) {
  const double *args[]={x,u,p};double *res[]={result};
  switch(kind) {
  case 0:return rt_transition(args,res,0,0,0);
  case 1:return rt_constraints(args,res,0,0,0);
  case 2:args[1]=p;return rt_terminal_constraints(args,res,0,0,0);
  case 3:return rt_cost(args,res,0,0,0);
  case 4:args[1]=p;return rt_terminal_cost(args,res,0,0,0);
  case 5:return rt_candidate(args,res,0,0,0);
  case 6:args[1]=p;return rt_terminal_candidate(args,res,0,0,0);
  default:return -1;
  }
}
'''


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--reference', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--horizon', type=int, default=15)
    parser.add_argument('--dt', type=float, default=.1)
    parser.add_argument('--source-only', action='store_true')
    args = parser.parse_args()
    print(export(args.config, args.reference, args.output, args.horizon, args.dt, args.source_only))
