<a id="native-mpcc-实现与阅读路线"></a>
# Native MPCC implementation and reading guide

**English** · [简体中文](implementation.zh-CN.md)

The current production chain uses offline Python to generate the model/reference/bundle, and online C++ to call generated acados SQP-RTI. Deployment and enable commands are in [usage](usage.md); frames and command ownership are in [architecture](../../../docs/architecture.md). This document describes the current v35 configuration and native execution chain.

<a id="从输入到执行"></a>
## From input to execution

| Location | Responsibility |
| --- | --- |
| [recorder.py](../aims_mpcc/recorder.py), [prepare.py](../aims_mpcc/prepare.py) | Record raw rear-axle CSV and prepare a closed-loop reference |
| [path.py](../aims_mpcc/path.py) | Periodic quintic reference, projection, coordinates and geometry metadata |
| [speed_planner.py](../aims_mpcc/speed_planner.py) | Curvature speed limits, physical-distance integration and cyclic forward/backward acceleration/braking propagation |
| [backend_models.py](../aims_mpcc/backend_models.py), [acados_backend.py](../aims_mpcc/acados_backend.py) | Offline discrete model, costs and constraints |
| [export_bundle.py](../../aims_mpcc_rt/scripts/export_bundle.py), [build_bundle.py](../../aims_mpcc_rt/scripts/build_bundle.py) | Freeze inputs, generate C, compile for the target platform and create manifests |
| [core.cpp](../../aims_mpcc_rt/src/core.cpp) | Bundle loading, RTI, candidate reintegration and validation |
| [node.cpp](../../aims_mpcc_rt/src/node.cpp) | ROS state, worker, takeover, output, startup/stopping and asynchronous logging |
| [history.hpp](../../aims_mpcc_rt/include/aims_mpcc_rt/history.hpp), [execution.cpp](../../aims_mpcc_rt/src/execution.cpp) | Actual command history, takeover alignment and output sampling |
| [health.hpp](../../aims_mpcc_rt/include/aims_mpcc_rt/health.hpp) | Protocol-v1 map-anchor/health qualification |

<a id="状态模型与输入"></a>
## States, model and inputs

The six principal states, including virtual progress, are `[x,y,yaw,v,theta,delta]`: rear-axle position, heading, speed, reference progress parameter and modeled steering angle. The three controls are `[acceleration,steering_endpoint,virtual_progress_speed]`. Previous acceleration, steering-command endpoint and command rate are stored internally as well, giving nine internal states in total.

The main rear-axle model relationships are:

\[
\dot x=v\cos\psi,\quad \dot y=v\sin\psi,\quad
\dot\psi=\frac{v\tan\delta}{L(1+k_u v^2)},\quad
\dot v=a,\quad \dot\theta=v_\theta,\quad
\dot\delta=\frac{\delta_{cmd}-\delta}{\tau_\delta}.
\]

The `theta` parameter comes from cumulative chord length of recorded points; after smoothing it is not exactly physical arc length. Geometry linearization and speed propagation use the spline tangent norm. Currently `L=.36 m`, `tau_delta=.08 s` and `k_u=0`; the steering lag is a lumped approximation of vehicle yaw response. Zero understeer coefficient is a model assumption, not a measurement of no sideslip.

Each 100 ms solve stage contains five 20 ms RK4 substeps, and steering endpoints are sampled along the command ramp held by the output. The optimization model chooses acceleration, while the chassis actually receives speed setpoints; ideal acceleration propagation does not establish the unknown motor response. Detailed longitudinal identification remains part of the subsequent acceleration-interface plan.

<a id="参考误差与目标"></a>
## Reference, errors and objective

The reference is a fixed periodic quintic curve, and physical states propagate in `odom`; map references use the `map ← odom` alignment from the same qualification payload. Each RTI pass freezes local geometry at reference progress, and the second pass may update geometry. This model uses Cartesian states with contour/lag errors, rather than Frenet-state dynamics.

Contour is normal distance, lag is tangential error, and heading is compared with the reference tangent. Costs include normalized contour, lag, heading, speed tracking, virtual-progress-speed tracking, steering feedforward deviation, acceleration and steering-command rate/rate change; terminal costs amplify state costs. Currently `contour_weight=16`, `heading_weight=4` and `speed_weight=133.333...`. The weights correspond to different scales and units, so their numerical values cannot directly indicate relative strength.

Current v and virtual progress speed track the curvature-based speed reference, with no linear progress reward. The curvature speed bound is `min(cruise,max_speed,sqrt(ay/abs(kappa)))`, followed by acceleration and braking limit propagation over cyclic physical distance. The planned reference is frozen into the bundle. In the field runs, increasing cruise from 2.6 to 3.5 improved the planned ideal rolling time by only about 0.096 s; see the [field review](../../../docs/reports/2026-10-10-mpcc-field-review.md).

<a id="当前约束范围"></a>
## Current constraint scope

`rate_bounded_v2` retains speed, acceleration/braking, steering angle and steering-rate bounds; steering-rate change still carries a cost. The current independent-acceleration setting is `combined_accel_constraint_enabled=false`; the generated OCP, reintegrated candidates, execution certification and braking budget use consistent configuration. Curvature speed planning ay=1 remains active. It does not guarantee the tire lateral-force budget.

`enforce_corridor=false` disables reference footprint constraints and measured/startup footprint stopping checks. The saved 0.5 m widths on each side are a course model, not measured track boundaries. Restoring boundary constraints or changing physical parameters requires a new bundle and evaluation against actual space/vehicle data. Subsequent research assumptions are not part of this field-run evidence.

<a id="测量时间计算与接管"></a>
## Measurement time, computation and takeover

Each request fixes the original EKF source epoch, the execution start predicted from actual `/vesc/command` history, and map alignment. Only one pending plan is owned at a time, optimization requests are capped at 20 Hz, and commands are output at 50 Hz. `solver_timeout=.05 s` includes the full worker computation and result-delivery wait; results over budget do not enter execution.

Candidates first undergo nonlinear reintegration and constraint checks under the current configuration. At takeover, they are realigned to actual state and the applied prefix, then validated again. Execution cannot skip new controls that have not actually been issued. Candidates that fail validation are discarded, and the old plan retains its original source TTL; N15/dt=.1 defaults to `plan_ttl=1.2 s`. `handover_delay=.02 s` is the anticipated takeover lead time, timed separately from the solve budget and source TTL.

Command history must distinguish `/drive` proposals from forwarded `/vesc/command` commands. `minimum_drive_speed=.2 m/s` maps small positive setpoints for the existing chassis; disabled/faulted output stays zero, and stopping output below the threshold becomes zero. Physical speed can still pass through this interval during startup/stopping.

Worker calls, result delivery, takeover and the complete output callback are timed separately. Changes in state/clock/localization/RC ownership invalidate plans. C++ does not guarantee that upstream state is always fresh, and TF timestamps do not replace map qualification.

<a id="启动单圈与故障"></a>
## Startup, single lap and faults

The native node starts disabled, and `/mpcc/enable` performs the unified condition checks. `auto_start=true` waits for the same conditions within the default 60 s window, then starts only once on success; explicit stop cancels the wait. COMPLETE or a fault after a successful start does not automatically enable again, and fatal faults do not restart automatically.

`repeat_laps=false` counts one lap from the actual enable position, and the endpoint speed cap and stopping logic bring the vehicle to a stop; true runs continuous laps until operator stop. Plan expiry/loss of qualification enters stopping or recovery logic, and running again requires explicit authorization. Field RC/emergency stop still determines chassis execution permission.

<a id="日志与证据"></a>
## Logs and evidence

`runtime.csv` records worker disposition, delivery, takeover acceptance/reasons, reanchor and command publication, including zero output. Diagnostics report dropped entries from the bounded asynchronous log queue. `/mpcc/status` records actual state, freshness, automatic startup and solve/takeover counts. `record=true` stores the bag, runtime and system snapshot in one session directory; see [recording](../../../docs/operations/recording.md).

On 2026-10-10, NX completed closed-loop driving with actual FAST-LIO2/EKF/NDT, with a planned peak of 2.99 and measured peak approximately 2.89 m/s. The report's motion windows include startup/braking and cannot be called flying laps; full worker duration, native solver duration and output callback duration are interpreted separately. Software regressions live under `verification/`; checks of new entry points and single-workspace integration do not replace the original field runs.

Model and reference attribution is retained in [NOTICE.md](../NOTICE.md), [LICENSE](../LICENSE) and the native [NOTICE.md](../../aims_mpcc_rt/NOTICE.md).
