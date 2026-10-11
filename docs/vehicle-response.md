<a id="车辆响应与控制器假设"></a>
# Vehicle response and controller assumptions

**English** · [简体中文](vehicle-response.zh-CN.md) · [Documentation](README.md)

This guide connects the current V35 controller model to the recorded vehicle response. It separates **configured model parameters**, **dated measurements**, and **behavior that has not yet been identified**.

<a id="当前控制器建模了什么"></a>
## What the current controller models

| Item | Current implementation | Meaning |
| --- | --- | --- |
| Longitudinal interface | MPCC speed command → RC selector → VESC speed PID | The optimizer's acceleration is integrated into a speed target; it is not a direct motor-acceleration command. |
| Longitudinal prediction | `dv/dt = a` | Ideal acceleration within the configured bounds; no separate speed-PID delay or response state. |
| Steering prediction | `dδ/dt = (δcmd − δ) / τ`, with `steering_tau: 0.08` | A first-order approximation of command-to-vehicle yaw response; no separate pure-delay state. |
| Yaw prediction | `dψ/dt = v tan(δ) / [L (1 + K v²)]` | Rear-axle bicycle model, `wheelbase: 0.36`, `understeer_coefficient: 0.0`. Zero `K` is a modeling assumption, not a measurement of zero tire slip. |
| Command publication / handover lead | 20 ms / 20 ms | Execution scheduling; neither value identifies actuator delay. |
| Acceleration / braking bounds | 1.0 / 1.0 m/s² | Configured prediction and command limits, not measured maximum traction or braking capability. |

`δ` is an **estimated effective steering state**, not a measured front-wheel angle. `/sensors/servo_position_command` reports the commanded servo position; receiving it does not prove that the wheels have reached that angle. The forwarded vehicle command is `/ackermann_cmd`; `/drive` is the controller proposal before RC selection.

The current model and configuration are in [vehicle.yaml](../src/controller/config/vehicle.yaml), [the exported interval model](../src/controller/aims_mpcc/backend_models.py), and [native state propagation](../src/aims_mpcc_rt/src/core.cpp). See [MPCC implementation](../src/controller/docs/implementation.md) for states, controls and integration.

### Longitudinal response depends on operating conditions

The speed-command response depends on the **VESC speed PID gains and controller settings**, **battery / DC-bus voltage**, current and power limits, current speed, vehicle load, and rolling resistance. Acceleration and braking can have different responses. A fitted delay and time constant therefore describe the recorded operating conditions; they are not universal vehicle constants.

The ERPM-to-speed conversion in [vesc.yaml](../src/aims_racer_system/params/vesc.yaml) sets the command/measurement scale. It does not identify the motor-controller PID tuning or the available torque. When comparing runs, record the VESC firmware configuration, PID gains, relevant limits, battery voltage and charge state, load, and speed range. The existing data do not establish a quantitative voltage-dependent response model.

<a id="纯延迟响应时间常数与估计滞后"></a>
## Pure delay, response constant, and estimation lag

For a delayed first-order response to a step command, a useful identification model is:

```text
t < D:   y(t) = y0
t ≥ D:   y(t) = y0 + (u − y0) [1 − exp(−(t − D)/τ)]
```

- **Pure delay `D`:** the interval before a response begins.
- **Response constant `τ`:** the rate of change after that delay. A first-order step reaches about 63.2% of its final change after one `τ` and about 90% after `2.3τ`.
- **Estimation lag:** the age and filtering/history effects of the state seen by the controller. This is separate from motor, servo and chassis response.

The current **0.08 s** steering parameter is a single lumped fit with no explicit `D`. It does not mean “the wheels start moving after 80 ms,” “90% response in 80 ms,” or a separately measured servo time constant. Adding another measured onset delay without refitting `τ` can count part of the same lag twice.

The 50 Hz command timer samples the plan more frequently than the up-to-20 Hz optimization requests. It does not make the actuator respond instantaneously. Similarly, 200 Hz EKF publication does not make delayed LIO measurements current; source timestamps and delayed-measurement handling still matter.

<a id="已有实测支持哪些结论"></a>
## MPCC computation and command delivery

A saved reference is prepared offline. Online MPCC repeatedly optimizes the next **15 × 0.1 s = 1.5 s** of motion; it does not optimize the entire lap on each request. The prediction horizon is a future time interval, not computation time. Bundle generation/build and startup are also separate from a running control cycle.

The running chain is: **latest EKF/map conditions → request snapshot and motion prediction → acados solve and candidate checks → result delivery → scheduled takeover and execution checks → `/drive` → RC selector → `/ackermann_cmd`**.

The two V35 NX field runs on 2026-10-10 ran FAST-LIO2, EKF and NDT concurrently. Rejoining the original logs by request sequence gives:

| Timing interval, P95 | V35 run A | V35 run B |
| --- | ---: | ---: |
| Native solver calls | 2.32 ms | 1.36 ms |
| Request snapshot → candidate delivery | 4.00 ms | 2.66 ms |
| Request snapshot → first `/drive` using the accepted new plan | 39.51 ms | 39.75 ms |
| Request snapshot → first matching `/ackermann_cmd` | 44.28 ms | 44.33 ms |
| EKF source timestamp → first matching `/ackermann_cmd` | 50.29 ms | 46.62 ms |

Request-to-candidate statistics cover **635 / 457 requests**. First-plan `/drive` statistics cover **531 / 442 accepted takeovers**. The `/ackermann_cmd` chains cover **428 / 427 unambiguously matched nonzero first commands**, excluding zero, repeated or ambiguous wire commands. Their EKF-source-to-command medians are **40.56 / 40.40 ms**, with observed maxima **55.67 / 60.81 ms**. These are observed samples, not guaranteed worst-case bounds; none of the 635 / 457 requests exceeded the configured delivery deadline.

Why does a few milliseconds of computation become tens of milliseconds of delivery? A request forecasts takeover at `submitted + 20 ms`; after that time, the next 20 ms output tick performs takeover checks and publishes the command. Computation overlaps the lead time, so **solver time + 20 ms + 20 ms is not a valid fixed sum**. The RC selector then forwards at a configured 200 Hz. For changed nonzero wire commands, measured `/drive`-to-`/ackermann_cmd` median delay was **5.13 / 4.98 ms**, P95 **6.31 / 6.03 ms**, across **1,094 / 1,100 matched pairs**. Independent stage percentiles cannot be added to obtain an end-to-end percentile.

The **50 ms `solver_timeout`** limits request-to-candidate delivery. It does not bound EKF-source-to-command delivery, receipt of an arbitrary new sensor sample, or physical vehicle response. Optimization requests are capped at 20 Hz: a newly arriving condition can wait for the next request, while existing plans continue producing commands at 50 Hz. The request always takes the latest qualified snapshot; the measurements above do not time every arriving input through a new solve.

Reproduction uses the original `controller/runtime.csv` files under the NX sessions `192955_v35_n15_accel1_cap4_lap01` and `193524_v35_n15_accel1_cap4_lap02`. Join `request`, accepted `activation`, and first `publish` on `sequence`; subtract `submitted` or `source_epoch` from that publication's `steady_s`. Match the recorded `/drive` and `/ackermann_cmd` by unique float32 wire speed/steering and message timestamps. The selector clears acceleration metadata in speed mode, so that field is not a matching key. Local analysis is retained under `/home/elesheep/aimsracer-data/sessions/2026-10-11/mpcc-command-latency/`; original data locations are documented in the [field review](reports/2026-10-10-mpcc-field-review.md).

**The endpoint here is command publication.** Driver/serial transport and the motor, servo and chassis response require separate measurements; `/ackermann_cmd` publication is not physical motion onset. EKF source age also does not account for the full age of every upstream LIO measurement.

## What the recorded measurements support

The following results come from the **2026-09-28 speed-mode calibration analysis**, mostly around 1 m/s. They are dated evidence, not a fresh high-speed identification:

| Measurement | Reported result | Interpretation |
| --- | --- | --- |
| Speed command → wheel speed fit | Approximately **40 ms pure delay + 0.16 s response constant** | Closed VESC speed loop and drivetrain response. The two fitted parameters are correlated. Neither is currently a longitudinal model parameter. |
| Sustained forward IMU acceleration onset | **45–60 ms**, median **52 ms**, 11 straight starts | A filtered onset detector; not the exact instant of first motor motion. |
| Single-lag steering fit | **0.0806 s** with wheel speed; **0.0798 s** with LIO speed | Supports the current lumped `steering_tau: 0.08` approximation in the measured regime. |
| Isolated steering-step yaw response | Median onset **52 ms**, 50% **109 ms**, 90% **159 ms**, 10 events around **0.88–1.03 m/s** | Vehicle yaw response inferred from IMU and speed, not measured mechanical wheel-angle response. |

The earlier response analysis also found that a richer fit of roughly **48 ms pure delay + 20–27 ms response constant** explained sharp steering steps. Those values describe another fit and event selection; they should not be combined with the current 0.08 s lumped parameter unchanged.

Yaw response includes the servo, tires, chassis and vehicle speed. Longer yaw settling at higher speed does not by itself prove a speed-dependent servo limit. The available records do not identify a complete acceleration/braking model across speed, battery state and load.

Read the original methods and qualifications in [speed-mode calibration](reports/2026-09-28-speed-mode-calibration.md) and [vehicle-response analysis](reports/2026-09-28-vehicle-response.md). The [2026-10-10 field review](reports/2026-10-10-mpcc-field-review.md) provides later tracking results, not a replacement actuator identification. Historical reports retain their original language and configuration context.

<a id="下一次调参如何利用这些信息"></a>
## How to use this in the next tuning session

1. Compare **forwarded `/ackermann_cmd`** with source-stamped wheel speed, LIO/EKF motion and bias-corrected IMU yaw rate. Keep command publication, measurement time and receipt time distinct.
2. Separate stationary starts, acceleration, braking and steady cornering; compare prediction residuals within speed ranges and comparable PID, voltage and load conditions. A speed-scale error or stale state can resemble an actuator-model error.
3. Keep frame and unit conventions consistent. `base_link` is the rear-axle center; raw Livox acceleration uses `g` and the rear-axle adapter converts it to m/s². Angular velocity uses rad/s. See [architecture](architecture.md).
4. Use repeated measured mismatch to decide whether to refit the lumped steering lag, introduce a separate pure delay, or identify a longitudinal response model. The current speed-mode interface has its own closed-loop response even though the optimizer predicts ideal acceleration.
5. After changing a model parameter such as `steering_tau`, **export and build a new solver bundle** and launch with it. Editing source YAML does not change an already running immutable bundle. Record the bundle/configuration with the run; follow the [reference and bundle workflow](../src/controller/docs/usage.md).

[Back to the documentation index](README.md)
