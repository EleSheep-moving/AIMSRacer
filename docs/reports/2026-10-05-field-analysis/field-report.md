# NDT-integrated field test, 2026-10-05

Map and reference identity match. No edits to submodules or tracked source during this session.

## First autonomous attempt

The controller stopped with `Plan expired` after 3.875 m of reference progress over 10.484 seconds. Three iteration-limited results had been received at the stop; the fourth was still pending and also reached 30 iterations. None of the returned results exceeded the 250 ms request deadline; no plans were rejected at handover.

Maximum absolute lateral error during autonomous motion: 1.067 m. Maximum measured forward speed before the fault: 1.040 m/s. These are measured tracking/state quantities, not target speed. Controller solve-time P50/P95/max: 92.4/163.7/177.7 ms, including unsuccessful returned solves and excluding warmup/manual operation.

During the motion window, recorded localization health remained `tracking`. Raw-IMU yaw direction agrees with steering commands. For turn samples above 0.3 m/s, the commanded-angle 80 ms first-order model predicts raw yaw rate with fitted gain 1.048 and RMS residual 0.075 rad/s. This is a check on this short run, not a new calibration. Map-to-odom values changed by spans 0.081 m in x, 0.136 m in y, and 0.030 rad yaw during the window; this is an observed span, not a single jump.

At 10.43 s, the command-derived angle estimate (-0.364 rad) and EKF speed (1.036 m/s) give modeled lateral acceleration -1.134 m/s² versus the configured 1.0 m/s² constraint. This later real state is inconsistent with that OCP constraint; exact failed-request states were not logged, so it does not prove the first failed request was infeasible. Increasing the iteration limit alone is not justified by these logs.

The car showed alternating steering corrections and an excursion exceeding one metre. The first run did not meet the 20 cm tracking objective and is not a successful autonomous lap.

## Next controlled trial

Only session-local cruise speed changed from 1.0 to 0.5 m/s in `config/vehicle-0p5.yaml`. Horizon 10, 5 Hz, 250 ms, 30 iterations, speed ceiling 1.5 m/s, existing dynamic constraints and disabled corridor enforcement remain. Operator confirmed locked/manual/stationary before restart. Cached worker ready; eight seconds of stationary computation returned successful solves. Awaiting renewed operator takeover confirmation.

Detailed data: `auto-first-controller-segment.json`, `auto-first-signals.json`, controller JSONL and recorded ROS bag.

## Completed low-speed lap

The 0.5 m/s session configuration completed one autonomous lap without iteration-limit skips, late-result skips or handover rejection. Reference progress 33.341 m, remaining 0.129 m, elapsed 90.4 s. Mean measured forward speed 0.395 m/s (0.402 m/s when >0.1 m/s). Absolute lateral-error median/P95/max 0.096/0.250/0.336 m; final lateral error -0.250 m. Solve-time median/P95/max 58.3/94.6/130.6 ms. Recorded localization health during this attempt: {}. This validates a complete low-speed run; it does not meet the requested 1 m/s mean and stable 0.2 m error objective. Steering oscillation remains to be addressed before increasing cruise speed. Full summary: `auto-0p5-summary.json`.

## Recording limitation and final state

The recorder exited with SQLite `database is locked` at about 23:14:29, after read-only analysis queries were made against its actively written file. The first autonomous attempt is present in the bag, but the completed 0.5 m/s lap is NOT. Do not infer full sensor coverage from the controller logs. Direct active-database reads will not be repeated; future analysis must wait for recorder shutdown. The completed lap has independent controller JSONL, predictions/status summary observations; localization observer counts: {'tracking': 2394}. The earlier empty localization-states field came from absent bag coverage, not absent localization messages.

Operator confirmed locked/manual/stationary after COMPLETE. Actual CH5=172, CH7=172; applied speed zero, measured speed 0.00015 m/s. MPCC stopped; V2, NDT and RViz retained. Reindexing the existing bag to recover metadata.

Recovered first bag metadata with `ros2 bag reindex`; `ros2 bag info` reads it. Integrated repository remains clean. MPCC process exited normally; V2/LIO, NDT and RViz remain running. Next work is to isolate steering oscillation using executed versus predicted trajectories, then increase cruise gradually; the 1 m/s / 20 cm objective remains outstanding.

## Follow-up timing and dynamics audit

See [diagnostics/README.md](diagnostics/README.md) for the source-time-aligned signal comparison, isolated EKF replay, prediction-prefix correction and offline sensitivity experiments. EKF forward-speed response lag relative to wheel speed was approximately 105–115 ms, while EKF message delivery age remained a few milliseconds. Isolated original-configuration replay reproduced 115 ms; changing only wheel measurement covariance or forward-speed process noise reduced that relative lag. These are local offline experiments, not adopted real-car settings. Pose yaw showed no clear tens-of-milliseconds lag relative to IMU-derived changes; EKF twist yaw rate showed approximately 40–45 ms. Velocity lag has not been established as the main cause of oscillation. No tracked source, submodules or production settings changed.
