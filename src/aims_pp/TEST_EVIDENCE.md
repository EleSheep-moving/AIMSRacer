# PP verification, 2026-10-13

Implemented with tests before production code. The first suite produced
14 failures and 6 setup errors because the new package did not exist; the
initial ROS tests produced 2 errors because `aims_pp.node` did not exist.
After implementation, the controller, canonical loader and message-independent
runtime tests passed, followed by the ROS interface tests.

Two additional concrete regressions were observed and fixed:

- With a ROS-clock publication timer, freezing `/clock` left phase `RUNNING`.
  The freeze test failed, the timer changed to `STEADY_TIME`, and it passed
  with zero speed and latched `FAULT` after monotonic source freshness expired.
- A repeated SIGINT during log-thread cleanup produced a `KeyboardInterrupt`
  traceback. The subprocess regression failed, shutdown ignored additional
  SIGINT while flushing the bounded queue, and it passed with exit code 0.

Final Humble command in the isolated development container:

```bash
source /opt/ros/humble/setup.bash
source /evidence/install/setup.bash
ROS_DOMAIN_ID=93 python3 -m pytest /repo/src/aims_pp/test -q -p no:cacheprovider --tb=short
```

Result after integration correction: **30 passed in 7.41 s**, no skips. Output retained in
`/evidence/pp-tests.txt` in `aims-pp-calibration-dev`. This includes AST numeric
parity against the recovered external controller (108 randomized steering
queries and 9 lookahead speeds), closed-circle projection/seam tests, source
clock/epoch/replay gates, one lap and repeated laps, actual SetBool services,
authority/current-mode stop, duplicate publisher detection, simulation-clock
freeze and CLI shutdown.

`colcon build --packages-select aims_pp` succeeded with install base
`/evidence/install`. Installed `ros2 launch aims_pp pp.launch.py --show-args`
listed all documented arguments. A 3-second isolated-domain installed launch
loaded the retained 2026-10-10 `bundle-v35-main/reference.json` with current
`src/controller/config/vehicle.yaml`, started disabled, and exited cleanly
after SIGINT. Log retained in `/evidence/pp-launch-smoke.txt`; disabled command
CSV in `/evidence/pp-legacy-smoke-verified/runtime.csv`. The legacy reference
is 33.46973190397723 m with 336 knots and map SHA-256
`1db8c1905dc99ed4c0897838421118b998d15eb7cf57f50cee96a8b0744870dc`.

Python compilation, package XML parsing and whitespace validation passed.
No hardware was accessed. This evidence does not establish target-ARM
realtime performance, full closed-loop tracking or field safety.

Integration correction: footprint checking follows the actual vehicle's
`enforce_corridor` flag, default false. The focused tests initially failed
because PPConfig lacked the flag and default association rejected a rear axle
outside nominal reference widths. Both passed after the flag was read from
vehicle configuration and the footprint gate became conditional. Forward
association, map identity, state freshness and authority gates remain active.
