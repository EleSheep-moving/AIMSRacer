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
parity against the recovered controller (108 randomized steering
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

Target portability correction: numeric parity now reads the bundled verbatim
fixture under `test/fixtures`, verifies fixture/class SHA and records its matching
historical Git blob. Tests require no external snapshot or host paths and do not
skip the 108 steering queries. This changes tests only.

## Native C++17 conversion — 2026-10-16

Workspace: `/repo` in `aims-calibration-native-20261016`; ROS Humble.
Isolated domain: `176`. No vehicle drivers were started.

- Native library/node build: PASS, isolated build `/tmp/pp-native-build`, install
  `/tmp/pp-native-install`; exported library consumed by the calibration package.
- Full retained and native PP test directory: **70 passed in 17.88 s**.
- Registered `colcon test` full regression target: PASS; `test-result` reports
  **71 tests, 0 errors, 0 failures, 0 skipped** (the additional result is the
  encompassing CTest invocation).
- Native reference/EnhancedPP parity covers wrap, projection, frozen speeds,
  steering transitions and clipping. Gate coverage includes the freshest 30 cm
  translation margin, yaw gate, duplicate source receipt, anchor heartbeat/loss,
  epoch retirement, transform mutation, malformed health, clock rollback with
  retained static map identity, explicit re-enable, and lap completion.
- Native ROS coverage includes disabled startup, SetBool ownership, authority
  loss, duplicate `/drive` publisher, current-mode final-command withdrawal,
  frozen ROS time and repeated SIGINT. The publication test checks 40–60 Hz from
  command header timestamps over one second; this is an adapter timing check,
  not a vehicle or actuator measurement.
- One earlier complete run had a retained Python repeated-SIGINT test exit with
  signal 2 when its second signal arrived 5 ms after the first; the native
  counterpart passed. Both the final full run and registered run passed.

Evidence files in the container: `/evidence/pp-native-build.txt`,
`/evidence/pp-native-tests.txt`, `/evidence/pp-native-colcon-test.txt`,
`/evidence/pp-native-colcon-test-result.txt`.

Commands after sourcing Humble and the message-package prefix:

```sh
colcon build --packages-select aims_pp --build-base /tmp/pp-native-build --install-base /tmp/pp-native-install
. /tmp/pp-native-install/setup.bash
AIMS_PP_NATIVE_RUNNER=/tmp/pp-native-build/aims_pp/pp_native_runner AIMS_PP_NATIVE_NODE=/tmp/pp-native-install/aims_pp/lib/aims_pp/pp_node ROS_DOMAIN_ID=176 python3 -m pytest -q src/aims_pp/test
colcon test --packages-select aims_pp --build-base /tmp/pp-native-build --install-base /tmp/pp-native-install
colcon test-result --test-result-base /tmp/pp-native-build
```

This verifies native source/build/numerical/ROS-adapter behavior on the isolated
x86 Humble host. It is not ARM realtime or real-vehicle acceptance evidence.
