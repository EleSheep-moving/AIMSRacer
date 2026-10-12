# Longitudinal acceleration runtime preparation — 2026-10-12

**English** · [简体中文](2026-10-12-longitudinal-acc-runtime.zh-CN.md)

## Scope and source

Runtime implementation: `ba81fc7`, merged to main. Earlier source-time adapter fixes were preserved as `ab0374a`; 11 original source/test/report files remain byte-identical on desktop and NX. The old NX pending changes were backed up in a named stash and patch before fast-forwarding. Canonical observer SHA256: `965c724a24cb69bcbcffb226edbb36c56adb78a9c0d6c06c0874490e75474bb6`, byte-identical to the offline core.

The selected wheel + IMU observer is a separate opt-in ROS process. It estimates `d(body_vx)/dt`, publishes no commands/TF, consumes no LIO attitude, and is not fused into navigation EKF. Existing driving defaults remain unchanged. [Operation guide](../operations/longitudinal-acceleration.md).

## Software verification

- Before source-time fixes were committed: NX package tests, **89 tests, 0 failures**.
- Runtime spec and quality reviews: PASS; numerical/pipeline, runtime and launch checks, **50 tests passed**.
- Desktop installed package: **110 tests, 0 errors/failures/skips**.
- NX isolated installed package: **110 tests, 0 errors/failures/skips**.
- NX main workspace after pull/build: **110 tests, 0 errors/failures/skips**; installed launch arguments and import provenance checked.
- Initial isolated NX staging omitted two existing localization test fixtures; after copying the complete fixture directory, the full package test run passed. This was a test staging error, retained in the initial log.

## Installed ROS replay

Run `193524_v35_n15_accel1_cap4_lap02`, frozen SHA-verified wheel and corrected-force values. Original receipt intervals are replayed approximately in real time; receipt/source timestamps are shifted together into current ROS time. Actual DDS inputs and outputs run on localhost domain 191. Published values/covariance are checked against the canonical observer at each exact estimate stamp, in actual callback order. The runtime module path in JSON points to the installed package, not the source shim.

| Metric | Desktop | Orin NX |
|---|---:|---:|
| Received input events | 8967 | 8967 |
| Delivered / published outputs | 7170 / 7170 | 7170 / 7170 |
| Mean output rate, Hz | 199.84 | 199.84 |
| IMU callback P95 / max, ms | 0.351 / 0.932 | 0.850 / 20.094 |
| Output interval P95 / max, ms | 10.932 / 30.914 | 10.997 / 27.727 |
| Latest IMU source to estimate P95 / max, ms | 12.905 / 34.150 | 14.421 / 36.816 |
| Maximum oracle acceleration / covariance error | 0 / 0 | 0 / 0 |

All 8,967 observations were admitted. No numerical, too-old, future-stamp or snapshot failure occurred. Three initial IMU callbacks precede full initialization, explaining 7,173 IMU callbacks versus 7,170 outputs. The oracle uses the same core: exact equivalence verifies wrapper routing and timing, not independent physical correctness.

Callback timings include the intercepted publication's oracle snapshot/assertion overhead; they are instrumented wall times, not clean production callback costs. Output intervals also reflect recorded receipt batching, DDS and scheduler behavior. Average 200 Hz does not guarantee every publication within 5 ms; NX maximum instrumented IMU callback was 20.09 ms. The source-to-estimate age excludes subsequent subscriber receipt delay.

## Evidence and remaining work

Desktop evidence: `/home/elesheep/aimsracer-data/sessions/2026-10-12/longitudinal-acc-runtime/`. NX evidence: `/home/aims/aimsracer-data/sessions/2026-10-12/longitudinal-acc-runtime/`. See `desktop-build-test.log`, `nx-stage-verified-replay.log`, `nx-main-deployment.log`, `desktop-installed-dds/` and `nx-installed-dds/`. JSON includes exact module provenance, configuration and counters; JSONL retains every publication. Raw evidence directories are external and unavailable from a fresh clone.

No vehicle graph or driving command was started during these tests. Next: enable `longitudinal_acceleration:=true record:=true`, record a stationary interval and two normal field laps under FAST-LIO/EKF/NDT load. Wheel slip, slope transitions, vibration and calibrated uncertainty remain unverified. This establishes installed estimation readiness, not acceleration-feedback control acceptance.
