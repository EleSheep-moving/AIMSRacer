# Stage B/C software verification

**English** · [简体中文](2026-10-14-calibration-current-sweep.zh-CN.md)

## Scope and revisions

Branch `feat/calibration-current-sweep` is based on the latest NX Stage A feature
checkout, `867dad3`, plus the eight field patches preserved in `564d85c`.
Stage A acquisition/profile and implicit schema v1 are retained. B/C use the same
native selector/driver chain, frozen baseline-derived plans and schema v2 evidence.
Original A source/build/bags were not replaced. Final synchronized source revisions
and hashes are recorded with the NX verification artifacts.

## Verification

| Check | Result | Interpretation |
| --- | --- | --- |
| Host pure calibration tests | 146 passed | Planner, engine, CLI helpers, evidence, package/configuration contracts |
| NX complete calibration + system calibration-launch tests | 195 passed, 63.35 s | Includes ROS adapter tests and installed native command-chain cases |
| NX build | `aims_pp`, `aims_calibration`, `aims_racer_system` succeeded | Existing compiled vehicle dependencies supplied by underlays |
| Changed guide links | 141 checked, none missing | Bilingual operation/index/package guides |
| Original NX A pending field patch | Exact byte match | SHA256 `7c1a629fc551ee52dd9fbd1a4822dd42c18bd671bbaee557298aa5d6910a98a6` |

NX artifacts:
`/home/aims/aimsracer-data/verification/calibration-current-sweep/nx-final-20261013-v1byGl/`.
It contains `build.log`, `tests.log`, `tests.xml`, test result JSON/CSV and ROS logs.
ROS tests use isolated domains and localhost networking; driver integration uses
only `/dev/pts` fake VESC ports. No physical vehicle experiment was enabled.

Four new installed-chain cases verify generated B completion, B current-limit
skips across two intervals, C 104 mA/coast/signed braking commands, and C positive
tracking failure followed by coast/braking. The earlier seven wire cases also
cover Stage A, explicit signed current, lost authority and CLI start/stop/interrupt.
Scripted telemetry checks encoding and logic, not vehicle response fidelity.

Regression fixes cover logging overflow/latest failure checkpoints, a late fault
after COMPLETE, baseline/plan provenance, RC vehicle identity, pose argument
separation, tracking-configuration validation before startup, floating timestamp
precision and Ctrl+C deferral outside native ROS callbacks. The Ctrl+C defect was
reproduced twice before its isolated wire test and final suite passed.

## Selected real Stage A source

Source result:
`/home/aims/aimsracer-data/sessions/2026-10-13/222944_118387098-pp-speed-hold-1to5-repeat-battery/20261013T142946Z-1849a491/trials.json`.
All nine items pass offline acceptance. Derived baseline:
`/home/aims/aimsracer-data/calibration/baselines/20261013-repeat-battery.json`.
Baseline hash: `e2e367319a67e84a4161e46b1d2936a0c1abc06146db6a6d714f5b90afa7ad62`.
Voltage uses only admitted A segment samples; means range from about 16.2 to 15.9 V.
The baseline's vehicle hash matches the packaged field vehicle configuration.

The installed offline CLI generates **145 B items and 184 C items**. Preview JSONs
are adjacent to the baseline as `20261013-B-preview.json` and `20261013-C-preview.json`.
This is full planned coverage before current-capability skips, not completed B/C data.

## Remaining acceptance

Physical low-current and field B/C acceptance is pending. No software result here
establishes stopping distance, delivered current accuracy, acceleration response,
or high-current vehicle capability. Use the explicit inspected/armed workflow in
the [B/C operation guide](../operations/current-sweep.md).
