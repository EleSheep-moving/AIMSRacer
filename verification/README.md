<a id="开发验证与重放"></a>
# Development verification and replay

**English** · [简体中文](README.zh-CN.md)

This directory retains development verification tools and reference configurations for the current controller and localization. It is not installed as a ROS production package; vehicle operation uses the [current bringup guide](../docs/operations/bringup.md). Historical experiment code can be consulted through Git `5ad071c` and dated [reports](../docs/reports/README.md), without restoring removed experimental dependencies to the vehicle.

| Content | Purpose / boundary |
| --- | --- |
| [mpcc/replay_requests.cpp](mpcc/replay_requests.cpp) | Independent cold-start replay of recorded requests; requires matching bundle/config/N/dt and does not reconstruct original warm-start history |
| [mpcc/replay_runtime.cpp](mpcc/replay_runtime.cpp) | Ordered request replay comparing frozen/refresh geometry strategies; requires matching bundle and captured JSON |
| [localization/fastlio_ndt_replay.py](localization/fastlio_ndt_replay.py) | Isolated FAST-LIO2/EKF/NDT replay of raw Livox bags; not vehicle control feedback |
| [localization/evaluate_replay.py](localization/evaluate_replay.py) | Evaluate collected replay evidence, judging sustained tracking and safe loss of qualification separately |
| [localization/tf_authority_audit.cpp](localization/tf_authority_audit.cpp) | Record TF publisher identity and source/receipt time to check unique ownership |
| [config](config/) | Configuration snapshots for development comparisons; production defaults are in `src/controller/config/vehicle.yaml` |

Current tests remain in the packages:

- [controller/tests](../src/controller/tests/): reference, model, speed planning, bundle and constraint checks.
- [aims_mpcc_rt/tests](../src/aims_mpcc_rt/tests/): native numerical, execution, timing, lifecycle and localization qualification checks.
- [aims_racer_system/tests](../src/aims_racer_system/tests/): rear-axle conversion, gyro, localization protocol and launch contracts.
- [tools/tests](../tools/tests/): pinned dependency preparation behavior.

Current Python checks in the native package retain `test_export.py`, `test_fractional_interval.py` and `test_node_startup.py`, alongside active C++ numerical/execution tests; old Python checks requiring removed experimental tools are not restored to production.

Run tests according to their source/CMake registration. Numerical tests need an explicitly prepared suitable bundle and acados shared-library environment. Replay uses original lidar/IMU and recording QoS in an isolated ROS domain, with output to a new evidence directory. Inspect parameters first:

```bash
python3 verification/localization/fastlio_ndt_replay.py --help
python3 verification/localization/evaluate_replay.py --help
```


The replay script launches ROS using the absolute path of the repository's [replay launch](localization/launch/fastlio_ndt_replay.launch.py), rather than an installed production launch. The TF audit executable explicitly comes from the development build:

```bash
python3 verification/localization/fastlio_ndt_replay.py \
  /absolute/path/to/raw-bag /absolute/path/to/new-evidence \
  --audit-executable "$PWD/build/aims_racer_system/tf_authority_audit" \
  --map /absolute/path/to/map.pcd --seed /absolute/path/to/seed.json
```


`seed.json` contains map/base_link `position: [x,y,z]` and `orientation: [qx,qy,qz,qw]`, filled in from the actual placement; specified inputs must actually exist.

Report evidence separately for source/config inspection, build, software execution, combined load and field behavior. Passing software does not prove untested road grip or higher speeds; recorded field-run motion windows are not flying laps either. Full boundaries are in the [2026-10-10 field review](../docs/reports/2026-10-10-mpcc-field-review.md).

## Current and lateral-acceleration calibration

[Current calibration A/B/C and selected results](../docs/operations/calibration.md#current-calibration) · [Lateral AY validation and grip status](../docs/operations/lateral-acceleration-calibration.md). Source is unified under `src/calibration/`; use the restored C++ lateral entry. Historical Python commands are provenance only.