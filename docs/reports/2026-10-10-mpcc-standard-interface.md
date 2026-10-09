# Native MPCC standard vehicle interfaces

Date: 2026-10-10. Branch: `feat/mpcc-acados-runtime`.
Tested source: `6bce41df` (documentation follow-up commits do not change the binary).

## Operator decision and implementation

The operator selected locked-car checks followed by supervised low-speed driving,
without a shadow mode. Longitudinal response identification is deferred to future
acceleration-control work. Synthetic response/envelope violations remain recorded
observations, not a prerequisite for these trials or a runtime stop gate.

The native node and launch no longer declare or select `shadow`. Outputs and service
use `/drive`, `/mpcc/status`, `/mpcc/reference`, `/mpcc/prediction` and `/mpcc/enable`.
The process supervisor observes `/mpcc/status`. Startup and restart remain disabled;
a successful explicit enable is required before driving. Authority, localization,
source-age, plan-TTL and retained actuator/envelope handling remain unchanged.
No model, objective, timing, acceleration/braking limit or bundle was changed.

Offline probes use explicit topic/service remaps and private ROS domains. This is
test-fixture isolation, independent of the removed vehicle runtime mode.

## Verification

| Check | Result |
|---|---|
| Desktop launch regression before repair | Expected RED: 3 failed, 3 passed |
| Desktop launch regression after repair | 6/6 PASS |
| Desktop C++ suite | 28/28 PASS |
| Desktop Python launch/supervision/accounting/startup tests | 42 PASS, 5 optional valid-bundle startup cases skipped |
| Desktop valid-bundle startup rerun | 7/7 PASS, including the five optional cases |
| Desktop actual ROS protocol | 7/7 PASS, 158 checks; original driving publishers all zero |
| NX ARM build and C++ suite | Build complete, 28/28 PASS |
| NX installed launch and actual ROS isolation | 6/6 launch tests, 8/8 ROS checks; original driving publishers all zero |

The actual-node startup regression checks removal of the parameter, standard
topic/service names honoring explicit remaps, disabled birth and explicit enable
counts across restart. The installed launch argument list has no shadow option.

NX executable SHA-256:
`650b18ba9ceff0257eb2711e13233ac126f0bad05bb22b519c8e16d529466cb2`.
Evidence directories:

- Desktop: `/home/elesheep/aimsracer-data/experiments/mpcc-acados-runtime/remove-shadow-20261010`.
- NX: `/home/aims/aimsracer-data/experiments/mpcc-acados-runtime/remove-shadow-20261010`.

Original vehicle installations and the previous frozen qualification installation
are preserved. The earlier joint-load and tracking measurements retain their
original binary provenance; no new joint-load timing or physical driving result
is claimed by this interface revision.

## Field handoff

Source the tested vehicle/localization underlays and the previous qualification
overlay for the updated localization monitor, then source:

```bash
source /home/aims/aimsracer-data/experiments/mpcc-acados-runtime/remove-shadow-20261010/ros-install/local_setup.bash
```

Select `implementation:=acados_cpp` and the matching v2 field bundle/config/reference.
There is no `shadow:=false` argument. For the first 0.5 m/s trial use the unchanged
`repair-final-20261009/bundles/qual-v2-field-v05-n10` bundle. Its reference binds
`/home/aims/maps/20260928_010503/map.pcd`; the experiment must use the corresponding
physical location and verified initial pose. Finish stationary checks before
calling `/mpcc/enable`. Full source order is in the
[repair validation launch section](2026-10-09-mpcc-runtime-repair-validation.md#launch--rollback).
