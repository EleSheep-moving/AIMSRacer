# Main: tested field stack

User authorization: consolidate and merge into main using the 2026-10-10 field implementation. Preserve actual driving behavior; retire older runtime stacks and experimental operational wrappers.

## Scope and invariants

- Baseline AIMSRacer 5ad071c, FAST-LIO afe5f4c, NDT 5f795a6 plus the audited trusted-anchor patch, ndt_omp 63bf15b plus the audited line-search patch.
- Preserve FAST-LIO → rear-axle adapters/gyro correction → EKF; NDT owns map→odom, EKF owns odom→base_link. Preserve field geometry, configuration, command authority and native candidate/execution checks.
- Native acados C++ is the sole production MPCC. Python contains offline reference and solver generation support; no Python/IPOPT/QP driving entry points.
- One colcon workspace with pinned NDT sources in src; no cross-worktree field environment overlays.
- Required map and prepared native bundle remain external data, selected explicitly. Default field configuration is the last recorded 3.5 cruise profile; map geometry and speed planning still limit actual speed.

## Entrypoints

`vehicle.launch.py` owns hardware, FAST-LIO, EKF and rear frames, with parameter files retaining field values. `mapping.launch.py` starts the same hardware graph with the mapping TF arrangement and PGO. `race.launch.py` composes vehicle, known-map NDT and native MPCC. Recording is an optional rosbag process controlled by record; session directory is unique. RViz is optional, and initial pose may be supplied explicitly or through /initialpose. No automatic map-origin pose assumption.

Native `mpcc.launch.py` derives reference/configuration/horizon/TTL from the bundle. It has no backend selection, shadow, experimental supervisor prefix or duplicate solve-frequency parameters. Default startup is disabled. Optional auto_start uses the existing native enable transition under the output mutex and retries only readiness refusals before its first successful enable; no automatic restart or re-enable after completion/fault. Explicit stop cancels pending automatic startup. The selector retains independent RC ownership and stale-command behavior.

## Source layout

Production launch/scripts/config are installed explicitly. Active localization monitoring and gyro/frame adapters are production components, retained. Dependency setup and solver export/build are preparation tools. Current native/localization replay tools live under verification and are not installed or needed to start the car. Superseded benchmark, forensic, qualification and process-supervision wrappers remain available in Git history. Existing native regression tests remain development checks. Legacy runtime modules/tests and outdated launch/config stacks are removed after import-closure validation. Historical reports remain dated evidence; current README/operations describe only the new stack.

## Verification and release

Validate fresh baseline, offline exporter imports and native bundle generation, launch composition/arguments, recording QoS, optional auto-start lifecycle, native CTest, localization/IMU contracts, and target ARM compilation. Run isolated domains with no real drive publication. Verify dependency patches against the exact field source. Merge only after source and launch checks; push main and fast-forward NX main. Rebuild into a new production install before selecting its environment; preserve field install and data for rollback. No live vehicle enable is part of this consolidation.
