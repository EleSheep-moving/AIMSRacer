# Source attribution

The solver ports the existing local f1tenth-mpcc CasADi/IPOPT adapter. The body-corner constraints, six-state steering lag model, 20 ms RK4 substeps, objective weights and smooth steering commands preserve that adapter's mathematics. AIMSRacer changes provide ordinary local imports, explicit validated vehicle limits, asymmetric user corridors, rear-axle state input, external speed references, and reset support. No Isaac or occupancy-map dependency is imported.

The vendored global_kinematic_model.py, contouring_lag.py and normalized_cost.py are copied verbatim from local_mpcc. track.py retains only its wrap_s/PeriodicCubic implementation and original source attribution. Its Alexander Liniger Apache-2.0 attribution and upstream commit are retained in its header. The local_mpcc repository license is LGPL-3.0; the three copied helper files retain that license. See aims_mpcc/vendor/alexliniger_MPCC_LICENSE, aims_mpcc/vendor/local_repository_LICENSE (LGPL-3.0), and aims_mpcc/vendor/GPL-3.0_LICENSE (the incorporated GPL-3.0 terms). These license files are installed with the vendor package. The source adapter BSD-3-Clause license is retained as LICENSE; it does not replace third-party licensing.

The new path preparation uses the adapter's periodic-reference principle with recording validation, rear-axle conversion and bounded resampling; no occupancy or skeletonization implementation was copied. Dependencies CasADi/IPOPT, NumPy and SciPy remain separately licensed.

Source snapshot: f1tenth-mpcc commit 283f124a83ebe95f37878cc2e721be428ae99cf4 (copied-file SHA-256 below records working-tree content).

- `f1tenth_mpcc/solver.py`: `ff1a75fc7654d9f839ddd34bea691293906d326d2ebb373a9f03e9536e388024`
- `f1tenth_mpcc/track_tools.py`: `f006e0c7ab9ac33f688d9e03fe406929b48e30dc5ac9d5e84662733a70147580`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/global_kinematic_model.py`: `0244612f5e35b0c0796e80da7ea08c4e1c8ddbd2cbe76deb7d3885688928ddbb`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/contouring_lag.py`: `a016482d30875cdb049f0d65e9ab779ad70af9a3e840b8c8651ee42313323d00`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/normalized_cost.py`: `ac96edcb8269c16a688cccbebd8c1fd29937c06a84e8be107b8b5a220b944c42`
- `third_party/local_mpcc/scripts/barc_acados_mpcc/track.py`: `395789d90a18759f865775af6043a11b2cb7e1072daab67dbd5fe6eddb0c6201`
