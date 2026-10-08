# Source attribution

The solver ports the existing local f1tenth-mpcc CasADi/IPOPT adapter. The body-corner constraints, six-state steering lag model, 20 ms RK4 substeps, objective weights and smooth steering commands preserve that adapter's mathematics. AIMSRacer changes provide ordinary local imports, explicit validated vehicle limits, asymmetric user corridors, rear-axle state input, external speed references, and reset support. No Isaac or occupancy-map dependency is imported.

The vendored global_kinematic_model.py, contouring_lag.py and normalized_cost.py are copied verbatim from local_mpcc. track.py retains its wrap_s/PeriodicCubic implementation and original source attribution, and AIMSRacer extends the shared NumPy/CasADi coefficient evaluator with PeriodicQuintic. Runtime references use the C4 quintic so curvature-dependent costs have continuous second derivatives at reference knots and the lap seam, provided the tangent is nonzero. Its Alexander Liniger Apache-2.0 attribution and upstream commit are retained in its header. The local_mpcc repository license is LGPL-3.0; the three copied helper files retain that license. See aims_mpcc/vendor/alexliniger_MPCC_LICENSE, aims_mpcc/vendor/local_repository_LICENSE (LGPL-3.0), and aims_mpcc/vendor/GPL-3.0_LICENSE (the incorporated GPL-3.0 terms). These license files are installed with the vendor package. The source adapter BSD-3-Clause license is retained as LICENSE; it does not replace third-party licensing.

The new path preparation uses the adapter's periodic-reference principle with recording validation, rear-axle conversion and bounded resampling; no occupancy or skeletonization implementation was copied. Dependencies CasADi/IPOPT, NumPy and SciPy remain separately licensed.

Source snapshot: f1tenth-mpcc commit 283f124a83ebe95f37878cc2e721be428ae99cf4 (copied-file SHA-256 below records working-tree content).

- `f1tenth_mpcc/solver.py`: `ff1a75fc7654d9f839ddd34bea691293906d326d2ebb373a9f03e9536e388024`
- `f1tenth_mpcc/track_tools.py`: `f006e0c7ab9ac33f688d9e03fe406929b48e30dc5ac9d5e84662733a70147580`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/global_kinematic_model.py`: `0244612f5e35b0c0796e80da7ea08c4e1c8ddbd2cbe76deb7d3885688928ddbb`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/contouring_lag.py`: `a016482d30875cdb049f0d65e9ab779ad70af9a3e840b8c8651ee42313323d00`
- `third_party/local_mpcc/scripts/barc_d7_f1tenth_mpcc_pilot/normalized_cost.py`: `ac96edcb8269c16a688cccbebd8c1fd29937c06a84e8be107b8b5a220b944c42`
- `third_party/local_mpcc/scripts/barc_acados_mpcc/track.py`: `395789d90a18759f865775af6043a11b2cb7e1072daab67dbd5fe6eddb0c6201`

## Native periodic projector

`aims_mpcc/kernels/projector.c` translates the scalar bounded minimizer and PPoly polynomial evaluation from SciPy 1.15.3, with NumPy 1.26.4 remainder arithmetic. The common arithmetic was checked against SciPy 1.8.0 / NumPy 1.21.5. An external scalar-kernel prototype had exact query and execution-gate comparisons on x86_64 and aarch64 for those respective version pairs; the integrated live-geometry wrapper requires its own qualification. This preserves the existing first coarse argmin, xatol=1e-12 and 500-call bounded minimizer policy. Unsupported library versions and customized geometry retain the Python algorithm.

SciPy copyright: Enthought, Inc. (2001-2002) and SciPy Developers (2003-2024). NumPy copyright: NumPy Developers. Both BSD-3-Clause redistribution licenses are retained as `aims_mpcc/vendor/SCIPY_LICENSE` and `aims_mpcc/vendor/NUMPY_LICENSE`, and are installed as package data. The original optimize.py module notice by Travis E. Oliphant (copy/use with no guarantee implied, provided that notice is retained) is also retained in the C header.

Primary sources:

- https://github.com/scipy/scipy/blob/v1.15.3/scipy/optimize/_optimize.py
- https://github.com/scipy/scipy/blob/v1.15.3/scipy/interpolate/_ppoly.pyx
- https://github.com/scipy/scipy/blob/v1.15.3/scipy/interpolate/_poly_common.pxi
- https://github.com/numpy/numpy/blob/v1.26.4/numpy/core/src/npymath/npy_math_internal.h.src

The library is compiled only by explicit offline preparation. Runtime dispatch reads the live PPoly coefficient/knot tables and existing coarse geometry, without changing their writability or copying a geometry snapshot. Compiler contraction, reassociation and fast math are disabled. This numerical optimization does not change controller limits, physical validation or acceptance policy.
