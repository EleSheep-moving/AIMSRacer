# Source and dependency attribution

The C++ runtime is an AIMSRacer implementation of the rear-axle kinematic
controller and ROS interfaces. Its generated solver exports the existing
`aims_mpcc` CasADi model, objective and constraints; generation does not replace
the original source attribution or licenses. The original modeling NOTICE and
BSD-3-Clause, Apache-2.0 and LGPL-3.0 license materials are retained in `licenses/`.
The exact quintic reference coefficients and prepared speed profile are exported
from the same AIMSRacer NumPy/SciPy representation.

The acados runtime is pinned to v0.5.5, commit
`59d93e17d2985fdd73fc58b8a83ed8f83a024171`. acados and HPIPM use BSD-2-Clause;
BLASFEO uses BSD-2-Clause. Their upstream source and dependency attribution
remain authoritative. CasADi remains an offline modeling/code-generation
dependency. YAML-CPP and OpenSSL are dynamically linked system dependencies.

The solver/publisher separation, generated-C deployment and bounded RTI strategy
were investigated in `npu-ius-lab/Roboracer_China_2026`, branch `real-car-original`,
commit `6c5012b9cd310c8fca5281c408298ffb5d4b3885`. The NPU ROS 1 node, dynamic
bicycle model, vehicle parameters and residual model are not copied into this
package. Original-source smoke results are documented separately from this
package's own-car model and ROS 2 validation.

Each prepared bundle includes original source files, source/native hashes,
dependency hashes, configuration/reference identities and this attribution.
