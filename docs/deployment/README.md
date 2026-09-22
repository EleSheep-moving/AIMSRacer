# Vehicle-computer deployment

Both onboard computers use a native Ubuntu 22.04 and ROS 2 Humble workspace.
They attach directly to vehicle hardware, so Docker is deliberately not part of
their deployment path.

| Computer | Role | Native installation and execution |
| --- | --- | --- |
| Jetson Orin NX | Vehicle sensing, RC, VESC and optional ZED | [Orin](orin.md) |
| x86-64 NUC | Nav2 and optional CP2102 FDI IMU | [NUC](nuc.md) |

Install device rules from [`rules/`](../../rules/README.md) only on the computer
physically connected to the matching serial hardware. A rule creates a stable
`/dev` name; it does not install or start its ROS driver.

## Simulation host

Numerical and Gazebo MPCC validation run on a separate Ubuntu 22.04 + ROS 2
Humble workstation, not on the Orin or NUC. The simulation source, native setup
and Docker setup are maintained on the
[`mpcc-sim` branch](https://github.com/EleSheep-moving/AIMSRacer/tree/mpcc-sim).
Start with that branch's [Simulation host guide](https://github.com/EleSheep-moving/AIMSRacer/blob/mpcc-sim/docs/simulation/README.md).
