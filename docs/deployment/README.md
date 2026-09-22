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

## Nav2 map launch

`aims_racer_system/nav.launch.py` is shared by the Orin and NUC native
workspaces. Run it on the computer assigned to navigation and supply the map
explicitly:

```bash
ROS_DOMAIN_ID=42 ros2 launch aims_racer_system nav.launch.py \
  map:=/absolute/path/to/map.yaml
```

The active Nav2 instance can publish autonomous commands. Do not run it beside
MPCC or another autonomous command producer.

## Simulation host

Numerical and Gazebo MPCC validation run on a separate Ubuntu 22.04 + ROS 2
Humble workstation, not on the Orin or NUC. The simulation source, native setup
and Docker setup are maintained on the
[`mpcc-sim` branch](https://github.com/EleSheep-moving/AIMSRacer/tree/mpcc-sim).
Start with that branch's [Simulation host guide](https://github.com/EleSheep-moving/AIMSRacer/blob/mpcc-sim/docs/simulation/README.md).
