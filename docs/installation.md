# Installation scope on `mpcc-sim`

This branch is for numerical and Gazebo MPCC validation on an x86-64 Ubuntu
22.04 workstation with ROS 2 Humble. It intentionally does not carry an Orin,
NUC, Livox, ZED, RC, VESC, or real-vehicle installation procedure.

Use the [MPCC simulation host guide](simulation/README.md) for the supported
native and Docker setup, build, numerical acceptance, Gazebo acceptance, and
Gazebo-plus-RViz2 execution commands.

For a vehicle computer, check out
[`feat/aims-mpcc`](https://github.com/EleSheep-moving/AIMSRacer/tree/feat/aims-mpcc)
and follow its [vehicle-computer deployment guide](https://github.com/EleSheep-moving/AIMSRacer/blob/feat/aims-mpcc/docs/deployment/README.md).

The obsolete vehicle-branch checkout instruction was removed from this branch.
