# AIMSRacer Gazebo MPCC validation

`aims_gazebo_sim` runs the existing ROS control path through Gazebo Fortress:

```text
aims_mpcc -> /drive -> RC selector -> /ackermann_cmd -> VESC converter
     -> VESC-to-Gazebo bridge -> Gazebo Ackermann/contact vehicle
     -> /odometry/filtered -> aims_mpcc
```

The model origin is the rear axle, matching the MPCC state convention. Its
`0.36 m` wheelbase and `0.26 m` track width are synthetic simulation parameters,
not a verified real-vehicle profile.

Use the [MPCC simulation host guide](../../docs/simulation/README.md) for the
complete installation and execution paths:

- [native Gazebo acceptance](../../docs/simulation/README.md#gazebo-acceptance)
- [native Gazebo and RViz2 window](../../docs/simulation/README.md#gazebo-and-rviz2-window)
- [Docker headless acceptance](../../docs/simulation/README.md#headless-gazebo-acceptance-in-docker)
- [Docker Gazebo and RViz2 window](../../docs/simulation/README.md#gazebo-and-rviz2-window-in-docker)

Gazebo acceptance establishes the configured ROS graph, simulated clock,
VESC conversion, vehicle model and contact physics. It does not calibrate real
tires, drivetrain dynamics, localization, onboard timing, or real-car safety.
