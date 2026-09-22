# AIMSRacer MPCC numerical ROS simulation

`aims_mpcc_sim` closes the deployed ROS control path through an independent,
real-time lagged kinematic-bicycle plant:

```text
aims_mpcc -> /drive -> joystick selector -> /ackermann_cmd -> VESC converter
     ^                                                          |
     +-------------- /odometry/filtered <- numerical plant <---+
```

The plant publishes rear-axle `odom/base_link` odometry and consumes the ERPM
and servo outputs from `vesc_ackermann`; it does not call the MPCC dynamics or
solver directly.

Install and run it on the workstation through the
[MPCC simulation host guide](../../docs/simulation/README.md):

- [native setup and numerical acceptance](../../docs/simulation/README.md#numerical-acceptance)
- [Docker setup and numerical acceptance](../../docs/simulation/README.md#numerical-acceptance-in-docker)

The numerical acceptance result establishes the ROS topic/process path and the
specified independent plant only. It does not validate real sensors, tire forces,
actuators, timing, or vehicle safety.
