# Calibration operator commands

Approved in conversation: preserve the NX speed-hold implementation and integrate its
operator workflow into aims_calibration. Four ROS executables share cli.py; readiness
and result acceptance belong to operator.py. The node and CLI use the same readiness
snapshot and predicates. No date, host user, reference path, authorization phrase or
launch PID is embedded. Vehicle launch remains separately owned by the operator.

Start explicitly enables one acquisition in one node lifetime. A refused preflight
can be corrected and retried; a submitted enable is never automatically retried.
Concurrent clients cannot reset a running acquisition. Interrupt or observation loss
requests disable; it cannot claim physical standstill without fresh wheel observations.
A new collection after completion or interruption needs a new launch/session.

PP preflight uses current matching localization and reference association: wheel speed
at most 0.1 m/s, lateral distance at most 0.25 m, heading error at most 0.35 rad.
Require actual selected calibration authority, fresh source/receipt timestamps, one
calibration and final command producer, no /drive producer, valid zero speed final
command, logging health, and driver current_startup_enabled=false. ACC availability
is required only if enabled by the system launch. Runtime safety stays with the node,
engine, selector and driver. The monitor reports; it never drives directly.

Nine speeds 1.0..5.0 m/s, settling 1 s and cumulative eligible sampling 5 s are an
explicit optional preset. Default vehicle/controller configuration does not change.
Results require planned target coverage, success and evidence_valid for each trial,
valid metadata and matching artifact hashes. Report partial/invalid runs honestly.

Publish the calibration reference as a latched map Path for RViz. Record settings,
association, start/stop events and evidence under the existing session. Documents are
English/Chinese. Validate pure policy first, ROS adapters and installed PTY chain on
NX in isolated ROS domains. Do not launch hardware or enable the real vehicle.

Implementation adds profile:=speed_hold_1to5 to resolve all three tuning YAML files
from the installed package. Optional rviz:=true selects packaged calibration.rviz;
software rendering is the default for the observed NX display issue. Site map and
reference data remain explicit launch inputs, and raw bags remain session artifacts.
