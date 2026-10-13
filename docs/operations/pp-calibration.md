# Map reference editing, PP and longitudinal calibration

**English** · [简体中文](pp-calibration.zh-CN.md)

These are separate workflows: the browser edits a reference offline, PP tracks a
reference, and calibration owns staged speed/current experiments. The default race
controller remains the field-tested MPCC. The editor never publishes vehicle commands.

For B/C generated current sweeps, use the [current-sweep guide](current-sweep.md).

For the current field order (old-map MPCC first, then expanded-map PP), use the [command chain](field-mpcc-then-pp.md).

## Feature overlay

This feature is on `feat/pp-calibration-reference-editor`. The built NX checkout is:

```bash
cd /home/aims/aimsracer-data/code/pp-calibration-reference-editor
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source "$PWD/install/setup.bash"
```

The matching host checkout is `/home/elesheep/aimsracer-data/code/pp-calibration-reference-editor`.
The original main checkout and field bundle are retained. Source the feature overlay
in each terminal used for vehicle commands.

## 1. Draw a reference

Follow the [editor setup](../../tools/reference/editor/README.md), then start it
from the repository environment:

```bash
export PYTHONPATH="$PWD/src/controller:$PWD:${PYTHONPATH:-}"
/home/aims/aimsracer-data/dependencies/reference-editor-venv/bin/python \
  -m tools.reference.editor.server \
  --map "$HOME/maps/20260928_010503/map.pcd" \
  --config src/controller/config/vehicle.yaml \
  --output "$HOME/aimsracer-data/references/editor"
```

Open `http://127.0.0.1:8765`. When running the server on NX, forward it from the
desktop with `ssh -L 8765:127.0.0.1:8765 aims@192.168.123.107` and open that same
desktop URL. Python/GUI dependency installation is separate from the ROS overlay.

Load the known PCD, choose a useful height slice, place at least six route points,
and close the loop. The coordinates represent the **rear-axle base_link in map**;
do not shift the map origin to the first point. Choose the starting point and forward
direction, inspect the smooth curve, curvature/steering demand and suggested speed.
Dragging sparse points can move the curve beyond those points; visible point cloud
does not establish free space or track boundaries.

For PP calibration, mark the straight trial intervals. Preview must pass before
export. Exports contain the existing `path.csv`/`metadata.json`, canonical
`reference.json`, editable project and hash-bound calibration regions. Keep the whole
directory; a changed curve requires a fresh export, not copying old trial regions.
The display is downsampled; localization uses the original PCD and its original SHA.

MPCC uses this reference through the existing [export/build workflow](../../src/controller/docs/usage.md).
PP reads the canonical JSON directly and does not need a generated solver library.

## 2. Run PP instead of MPCC

Source the same overlay used for the vehicle. Start one controller:

```bash
ros2 launch aims_racer_system race.launch.py controller:=pp \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  reference_directory:="$HOME/aimsracer-data/references/editor/my-route" \
  speed_limit:=1.0 record:=true
```

Initialize known-map localization with a **base_link map pose** as described in the
[initialization guide](known-map-mpcc.md), select unlocked navigation speed mode on
RC, then explicitly enable PP:

```bash
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: true}'
```

`controller` defaults to `mpcc`; the existing `artifact_directory` MPCC command
retains its original behavior. PP may also use `artifact_directory` to read an
existing bundle's canonical reference and vehicle config. Its default additional
speed cap is 1 m/s, and the default run is one lap.

PP retains the old enhanced geometric steering law, including heading and curvature
terms. See [PP parameters](../../src/aims_pp/README.md). It has no optimizer or plan
expiration gate. Missing localization, stale state or withdrawn control authority
stops it and requires explicit re-enable. Stop with `{data: false}` or RC lock;
close launch only after the vehicle is stopped. MPCC and PP must not run together.

## Packaged calibration commands

The four live-operation executables share `aims_calibration/cli.py`. `operator.py` holds pure
readiness and result checks; `node.py` and `engine.py` own control and acquisition.
No temporary `field-tests` Python script, fixed PID or conversation authorization
string is required.

If the vehicle launch is already running, use only these commands:

```bash
ros2 run aims_calibration inspect
# Confirm pose/direction in RViz, unlock RC and select CH8 calibration; enable once:
ros2 run aims_calibration start
# To abort, lock with RC first; this leaves the vehicle launch and recorder running:
ros2 run aims_calibration stop
# Use the output_directory printed by inspect/start for offline acceptance:
ros2 run aims_calibration results /absolute/path/to/result-directory
```

The CLI and direct `/calib/enable` use the same node preflight: fresh observations
and selected authority, stationary wheel speed (absolute value <=0.1 m/s), a final
zero-speed request, disabled driver current-startup substitution, exclusive command
publishers and healthy logging. PP additionally requires matching trusted localization,
lateral distance <=0.25 m and heading error <=0.35 rad. ACC availability is checked
when the launch enables it. Start rechecks vehicle, PP, calibration, reference and
region file hashes. Correct a refused preflight and retry; an accepted start cannot
be repeated or reset. Completion, interruption or fault needs a fresh launch/session
for the next collection.

`start` submits one enable request. An uncertain response, lost monitor input or
Ctrl+C requests disable. It checks fresh wheel standstill continuously for 3 seconds
before reporting a stop. Zero current requests coasting, not guaranteed stopping;
unverified standstill explicitly requires operator intervention and never closes
the vehicle launch. `results` checks complete target coverage, configuration hash
and each record's `success` / `evidence_valid`. Empty, duplicate and partial runs
return nonzero exit codes.

In an NX graphical terminal, append `rviz:=true` to load the packaged `calibration.rviz` with the prior map and green reference. Software rendering is the default for the observed NX black-screen issue; `rviz_software_rendering:=false` selects GL rendering.

### Repository nine-speed preset

This preset is optional; default vehicle startup is unchanged. Site maps and exported
references remain data paths. All scripts, launch files, settings and documentation
are installed from the repository. The preset chooses packaged `speed_hold_1to5.yaml`,
`speed_hold_vehicle.yaml` and `speed_hold_pp.yaml`; no temporary vehicle/PP YAML is needed.

**Start this once only when no vehicle launch is running**. With an existing launch,
use the commands above instead:

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=speed_hold_1to5 steering_source:=pp \
  map_file:=/absolute/path/to/map.pcd \
  reference_directory:=/absolute/path/to/pp-routev2-straight-trials \
  record:=true longitudinal_acceleration:=true auto_start:=false
```

Nine targets 1.0, 1.5, ..., 5.0 m/s each settle for 1 second and accumulate 5 seconds
of eligible straight samples. Turns use 1 m/s recovery; turns and unstable intervals
add no acquisition time. `/calib/reference` is a latched `nav_msgs/Path`; add a Path
display in RViz with Fixed Frame `map`. NDT pose corrections retain `/initialpose`.
Use the printed result and bag session paths, not a fixed PID. After physical
standstill, Ctrl+C in the launch terminal and verify that session's `bag`.

## 3. Run longitudinal calibration

The system calibration launch starts the vehicle chain and **one** calibration
publisher; it does not start MPCC or an independent PP command publisher.

```bash
ros2 launch aims_racer_system calibration.launch.py \
  stage:=speed_hold steering_source:=pp \
  map_file:="$HOME/maps/20260928_010503/map.pcd" \
  reference_directory:="$HOME/aimsracer-data/references/editor/my-route" \
  record:=true
```

For manual steering, select `steering_source:=rc`; map and reference arguments are
unnecessary. RC uses steering CH1, lock CH5 and calibration CH8 under the current
vehicle profile. Select unlocked calibration mode before enabling the experiment:

```bash
ros2 service call /calib/enable std_srvs/srv/SetBool '{data: true}'
```

An idle fresh zero-speed request establishes the selector link. A service enable
does not bypass RC. `/control/calibration_enabled` reports actual selected valid
calibration authority; the original request is `/calib/vesc_command` and the
forwarded driver request is `/vesc/command`.

| Stage | Experiment | Configuration |
| --- | --- | --- |
| `speed_hold` | Stable speed, unique actual motor-current samples on straights | Defaults 1/2/3 m/s, 1 s stability, 10 s sample interval |
| `accel_interval` | Current acceleration through adjacent speed intervals | `accel_sweep_1to5` with accepted baseline; manual list optional |
| `decel_current` | Positive below-baseline reduction, coast and negative braking | `decel_sweep_1to5` with accepted baseline; manual list optional |

Copy the installed [calibration YAML](../../src/aims_calibration/config/calibration.yaml)
outside source, configure the trial, and pass `calibration_config:=/absolute/path.yaml`.
The default empty current list deliberately refuses current stages. Speed/current
requests stay within the selected vehicle bounds, with driver limits at ±100 A;
the packaged B/C profiles generate explicit baseline-bound sweeps. Use the [B/C guide](current-sweep.md) for their commands and acceptance. Both signed-current stages use
`MODE_CURRENT`, not the disabled brake mode or reserved acceleration mode.


The [speed_hold_1to5.yaml](../../src/aims_calibration/config/speed_hold_1to5.yaml) preset requests nine targets from 1 to 5 m/s in 0.5 m/s steps, accumulating five valid straight seconds each, settling for one second after every pause, and stopping after the ninth target. Select the packaged `profile:=speed_hold_1to5` above, or explicitly supply dedicated repository YAML with `max_speed: 5.0`. `hold_sampling: cumulative` retains individually documented segments; inter-segment gaps add no time. `continuous` selects the original uninterrupted-window mode. The dedicated 5 m/s limit does not change ordinary race settings.

The full calibration launch disables the 40 A current startup substitution **for
this driver instance**. Regular vehicle/race defaults are preserved. Acceleration
observation defaults on for calibration and remains independent of motor control.
The node-only package launch assumes this driver override has already been applied.

Cumulative speed hold pauses at a PP region exit or unstable speed, retaining
admitted samples and requiring settling again before resuming. Continuous hold
and current-response trials invalidate interrupted windows. Lost authority,
stale inputs and faults invalidate the current target with a retained reason;
reenabling begins preparation and does not resume a partly executed current step.
Normal completion commands zero speed; manual disable/fault publishes zero current
with the calibration startup substitution off.

PP continues tracking when nearest-point reference progress jumps. A projection
jump alone is not a localization fault; sampling uses the current projected
region, while actual localization invalidity still stops acquisition.
Measured speed overshoot no longer triggers an `overspeed` stop. Hold sampling
still admits only settled observations within the target speed tolerance.

## 4. Inspect evidence

`record:=true` creates a new dated session with bag, selected configuration and
reference snapshots, stage status and per-trial results. Recording does not arm the
vehicle. Calibration results are saved even when bag recording is off.

- Compare raw `/calib/vesc_command`, selected `/vesc/command` and
  `/control/calibration_enabled`; do not infer selection from producer intent.
- Measure actual motor current from `/sensors/core.state.current_motor`.
  `current_input` is battery-side current; final commands are not current measurements.
- Retain source timestamps for wheel speed, ERPM, voltage and acceleration quality;
  repeated messages are not independent samples. Interrupted/timeout trials are not
  successful acceleration measurements.
- Keep the PCD, exported reference, trial YAML and source revision together with the
  session. Software/PTY/replay checks and physical driving acceptance are separate.

The previous collection tutorials remain in the historical archive; their old
`/calib/ackermann_cmd` and direct-driver remaps are not current launch instructions.
