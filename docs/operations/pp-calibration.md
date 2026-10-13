# Map reference editing, PP and longitudinal calibration

**English** · [简体中文](pp-calibration.zh-CN.md)

These are separate workflows: the browser edits a reference offline, PP tracks a
reference, and calibration owns staged speed/current experiments. The default race
controller remains the field-tested MPCC. The editor never publishes vehicle commands.

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
| `accel_interval` | Positive current from start_speed to end_speed | Explicit positive `currents`; end_speed greater than start_speed |
| `decel_current` | Negative current to a positive ending speed | Explicit negative `currents`; end_speed below start_speed |

Copy the installed [calibration YAML](../../src/aims_calibration/config/calibration.yaml)
outside source, configure the trial, and pass `calibration_config:=/absolute/path.yaml`.
The default empty current list deliberately refuses current stages. Speed/current
requests stay within the selected vehicle bounds, with driver limits at ±100 A;
there is no automatic full-current sweep. Both signed-current stages use
`MODE_CURRENT`, not the disabled brake mode or reserved acceleration mode.

The full calibration launch disables the 40 A current startup substitution **for
this driver instance**. Regular vehicle/race defaults are preserved. Acceleration
observation defaults on for calibration and remains independent of motor control.
The node-only package launch assumes this driver override has already been applied.

Leaving a PP trial region, intervention, lost authority, stale inputs or a timeout
invalidates the affected trial. Invalid trials are retained with their reason;
reenabling begins preparation and does not resume a partly executed current step.
Normal completion commands zero speed; manual disable/fault publishes zero current
with the calibration startup substitution off.

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
