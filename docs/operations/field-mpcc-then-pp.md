# Old-map MPCC → expanded-map PP: field command chain

[简体中文](field-mpcc-then-pp.zh-CN.md)

Confirmed order: **tune MPCC using the old map and old reference first; then rebuild a larger map of the same site, draw a new reference and test PP. Record ACC during both driving workflows.** Small-current calibration is a separate experiment.

| Workflow | Map | Reference | Controller artifact |
| --- | --- | --- | --- |
| MPCC baseline/tuning | Old `20260928_010503/map.pcd` | Old bundle `input_reference` | Existing field bundle, or a newly exported tuning bundle on the same reference |
| Mapping | New directory | None yet | PGO `map.pcd` and `poses.txt` |
| PP | New expanded map | New GUI export | Canonical `reference.json`; no MPCC bundle |

## 0. Common environment in each NX terminal

Use **bash on NX**. Run the same block in every operation terminal. Choose a unique `AIMS_FIELD_TAG`, identical across terminals; preserve existing maps, references and bundles. The feature checkout is `feat/pp-calibration-reference-editor`; main retains the tested field version.

```bash
cd /home/aims/aimsracer-data/code/pp-calibration-reference-editor
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source "$PWD/install/setup.bash"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export AIMS_WS="$PWD"
export AIMS_FIELD_TAG=20261013-field
export AIMS_FIELD_ROOT="$HOME/aimsracer-data/field-tests/$AIMS_FIELD_TAG"
export AIMS_OLD_MAP="$HOME/maps/20260928_010503/map.pcd"
export AIMS_OLD_BUNDLE="$HOME/aimsracer-data/bundles/field-v35-20261010"
export AIMS_NEW_MAP_DIR="$HOME/maps/$AIMS_FIELD_TAG-expanded"
export AIMS_EDITOR_OUTPUT="$AIMS_FIELD_ROOT/new-references"
export AIMS_NEW_PP_REF="$AIMS_EDITOR_OUTPUT/pp-route"
export ACADOS_SOURCE_DIR=/home/aims/AIMSRacer/dependencies/work/acados/source
export ACADOS_INSTALL_DIR=/home/aims/AIMSRacer/dependencies/work/acados/install
export LD_LIBRARY_PATH="$ACADOS_INSTALL_DIR/lib:${LD_LIBRARY_PATH:-}"
mkdir -p "$AIMS_FIELD_ROOT/configs"
```

The old map/bundle paths were checked on NX. All vehicle terminals share the ROS domain. These are hardware commands; do not add replay simulated time or virtual timing re-arms.

## 1. Old-map MPCC baseline with ACC recording

**Terminal A:** keep RC locked and launch the full vehicle/localization/controller/recorder:

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-mpcc-baseline"
ros2 launch aims_racer_system race.launch.py \
  controller:=mpcc \
  map_file:="$AIMS_OLD_MAP" \
  artifact_directory:="$AIMS_OLD_BUNDLE" \
  auto_start:=false repeat_laps:=false \
  longitudinal_acceleration:=true record:=true \
  session_directory:="$AIMS_SESSION"
```

**Terminal B:** source the common environment, enter the actual rear-axle base_link pose in the old map. Angles are radians; neither an example origin nor the Livox position is an initialization measurement.

```bash
read -r -p 'Old-map base_link: x y z roll pitch yaw (rad): ' \
  AIMS_X AIMS_Y AIMS_Z AIMS_ROLL AIMS_PITCH AIMS_YAW
ros2 run aims_racer_system relocalize_known_map.py "$AIMS_OLD_MAP" \
  --pose-frame base_link --x "$AIMS_X" --y "$AIMS_Y" --z "$AIMS_Z" \
  --roll "$AIMS_ROLL" --pitch "$AIMS_PITCH" --yaw "$AIMS_YAW"
ros2 topic echo /localization/status --once
ros2 topic echo /mpcc/status --once
ros2 topic echo /state_estimation/longitudinal_acceleration/status --once
```

Check localization `ready=true`, solver/state eligibility and ACC `initialized=true`. With the car stationary and heading aligned to the reference, select unlocked navigation speed mode on RC and explicitly enable:

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: true}'
```

The default stops after one lap. Establish 3–5 repeat runs per configuration, retaining stationary segments before/after motion. Each new enable requires stationary/alignment/health conditions again. To separate bags, stop and restart launch using a new session. Operator stop:

```bash
ros2 service call /mpcc/enable std_srvs/srv/SetBool '{data: false}'
```

After actual standstill and permission withdrawal, Ctrl-C terminal A; then verify its recording in section 6.

## 2. MPCC tuning: keep old map/reference, build a new bundle

Stop the vehicle launch before offline generation. Copy the old bundle configuration and change one parameter at a time, such as `contour_weight`, `heading_weight` or speed planning. Keep the existing bundle intact:

```bash
cp "$AIMS_OLD_BUNDLE/input_config.yaml" "$AIMS_FIELD_ROOT/configs/mpcc-trial-01.yaml"
nano "$AIMS_FIELD_ROOT/configs/mpcc-trial-01.yaml"
export PYTHONPATH="$AIMS_WS/src/controller:$ACADOS_SOURCE_DIR/interfaces/acados_template:${PYTHONPATH:-}"
export AIMS_TRIAL_BUNDLE="$HOME/aimsracer-data/bundles/$AIMS_FIELD_TAG-mpcc-trial-01"
python3 "$AIMS_WS/src/aims_mpcc_rt/scripts/export_bundle.py" \
  --config "$AIMS_FIELD_ROOT/configs/mpcc-trial-01.yaml" \
  --reference "$AIMS_OLD_BUNDLE/input_reference" \
  --output "$AIMS_TRIAL_BUNDLE" --horizon 15 --dt 0.1 --source-only
python3 "$AIMS_WS/src/aims_mpcc_rt/scripts/build_bundle.py" \
  --bundle "$AIMS_TRIAL_BUNDLE" --acados-install "$ACADOS_INSTALL_DIR"
```

**Parameters are frozen in generated C and the bundle. YAML edits require export/build and a node restart.** The output must be a new directory; use `trial-02` etc. for later configurations. These steps rebuild the target bundle, not the entire ROS workspace.

Terminal A starts the new bundle on the same old map:

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-mpcc-trial-01"
ros2 launch aims_racer_system race.launch.py \
  controller:=mpcc map_file:="$AIMS_OLD_MAP" \
  artifact_directory:="$AIMS_TRIAL_BUNDLE" \
  auto_start:=false repeat_laps:=false \
  longitudinal_acceleration:=true record:=true \
  session_directory:="$AIMS_SESSION"
```

Terminal B repeats section 1 initialization/status/enable. The common block does not set `AIMS_TRIAL_BUNDLE`; build and launch in the same terminal, or set the exact generated bundle path in the launch terminal. Preserve configuration/run names, measured speeds and interventions. Stop the entire race graph before mapping.

## 3. Rebuild a larger map of the same site

**Terminal A:** drive manually over the expanded area, include loop closures and allow mapping to settle:

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-mapping-expanded"
ros2 launch aims_racer_system mapping.launch.py \
  record:=true session_directory:="$AIMS_SESSION"
```

**Terminal B:** common environment, save while mapping is still running:

```bash
bash "$AIMS_WS/tools/save_map.sh" "$AIMS_NEW_MAP_DIR"
sha256sum "$AIMS_OLD_MAP" "$AIMS_NEW_MAP_DIR/map.pcd"
```

The service must succeed and both `map.pcd` and `poses.txt` must be nonempty. This creates a new map; it does not preserve the old origin or stitch into the old map. After saving, standstill and Ctrl-C mapping, then verify its bag. PGO mapping and NDT race must run separately.

## 4. Draw a new PP reference on the new map

**NX terminal A:** common environment, start the offline editor:

```bash
export PYTHONPATH="$AIMS_WS/src/controller:$AIMS_WS:${PYTHONPATH:-}"
/home/aims/aimsracer-data/dependencies/reference-editor-venv/bin/python \
  -m tools.reference.editor.server \
  --map "$AIMS_NEW_MAP_DIR/map.pcd" \
  --config "$AIMS_WS/src/controller/config/vehicle.yaml" \
  --output "$AIMS_EDITOR_OUTPUT"
```

**Workstation terminal:** forward the loopback port, then open `http://127.0.0.1:8765`:

```bash
ssh -N -L 8765:127.0.0.1:8765 aims@192.168.123.107
```

Draw a forward closed rear-axle XY route, checking direction, start, curvature and actual clearance. Straight annotations are only needed for later PP calibration. Validate and **export using the name `pp-route`**. The resulting `$AIMS_NEW_PP_REF` contains the canonical reference, path/metadata, project and regions. The new map origin may differ; old reference coordinates and initialization poses do not transfer automatically. Preserve exports, then close the editor/tunnel if desired.

## 5. PP afterward: new map/reference, ACC recorded

**NX terminal A:** common environment, launch with the 1 m/s cap:

```bash
export AIMS_SESSION="$HOME/aimsracer-data/sessions/$(date +%F)/$(date +%H%M%S)-pp-new-map"
ros2 launch aims_racer_system race.launch.py \
  controller:=pp map_file:="$AIMS_NEW_MAP_DIR/map.pcd" \
  reference_directory:="$AIMS_NEW_PP_REF" \
  vehicle_config:="$AIMS_WS/src/controller/config/vehicle.yaml" \
  speed_limit:=1.0 auto_start:=false repeat_laps:=false \
  longitudinal_acceleration:=true record:=true \
  session_directory:="$AIMS_SESSION"
```

**Terminal B:** common environment, initialize base_link using the new map's actual pose:

```bash
read -r -p 'New-map base_link: x y z roll pitch yaw (rad): ' \
  AIMS_X AIMS_Y AIMS_Z AIMS_ROLL AIMS_PITCH AIMS_YAW
ros2 run aims_racer_system relocalize_known_map.py "$AIMS_NEW_MAP_DIR/map.pcd" \
  --pose-frame base_link --x "$AIMS_X" --y "$AIMS_Y" --z "$AIMS_Z" \
  --roll "$AIMS_ROLL" --pitch "$AIMS_PITCH" --yaw "$AIMS_YAW"
ros2 topic echo /localization/status --once
ros2 topic echo /pp/status --once
ros2 topic echo /state_estimation/longitudinal_acceleration/status --once
```

After qualified localization/state, stationary checks and unlocked navigation speed mode:

```bash
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: true}'
```

The default is one lap. Check tracking, stop and RC takeover before tuning PP. Operator stop:

```bash
ros2 service call /pp/enable std_srvs/srv/SetBool '{data: false}'
```

After standstill, close terminal A launch and verify the bag. Optional PP tuning file:

```bash
cp "$AIMS_WS/src/aims_pp/config/pp.yaml" "$AIMS_FIELD_ROOT/configs/pp-trial-01.yaml"
nano "$AIMS_FIELD_ROOT/configs/pp-trial-01.yaml"
```

Append `pp_config:="$AIMS_FIELD_ROOT/configs/pp-trial-01.yaml"` to the section 5 launch, choose a new session and restart. PP parameter edits require neither acados generation nor ROS recompilation, but this node does not support hot updates. Geometry changes require a new editor export/reference directory; preserve earlier versions.

## 6. Verify each run and analyze afterward

After normal Ctrl-C, in the **terminal that owned that launch**:

```bash
ros2 bag info "$AIMS_SESSION/bag"
ls "$AIMS_SESSION"
```

Confirm wheel odometry, rear IMU, longitudinal acceleration and its status, final VESC commands, motor telemetry, localization and the active controller are present. Retain map/reference/bundle/configuration identity with the recording. Include stationary start/end, acceleration, cruise, turns and deceleration. Dedicated current–wheel–ACC identification is a separate run.

Analyze MPCC tuning/lap timing/tracking, PP behavior, and ACC stationary noise, response lag, return to zero, turning effects and source validity separately. ACC remains observational, without navigation-EKF or acceleration-command integration. See the [implementation report](../reports/2026-10-13-pp-calibration-reference-editor.md) for software evidence and limitations. This command chain was not automatically executed on the vehicle; the operator starts and grants permission for each drive.
