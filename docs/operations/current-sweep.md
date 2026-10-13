# Baseline, acceleration and deceleration calibration

**English** · [简体中文](current-sweep.zh-CN.md)

This workflow uses the NX field version of Stage A, then generates B/C from an
explicitly selected accepted A result. A acquisition settings and schema v1 are
preserved. All control and operator commands live in AIMSRacer; maps, references,
baseline exports and recorded sessions are external data.

## 1. Load the B/C overlay

Branch: `feat/calibration-current-sweep`. In each NX terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/aims/AIMSRacer/install/setup.bash
source /home/aims/aimsracer-data/code/pp-calibration-reference-editor/install/setup.bash
source /home/aims/aimsracer-data/code/calibration-current-sweep/install/setup.bash
export AIMS_MAP=/home/aims/maps/20261013-field-expanded/map.pcd
export AIMS_REFERENCE=/home/aims/aimsracer-data/field-tests/20261013-field/new-references/pp-routev2-straight-trials
export AIMS_BASELINE=/home/aims/aimsracer-data/calibration/baselines/20261013-repeat-battery.json
```

The earlier feature overlay supplies existing compiled vehicle dependencies. The
new overlay supplies PP, calibration and system launch files. Stage A's original
checkout, build and recorded data remain available.

## 2. Export the accepted A baseline offline

The selected run has nine accepted speeds, 1–5 m/s at 0.5 m/s spacing:

```bash
export AIMS_A_RESULT=/home/aims/aimsracer-data/sessions/2026-10-13/222944_118387098-pp-speed-hold-1to5-repeat-battery/20261013T142946Z-1849a491/trials.json
ros2 run aims_calibration results "$AIMS_A_RESULT"
ros2 run aims_calibration baseline "$AIMS_A_RESULT" --output "$AIMS_BASELINE"
ros2 run aims_calibration plan --stage accel_interval --base-current-file "$AIMS_BASELINE"
ros2 run aims_calibration plan --stage decel_current --base-current-file "$AIMS_BASELINE"
```

These commands never enable the vehicle. The export checks all nine A records,
configuration and source hashes. It retains actual motor-current means, standard
deviations, sample counts, effective durations and source context. If segment
boundaries and `observations.csv` are available, voltage is calculated from those
admitted samples only, with its file hash retained. Older results without this
evidence retain an unknown voltage; no voltage is invented.

The source A files are never edited. Select a new baseline explicitly after a new
A run, a vehicle change or a material battery-condition change. Baseline voltage
is provenance, not an automatic correction or a runtime voltage gate. PP sweeps
check the source map, reference, region and vehicle identity when present; RC
sweeps check vehicle identity. A modified baseline file refuses a new enable.

## 3. What the sweeps measure

Let `I0(v)` be the measured Stage A mean motor current at speed `v`.

| Stage | Speeds | Requested currents | End condition |
| --- | --- | --- | --- |
| A `speed_hold_1to5` | 1, 1.5, …, 5 m/s | Speed mode; measure actual current | Five effective straight seconds per speed |
| B `accel_sweep_1to5` | Eight adjacent intervals from 1→1.5 through 4.5→5 | `I0(v0)+5`, `I0(v0)+10`, …, up to 100 A | Wheel speed ≥ `v0+0.5` |
| C positive | Start speeds 1.5, 2, …, 5 | `I0(v0)-5`, `I0(v0)-10`, … while strictly positive | Wheel speed ≤ `v0-0.5` |
| C coast | Same start speeds | 0 A | Wheel speed ≤ 1 m/s |
| C braking | Same start speeds | −5, −10, …, −100 A | Wheel speed ≤ 1 m/s |

B appends 100 A if its final 5 A increment is lower than that ceiling; it refuses
an interval when even `I0+5` exceeds the ceiling. B never tests positive currents
at or below baseline. C preserves fractional offsets: with `I0(1.5)=10.1045 A`,
the positive entries are about 5.1045 and 0.1045 A. They are not rounded to zero.
The selected A run generates **145 B items and 184 C items** before any skips.

Each item restores speed mode to its own start speed, settles within ±0.2 m/s for
one continuous second, then requests current mode. Response timeout is 8 s.
Exiting a marked straight invalidates that attempt; the same item retries up to
three total attempts. PP follows curves at 1 m/s and settles again on reentry.
Normal item transitions restore the next start speed; the final item requests zero
speed. Signed current and coast all use native `MODE_CURRENT`, not brake mode.

### Current capability and skipping

Current checks begin 0.25 s after final selected current confirmation. A rolling
0.5 s window needs at least ten unique, fresh telemetry observations. Tolerance
is `max(1 A, 10% of |target|)`; failure needs at least 90% qualifying samples.

- B: insufficient positive motor current skips stronger B entries for that same
  speed interval, then advances to the next interval.
- C positive: a consistent same-side absolute tracking error skips remaining
  positive entries at that start speed, then continues coast and negative braking.
- C negative: insufficient braking magnitude skips stronger negative entries at
  that start speed, then advances to the next start speed.
- A speed timeout with adequate current feedback does **not** prove an electrical
  limit; the next current is still tested. A fresh successful speed boundary takes
  priority over a simultaneous current-limit decision.

A response shorter than the tracking window can complete with
`tracking_status: unverified`; it never justifies skipping stronger currents.
The skip rule applies to generated ordered groups. In an explicitly supplied
manual list, it skips only the following contiguous stronger entries of that group.

## 4. Launch one stage

First finish and close any existing vehicle launch after physical standstill.
Changing profiles requires a fresh session. Launch **B** once:

```bash
ros2 launch aims_racer_system calibration.launch.py \
  profile:=accel_sweep_1to5 base_current_file:="$AIMS_BASELINE" \
  steering_source:=pp map_file:="$AIMS_MAP" reference_directory:="$AIMS_REFERENCE" \
  record:=true longitudinal_acceleration:=true auto_start:=false rviz:=true
```

For **C**, use the same command with `profile:=decel_sweep_1to5` in a separate
session. The profile selects the packaged 5 m/s vehicle and PP settings, requires
the baseline and freezes its full plan before recording or vehicle startup.
`stage:=auto` is the default and resolves to the selected profile; incompatible
explicit `stage` arguments are refused. Stage A remains
`profile:=speed_hold_1to5` and needs no baseline.

The calibration graph turns off the VESC driver's 40 A startup substitution and
starts one native calibration publisher. Requested current is capped at ±100 A;
actual capability depends on motor, VESC limits, voltage and speed. The graph does
not start MPCC or a second PP command publisher.

## 5. Inspect, enable once and stop

In a second terminal with the same environment:

```bash
ros2 run aims_calibration inspect
# Check map pose, direction and marked straight in RViz. Unlock/select calibration.
ros2 run aims_calibration start
```

Preflight checks standstill, selected authority, fresh input, exclusive command
ownership, localization/reference identity, disabled startup substitution and
logging. No automatic enable is introduced. The current field patch continues
to tolerate PP projection jumps and measured-speed overshoot; settled A sampling
still uses its target tolerance. Current stages also reject nonzero VESC fault codes.

`start` monitors B/C for up to 7200 s by default (A remains 900 s). An override is
available as `start --max-seconds 3600`. It displays interval, kind, target current,
attempts and skips. For interruption, lock the vehicle with RC first, then:

```bash
ros2 run aims_calibration stop
```

Disable/fault requests zero current; zero current is coasting and does not prove
standstill. The CLI waits for three seconds of fresh stopped wheel evidence.
After confirming physical standstill, close launch with Ctrl+C. It does not close
the vehicle graph automatically.

## 6. Accept evidence separately from coverage

```bash
ros2 run aims_calibration results /absolute/path/from/output_directory
ros2 bag info /absolute/path/to/session/bag
```

B/C schema v2 contains the frozen plan and hash, baseline, current-tracking
configuration, every actual attempt and explicit skipped items. `attempt_id`
identifies a real attempt; skipped items refer to the limit-triggering attempt
and retain its feedback proof. A keeps schema v1. Results distinguish:

- `execution_complete`: every item has an execution outcome or recorded skip.
- `evidence_valid`: logs and provenance passed validation.
- `coverage_complete`: every item has a valid successful measurement or a verified
  current-limit/skip decision. Timeouts and exhausted retries leave coverage gaps.
- `accepted`: successful measurements only; unreachable/skipped items are separate.

Current-limit decisions do not become successful acceleration measurements. A
logging overflow or writer error invalidates evidence, including completion races.
If disk failure prevents a checkpoint, status still reports the failure; an old
success JSON is insufficient for runtime acceptance. Mean acceleration uses
measured speed change divided by the interval between paired wheel timestamps;
`response_duration_s` also records producer-to-end time.

Software tests, isolated ROS tests and simulated VESC wire tests are verification
of the program. **B/C physical low-current and field acceptance remain to be run.**

[PP and A guide](pp-calibration.md) · [Package interface](../../src/aims_calibration/README.md)

[Verification report](../reports/2026-10-14-calibration-current-sweep.md)
