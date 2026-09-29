# ethercat_mc — multi-axis EtherCAT motion control

Python motion control for CiA 402 drives over EtherCAT, built for a mixed bus
of ZeroErr eRob joints and Maxon EPOS4/IDX (HEJ) drives.

## Layout

| Module | Responsibility |
| --- | --- |
| [cia402.py](ethercat_mc/cia402.py) | CiA 402 object indices, statusword decoding, enable sequence. Vendor neutral. |
| [pdo.py](ethercat_mc/pdo.py) | PDO entry descriptions, mapping values, pack/unpack of the process image. |
| [config.py](ethercat_mc/config.py) | YAML loading, and the only place that knows physical units. |
| [trajectory.py](ethercat_mc/trajectory.py) | Setpoint generators (trapezoidal, velocity ramp, torque ramp). Pure maths. |
| [master.py](ethercat_mc/master.py) | Bus bring-up and the single cyclic process-data loop. |
| [axis.py](ethercat_mc/axis.py) | One axis: state machine, setpoints, safety latches. |
| [homing.py](ethercat_mc/homing.py) | CiA 402 homing (mode 6). |
| [controller.py](ethercat_mc/controller.py) | Application entry point and coordinated motion. |
| [drivers/](ethercat_mc/drivers/) | Vendor specifics: `erob`, `maxon`, `generic`. |
| [cli.py](ethercat_mc/cli.py) | `scan`, `info`, `home`, `run`. |

### The rule that shapes the design

**One process-data exchange per cycle, for the whole bus.** `send_processdata`
and `receive_processdata` appear only in [master.py](ethercat_mc/master.py).
Axes never touch the wire; they write into their output buffer and read their
input buffer, and the cyclic task moves both. The previous single-motor script
called both from inside a per-motor thread, which cannot extend to more than
one drive.

## Getting started

```bash
.venv/Scripts/python.exe -m pip install pysoem pyyaml pytest

# 1. What is on the bus? (Administrator / root required)
python -m ethercat_mc.cli scan
python -m ethercat_mc.cli scan --adapter "\Device\NPF_{...}"

# 2. Copy the example and edit it to match the hardware.
cp configs/example_mixed.yaml configs/robot.yaml

# 3. Check the config reads the way you meant, without touching the bus.
python -m ethercat_mc.cli info -c configs/robot.yaml

# 4. Drive it.
python -m ethercat_mc.cli run -c configs/robot.yaml
```

Windows needs Npcap installed with **WinPcap API-compatible mode** ticked, and
the terminal must run as Administrator.

## Configuring an axis

The values that matter most, because getting them wrong moves real hardware the
wrong way or the wrong distance:

```yaml
counts_per_rev: 524288   # encoder counts per MOTOR revolution
gear_ratio: 1.0          # motor revolutions per OUTPUT revolution
direction: 1             # +1 or -1
zero_offset_counts: 0    # raw count that means 0 degrees
```

`counts_per_output_rev = counts_per_rev * gear_ratio`.

* **eRob** reads an absolute encoder on the *output* shaft, so the harmonic
  drive is already accounted for: `gear_ratio: 1.0`.
* **Maxon HEJ** reads the *motor* shaft, so `gear_ratio` must be the real
  reduction (100.0 for 100:1) and `counts_per_rev` is 4× the encoder line
  count for a quadrature encoder.

`info` prints the resulting resolution in arcsec/count — check it against the
datasheet before enabling anything.

## Modes

`csp` is the default and the one to use for coordinated motion; the master
generates the trajectory and sends a position every cycle. `csv` and `cst` are
for outer control loops (force, teleoperation). `homing` runs the drive's own
homing procedure.

```
mode joint1 csv
vel joint1 5.0        # 5 deg/s
mode joint1 csp
```

## Demo

[demo.py](demo.py) runs any bus config through four stages that escalate only
once each has proved the configuration:

| Config | Hardware |
| --- | --- |
| [configs/demo_1x_hej70.yaml](configs/demo_1x_hej70.yaml) | one Maxon HEJ 70 (EPOS4), 4 ms cycle |
| [configs/demo_2x_erob70.yaml](configs/demo_2x_erob70.yaml) | two eRob70, 2 ms cycle |

| Stage | What it does | Speed |
| --- | --- | --- |
| 1 | 2 deg on each axis in turn | 5 deg/s |
| 2 | independent point-to-point moves | 15 deg/s |
| 3 | 60 deg (and 30 deg on a second axis), arriving together | 15 deg/s |
| 4 | the same move at the ceiling | 30 deg/s |

```bash
python demo.py -c configs/demo_1x_hej70.yaml             # prompts before each stage
python demo.py -c configs/demo_2x_erob70.yaml --yes      # no prompts
python demo.py -c configs/demo_1x_hej70.yaml --stage 3   # skip ahead
```

The demo takes the current pose as 0 deg for every axis before enabling.

Stage 1 exists because the two things most likely to be wrong —
`counts_per_rev` and `direction` — are both cheap to check with a 2 deg move
and expensive to discover at 30 deg/s. Watch which motor moves (proves
`slave_position`) and which way (proves `direction`).

## Coordinated motion

`move_coordinated` scales every axis onto a common duration set by the slowest,
so a multi-joint move starts and ends together:

```python
duration = mc.move_coordinated({"joint1": 30.0, "joint2": -10.0})
mc.wait_for_targets(timeout=duration + 5)
mc.restore_configured_limits()
```

From the shell: `move joint1=30 joint2=-10`.

The axis that sets the pace keeps its configured limits; the others are fitted
to its duration by bisection on `sync_duration`, which is the same function
that measured the duration in the first place. Scaling every axis with the
open-loop formula instead left the arrivals ~7% apart; closing the loop this
way brings two eRob70 axes (60 deg and 20 deg over 2.5 s) to within 2 ms.

## Safety model

Four independent layers, because any one of them can be defeated:

1. **Command validation** — `move_to` rejects targets outside `min_deg`/
   `max_deg`; `move_by` rejects steps over `max_step_deg` (a typo guard).
2. **Setpoint generation** — the profile never exceeds the configured velocity
   or acceleration, whatever is commanded.
3. **Cyclic monitoring** — a following error over `max_following_error_deg`
   latches a fault and stops the axis. So does travelling *further* outside the
   software limits.
4. **Drive-side limits** — the same numbers are written into the drive's own
   objects (0x607D, 0x607F, and 0x6065/0x6080 on Maxon), so the drive still
   protects itself if the master stalls.

**Recovery matters as much as tripping.** An axis parked outside its limits
still enables, and motion back toward the window is allowed — otherwise an axis
that drifted out after a collision could never be driven back. What trips is
moving *deeper* out. A latched fault is cleared by `reset_fault()`, or by
`request_enable()`, which is an explicit operator action.

## Real-time behaviour

This is best-effort Python on Windows, not a hard real-time system. The cyclic
task raises the Windows timer resolution to 1 ms and schedules on an absolute
clock so work time does not accumulate as drift, but the OS can still preempt
it. `status_lines()` reports the worst jitter seen — watch it.

If jitter is a problem:

* raise `cycle_time_s` to 4 ms;
* enable `use_dc` so the drives interpolate on their own clock;
* prefer `pp` for point-to-point moves, which puts the trajectory in the drive.

For genuinely hard real-time, the parts to move to C/Linux PREEMPT_RT are
[master.py](ethercat_mc/master.py) and [trajectory.py](ethercat_mc/trajectory.py);
everything else is configuration and supervision.

## Tests

```bash
python -m pytest
```

142 tests, no hardware required. [test_axis.py](tests/test_axis.py) runs the
axis against a simulated CiA 402 drive, so the enable sequence, mode switching
and the safety latches are all covered.

## Bringing up new hardware

1. `scan` — confirm slave order, vendor and product IDs. Pin them in the config
   so a rewired bus is caught instead of moving the wrong joint.
2. `info` — check counts/rev and resolution against the datasheet.
3. Set `min_deg`/`max_deg` tight, and `max_velocity_deg_s` low (5 deg/s).
4. `run`, then a small move: `joint1 1`.
5. Confirm the direction, then widen the limits.

The old single-motor scripts are kept in [legacy/](legacy/) for reference.
