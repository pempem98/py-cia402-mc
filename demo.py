"""Staged motion demo for any bus config (tested: 2x eRob70, 1x Maxon HEJ 70).

Runs a staged sequence that starts slow and only speeds up once each stage has
proved the configuration is right:

  Stage 1  verify  - 2 deg on each axis in turn, at 5 deg/s. Confirms slave
                     addressing, direction and scale before anything larger.
  Stage 2  single  - independent point-to-point moves on each axis.
  Stage 3  coord   - all axes together over unequal distances, starting and
                     finishing at the same moment (one axis: a plain move).
  Stage 4  speed   - the same move at the configured ceiling.

Every stage prompts before it runs, so the sequence can be stopped at the first
sign that something is wrong. Ctrl+C at any point disables every drive.

Usage (Administrator on Windows):

    python demo.py -c configs/demo_1x_hej70.yaml
    python demo.py -c configs/demo_2x_erob70.yaml --yes     # no prompts
    python demo.py -c configs/demo_1x_hej70.yaml --stage 3  # skip ahead
"""
from __future__ import annotations

import argparse
import logging
import sys
import time

from ethercat_mc import MotionController, load_config
from ethercat_mc import homing
from ethercat_mc.axis import AxisError
from ethercat_mc.cli import choose_adapter
from ethercat_mc.master import BusError

CONFIG = "configs/demo_1x_hej70.yaml"

#: The first moves run far below the configured ceiling: if counts_per_rev or
#: direction is wrong, a slow axis can be stopped before it does damage.
VERIFY_SPEED_DEG_S = 5.0
MODERATE_SPEED_DEG_S = 15.0

log = logging.getLogger("demo")


def confirm(prompt: str, auto_yes: bool) -> bool:
    """Ask before moving. Returns False to skip the stage."""
    if auto_yes:
        print(f"\n>>> {prompt}  [auto-yes]")
        return True
    answer = input(f"\n>>> {prompt}  [Enter = go, s = skip, q = quit] ").strip().lower()
    if answer == "q":
        raise KeyboardInterrupt
    return answer != "s"


def show(mc: MotionController) -> None:
    for line in mc.status_lines():
        print("    " + line)


def move_and_wait(mc: MotionController, axis_name: str, target_deg: float,
                  speed_deg_s: float) -> bool:
    """Move one axis and wait for it, reporting what happened."""
    axis = mc.axis(axis_name)
    axis.set_profile_limits(velocity_deg_s=speed_deg_s)

    start = axis.position_deg
    distance = abs(target_deg - start)
    timeout = distance / speed_deg_s + 10.0

    print(f"    {axis_name}: {start:+8.3f} -> {target_deg:+8.3f} deg "
          f"at {speed_deg_s} deg/s")
    axis.move_to(target_deg)

    if not axis.wait_for_target(timeout):
        print(f"    !! {axis_name} did not arrive: {axis.describe()}")
        return False
    print(f"    {axis_name}: arrived at {axis.position_deg:+8.3f} deg "
          f"(error {axis.position_deg - target_deg:+.3f})")
    return True


# --- stages --------------------------------------------------------------

def stage_verify(mc: MotionController, auto_yes: bool) -> bool:
    """Smallest possible move on each axis, to prove the basics."""
    print("\n" + "=" * 68)
    print("STAGE 1 - verification: 2 deg on each axis, one at a time")
    print("=" * 68)
    print("  Watch which physical motor moves, and which way it turns.")
    print("  If the wrong motor moves, slave_position is swapped in the YAML.")
    print("  If it turns the wrong way, flip `direction` to -1.")

    for name in [a.name for a in mc.axes]:
        if not confirm(f"Move {name} by +2 deg at {VERIFY_SPEED_DEG_S} deg/s?",
                       auto_yes):
            continue
        axis = mc.axis(name)
        start = axis.position_deg
        if not move_and_wait(mc, name, start + 2.0, VERIFY_SPEED_DEG_S):
            return False
        if not move_and_wait(mc, name, start, VERIFY_SPEED_DEG_S):
            return False
    return True


def stage_single(mc: MotionController, auto_yes: bool) -> bool:
    """Independent point-to-point moves: each axis commanded on its own."""
    print("\n" + "=" * 68)
    print("STAGE 2 - independent control: each axis moves separately")
    print("=" * 68)

    targets = [(a.name, 30.0 if i % 2 == 0 else -20.0)
               for i, a in enumerate(mc.axes)]
    for name, target in targets:
        if not confirm(f"Move {name} to {target:+.1f} deg at "
                       f"{MODERATE_SPEED_DEG_S} deg/s?", auto_yes):
            continue
        if not move_and_wait(mc, name, target, MODERATE_SPEED_DEG_S):
            return False

    if confirm("Return all axes to 0 deg?", auto_yes):
        for name in [a.name for a in mc.axes]:
            if not move_and_wait(mc, name, 0.0, MODERATE_SPEED_DEG_S):
                return False
    return True


def stage_coordinated(mc: MotionController, auto_yes: bool,
                      speed_deg_s: float, label: str) -> bool:
    """All axes over unequal distances, arriving together.

    This is what the single cyclic loop buys: the first axis travels furthest
    and sets the pace; the others are slowed so every axis ramps up, cruises
    and stops on the same schedule. With one axis it is a plain move.
    """
    print("\n" + "=" * 68)
    print(f"STAGE {label} - coordinated motion at {speed_deg_s} deg/s")
    print("=" * 68)
    distances = {a.name: 60.0 / (i + 1) for i, a in enumerate(mc.axes)}
    print("  " + ", ".join(f"{n} travels {d:.0f} deg" for n, d in distances.items()))
    if len(mc.axes) > 1:
        print("  All axes should start and stop at the same instant.")

    for targets in (distances, {n: 0.0 for n in distances}):
        pretty = ", ".join(f"{n}={v:+.1f}" for n, v in targets.items())
        if not confirm(f"Coordinated move to {pretty}?", auto_yes):
            continue

        started = time.perf_counter()
        duration = mc.move_coordinated(targets, max_velocity_deg_s=speed_deg_s)
        print(f"    planned duration: {duration:.2f} s")

        if not mc.wait_for_targets(list(targets), timeout=duration + 10.0):
            print("    !! not all axes arrived")
            show(mc)
            mc.restore_configured_limits()
            return False

        elapsed = time.perf_counter() - started
        print(f"    arrived in {elapsed:.2f} s "
              f"(planned {duration:.2f} s)")
        for name in targets:
            axis = mc.axis(name)
            print(f"      {name}: {axis.position_deg:+8.3f} deg "
                  f"(error {axis.position_deg - targets[name]:+.3f})")

    mc.restore_configured_limits()
    return True


STAGES = {
    1: ("verification", stage_verify),
    2: ("independent control", stage_single),
    3: ("coordinated motion (moderate)",
        lambda mc, y: stage_coordinated(mc, y, MODERATE_SPEED_DEG_S, "3")),
    4: ("coordinated motion (full speed)", None),  # filled in from the config
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-c", "--config", default=CONFIG)
    parser.add_argument("--yes", action="store_true",
                        help="run every stage without prompting")
    parser.add_argument("--stage", type=int, default=1,
                        help="first stage to run (1-4)")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    cfg = load_config(args.config)
    if not cfg.axes:
        print(f"No axes in {args.config}.")
        return 1

    # Stage 4 runs at whatever ceiling the configuration allows.
    full_speed = min(a.limits.max_velocity_deg_s for a in cfg.axes)
    STAGES[4] = (
        f"coordinated motion (full speed, {full_speed} deg/s)",
        lambda mc, y: stage_coordinated(mc, y, full_speed, "4"),
    )

    print(__doc__.split("Usage")[0].rstrip())
    print(f"\nConfig: {args.config}")
    for a in cfg.axes:
        print(f"  slave {a.slave_position}: {a.name}, "
              f"{a.counts_per_output_rev:,.0f} counts/rev, "
              f"direction {a.direction:+d}, "
              f"limits [{a.limits.min_deg}, {a.limits.max_deg}] deg")
    print("\n  !! Confirm counts_per_rev against the drive manual before")
    print("     the first run. A wrong value scales every angle.")

    adapter = choose_adapter(cfg.adapter)

    with MotionController(cfg) as mc:
        try:
            mc.start(adapter)
            print("\nBus is operational:")
            show(mc)

            # The absolute encoders report wherever the shafts happen to sit
            # (e.g. 342 deg), usually outside the +/-180 deg demo window. The
            # joints are free-standing, so the current pose becomes 0 deg and
            # every move below is relative to it. This also rewrites the
            # drive-side limits around the new zero.
            print("\nSetting the current pose as 0 deg:")
            for axis in mc.axes:
                raw = axis.position_deg
                homing.set_zero_here(axis)
                print(f"    {axis.name}: was {raw:8.3f} deg -> now "
                      f"{axis.position_deg:+.3f} deg")

            if not confirm("Enable all drives?", args.yes):
                print("Nothing enabled; exiting.")
                return 0
            if not mc.enable_all():
                print("!! Could not enable every axis:")
                show(mc)
                return 1
            print("    all axes enabled")

            for number in sorted(STAGES):
                if number < args.stage:
                    continue
                name, fn = STAGES[number]
                if not fn(mc, args.yes):
                    print(f"\n!! Stage {number} ({name}) failed. Stopping.")
                    show(mc)
                    return 1

            print("\n" + "=" * 68)
            print("Demo complete.")
            print("=" * 68)
            show(mc)
            return 0

        except KeyboardInterrupt:
            print("\n\nInterrupted - stopping all axes.")
            mc.stop_all()
            time.sleep(0.2)
            return 130
        except (AxisError, BusError) as exc:
            print(f"\n!! {exc}")
            show(mc)
            return 1


if __name__ == "__main__":
    sys.exit(main())
