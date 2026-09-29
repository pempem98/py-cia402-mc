"""Command-line interface.

    python -m ethercat_mc.cli scan
    python -m ethercat_mc.cli info    -c configs/example_mixed.yaml
    python -m ethercat_mc.cli run     -c configs/example_mixed.yaml
    python -m ethercat_mc.cli home    -c configs/example_mixed.yaml

Needs Administrator on Windows (Npcap) or root/CAP_NET_RAW on Linux.
"""
from __future__ import annotations

import argparse
import logging
import struct
import sys
import time

from . import cia402 as c
from . import homing as homing_mod
from .axis import AxisError
from .config import load
from .controller import MotionController
from .drivers import available_drivers
from .master import BusError, find_adapters

log = logging.getLogger("ethercat_mc")


def setup_logging(verbose: bool) -> None:
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def choose_adapter(preset: str | None) -> str:
    """Return the adapter name, prompting only when the config has none."""
    adapters = find_adapters()
    if not adapters:
        raise SystemExit(
            "No network adapters found. On Windows, install Npcap with "
            "'WinPcap API-compatible mode' ticked."
        )
    if preset:
        names = [name for name, _ in adapters]
        if preset not in names:
            print(f"Configured adapter {preset!r} not found. Available:")
            for name, desc in adapters:
                print(f"  {name}\n      {desc}")
            raise SystemExit(1)
        return preset

    print("Network adapters:")
    for i, (name, desc) in enumerate(adapters):
        print(f"  [{i}] {desc}\n      {name}")
    while True:
        try:
            choice = int(input("Adapter index: "))
            return adapters[choice][0]
        except (ValueError, IndexError):
            print("  Enter one of the indices above.")


# --- commands ------------------------------------------------------------

def cmd_scan(args) -> int:
    """List adapters and, with one chosen, every slave on that segment."""
    import pysoem

    adapters = find_adapters()
    if not args.adapter and not args.all:
        print("Network adapters:")
        for name, desc in adapters:
            print(f"  {desc}\n      {name}")
        print("\nRe-run with --adapter <name> to enumerate slaves.")
        return 0

    targets = [a[0] for a in adapters] if args.all else [args.adapter]
    for name in targets:
        print(f"\n=== {name} ===")
        master = pysoem.Master()
        try:
            master.open(name)
        except Exception as exc:  # noqa: BLE001 - adapter may be unusable
            print(f"  cannot open: {exc}")
            continue
        try:
            count = master.config_init()
            if count <= 0:
                print("  no slaves found")
                continue
            print(f"  {count} slave(s):")
            for i, s in enumerate(master.slaves):
                print(f"\n  [{i}] {s.name}")
                print(f"      vendor 0x{s.man:08X}  product 0x{s.id:08X}  "
                      f"rev 0x{s.rev:08X}")
                for index, sub, fmt, label in [
                    (0x1008, 0, "str", "device name"),
                    (0x100A, 0, "str", "software version"),
                    (c.OD_STATUSWORD, 0, "<H", "statusword"),
                    (c.OD_MODE_OF_OP_DISPLAY, 0, "<b", "mode"),
                    (c.OD_POSITION_ACTUAL, 0, "<i", "position"),
                    (c.OD_ERROR_CODE, 0, "<H", "error code"),
                ]:
                    try:
                        raw = s.sdo_read(index, sub)
                        if fmt == "str":
                            value = raw.decode(errors="ignore").strip("\x00")
                        else:
                            value = struct.unpack(
                                fmt, raw[:struct.calcsize(fmt)]
                            )[0]
                            if label in ("statusword", "error code"):
                                value = f"0x{value:04X}"
                        print(f"      {label:<18} {value}")
                    except Exception as exc:  # noqa: BLE001
                        print(f"      {label:<18} <unreadable: {exc}>")
        finally:
            master.close()
    return 0


def cmd_linktest(args) -> int:
    """Measure mailbox reliability per slave. Read-only, nothing moves."""
    from .config import BusConfig
    from .master import EtherCATMaster

    adapter = choose_adapter(args.adapter)
    master = EtherCATMaster(BusConfig(adapter=adapter))
    try:
        master.open()
        master.scan()
        worst = 1.0
        for i, ok, total in master.mailbox_health(args.reads):
            ratio = ok / total
            worst = min(worst, ratio)
            flag = "OK" if ratio >= 0.95 else "UNRELIABLE"
            print(f"  slave {i}: {ok}/{total} replies ({ratio:.0%})  {flag}")
        if worst < 0.95:
            print("\nSome requests went unanswered. Make sure no other EtherCAT "
                  "master (TwinCAT) is using this adapter, then re-run.")
            return 1
        print("\nLink looks healthy.")
        return 0
    finally:
        master.close()


def cmd_info(args) -> int:
    """Show what the configuration file means, without touching the bus."""
    cfg = load(args.config)
    print(f"Bus: cycle {cfg.cycle_time_s * 1e3:.1f} ms, "
          f"DC {'on' if cfg.use_dc else 'off'}, "
          f"adapter {cfg.adapter or '<prompt>'}")
    print(f"Drivers available: {', '.join(available_drivers())}\n")
    for a in cfg.axes:
        print(f"[{a.slave_position}] {a.name}  ({a.driver})")
        print(f"     {a.counts_per_output_rev:,.0f} counts per output rev "
              f"({a.counts_per_rev:,} x {a.gear_ratio:g} gear), "
              f"direction {a.direction:+d}")
        print(f"     resolution {360.0 / a.counts_per_output_rev * 3600:.2f} "
              f"arcsec/count")
        limits = a.limits
        span = (
            f"[{limits.min_deg}, {limits.max_deg}] deg"
            if limits.min_deg is not None else "unbounded"
        )
        print(f"     limits {span}, max {limits.max_velocity_deg_s} deg/s, "
              f"{limits.max_acceleration_deg_s2} deg/s^2")
        print(f"     mode {a.default_mode}, homing "
              f"{'method ' + str(a.homing.method) if a.homing.enabled else 'off'}")
    return 0


def cmd_home(args) -> int:
    """Bring the bus up, enable and home every axis that asks for it."""
    cfg = load(args.config)
    adapter = choose_adapter(cfg.adapter)
    with MotionController(cfg) as mc:
        mc.start(adapter)
        if not mc.enable_all():
            print("Could not enable every axis; aborting.")
            return 1
        results = mc.home_all()
        if not results:
            print("No axis has homing enabled.")
            return 0
        for name, ok in results.items():
            print(f"  {name}: {'homed' if ok else 'FAILED'}")
        return 0 if all(results.values()) else 1


HELP = """
Commands:
  <axis> <deg>        relative move, e.g.  joint1 15
  <axis> = <deg>      absolute move, e.g.  joint1 = 90
  move <a>=<d> ...    coordinated move,    move joint1=30 joint2=-10
  v <axis> <deg/s>    set profile speed
  mode <axis> <m>     csp | csv | cst | pp | homing
  vel <axis> <deg/s>  CSV velocity command (needs mode csv)
  trq <axis> <‰>      CST torque command in per-mille (needs mode cst)
  home <axis>         run homing on one axis
  zero <axis>         make the current position zero
  stop                decelerate every axis
  enable / disable    CiA 402 enable state
  s                   status
  q                   quit
""".strip()


def cmd_run(args) -> int:
    """Interactive shell against a live bus."""
    cfg = load(args.config)
    adapter = choose_adapter(cfg.adapter)

    with MotionController(cfg) as mc:
        mc.start(adapter)
        print("\n".join(mc.status_lines()))
        if not mc.enable_all():
            print("Could not enable every axis.")
            for line in mc.status_lines():
                print("  " + line)
            return 1

        print("\n" + HELP)
        while True:
            try:
                raw = input("\n> ").strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if not raw:
                continue
            try:
                if not _dispatch(mc, raw):
                    break
            except (AxisError, BusError, KeyError, ValueError) as exc:
                print(f"  ! {exc}")
            except Exception as exc:  # noqa: BLE001 - keep the shell alive
                print(f"  ! unexpected: {exc}")
                log.debug("command failed", exc_info=True)
    return 0


def _dispatch(mc: MotionController, raw: str) -> bool:
    """Run one shell command. Returns False to quit."""
    parts = raw.split()
    head = parts[0].lower()

    if head in ("q", "quit", "exit"):
        return False
    if head in ("h", "help", "?"):
        print(HELP)
        return True
    if head == "s":
        for line in mc.status_lines():
            print("  " + line)
        return True
    if head == "stop":
        mc.stop_all()
        print("  stopping")
        return True
    if head == "enable":
        print("  enabled" if mc.enable_all() else "  FAILED to enable")
        return True
    if head == "disable":
        mc.disable_all()
        print("  disabled")
        return True

    if head == "move":
        targets = {}
        for token in parts[1:]:
            name, _, value = token.partition("=")
            if not value:
                raise ValueError(f"expected axis=degrees, got {token!r}")
            targets[name] = float(value)
        duration = mc.move_coordinated(targets)
        print(f"  coordinated move, {duration:.2f} s")
        if mc.wait_for_targets(list(targets), timeout=duration + 10.0):
            print("  reached")
        else:
            print("  NOT reached in time")
        mc.restore_configured_limits()
        return True

    if head == "mode":
        mc.axis(parts[1]).set_mode(parts[2])
        print(f"  {parts[1]} -> {parts[2]}")
        return True

    if head == "v":
        mc.axis(parts[1]).set_profile_limits(velocity_deg_s=float(parts[2]))
        print(f"  {parts[1]} speed = {parts[2]} deg/s")
        return True

    if head == "vel":
        mc.axis(parts[1]).set_velocity(float(parts[2]))
        print(f"  {parts[1]} velocity = {parts[2]} deg/s")
        return True

    if head == "trq":
        mc.axis(parts[1]).set_torque(float(parts[2]))
        print(f"  {parts[1]} torque = {parts[2]} per-mille")
        return True

    if head == "home":
        axis = mc.axis(parts[1])
        ok = homing_mod.run(axis)
        print(f"  {parts[1]}: {'homed' if ok else 'FAILED'}")
        return True

    if head == "zero":
        axis = mc.axis(parts[1])
        homing_mod.set_zero_here(axis)
        print(f"  {parts[1]} zeroed, now {axis.position_deg:.3f} deg")
        return True

    # Bare axis command: "joint1 15" or "joint1 = 90".
    axis = mc.axis(head)
    rest = " ".join(parts[1:])
    if rest.startswith("="):
        target = float(rest[1:])
        axis.move_to(target)
    else:
        target = axis.position_deg + float(rest)
        axis.move_by(float(rest))
    print(f"  {head} -> {target:.3f} deg")
    if axis.wait_for_target(timeout=abs(float(rest.lstrip('='))) / 5.0 + 10.0):
        print(f"  reached {axis.position_deg:.3f} deg")
    else:
        print(f"  NOT reached: {axis.describe()}")
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="ethercat_mc",
        description="Multi-axis EtherCAT motion control (eRob, Maxon).",
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)

    p_scan = sub.add_parser("scan", help="list adapters and slaves")
    p_scan.add_argument("--adapter", help="adapter name to enumerate")
    p_scan.add_argument("--all", action="store_true",
                        help="enumerate every adapter")
    p_scan.set_defaults(func=cmd_scan)

    p_link = sub.add_parser("linktest", help="measure mailbox reliability")
    p_link.add_argument("--adapter", help="adapter name (prompted if omitted)")
    p_link.add_argument("--reads", type=int, default=200)
    p_link.set_defaults(func=cmd_linktest)

    p_info = sub.add_parser("info", help="explain a config file, offline")
    p_info.add_argument("-c", "--config", required=True)
    p_info.set_defaults(func=cmd_info)

    p_home = sub.add_parser("home", help="bring up and home every axis")
    p_home.add_argument("-c", "--config", required=True)
    p_home.set_defaults(func=cmd_home)

    p_run = sub.add_parser("run", help="interactive motion shell")
    p_run.add_argument("-c", "--config", required=True)
    p_run.set_defaults(func=cmd_run)

    args = parser.parse_args(argv)
    setup_logging(args.verbose)
    try:
        return args.func(args)
    except (BusError, AxisError) as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
