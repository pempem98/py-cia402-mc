"""CiA 402 homing (mode 6).

Homing is a drive-side procedure: the master selects the method and speeds,
raises the start bit and waits for the "homing attained" bit. It is driven
here rather than in `Axis` because it is a one-shot startup sequence, not a
per-cycle concern.

The controlword must keep flowing through the cyclic PDO while homing runs, so
this module sets the mode and the start bit through the axis's output buffer
and lets the cyclic task transmit it.
"""
from __future__ import annotations

import logging
import struct
import time

from . import cia402 as c
from .axis import Axis, AxisError

log = logging.getLogger(__name__)

#: Methods that home on the current position without moving the axis. Both
#: exist because drives disagree on which one is "set current position".
NO_MOTION_METHODS = frozenset({35, 37})


def configure(axis: Axis) -> None:
    """Write the homing parameters from the axis configuration over SDO."""
    cfg = axis.cfg
    homing = cfg.homing

    axis.sdo_write(c.OD_HOMING_METHOD, 0, struct.pack("<b", homing.method))

    if homing.method not in NO_MOTION_METHODS:
        # 0x6099:1 = speed during search for switch, :2 = speed during search
        # for zero. Both are in the drive's velocity units (counts/s here).
        axis.sdo_write(
            c.OD_HOMING_SPEEDS, 1,
            struct.pack("<I", cfg.velocity_to_counts(homing.speed_search_deg_s)),
        )
        axis.sdo_write(
            c.OD_HOMING_SPEEDS, 2,
            struct.pack("<I", cfg.velocity_to_counts(homing.speed_zero_deg_s)),
        )
        axis.sdo_write(
            c.OD_HOMING_ACCELERATION, 0,
            struct.pack("<I", cfg.velocity_to_counts(homing.acceleration_deg_s2)),
        )
    log.info("%s: homing method %d configured", axis.name, homing.method)


def run(axis: Axis, timeout: float | None = None) -> bool:
    """Home one axis. Returns True on success.

    The axis must already be in OPERATION_ENABLED. The mode is switched to
    homing, the start bit is raised through the cyclic controlword, and the
    statusword is polled for bit 12 (attained) or bit 13 (error).

    The original mode is restored before returning, whatever the outcome.
    """
    cfg = axis.cfg
    timeout = timeout if timeout is not None else cfg.homing.timeout_s

    if not axis.is_enabled:
        raise AxisError(f"{axis.name}: enable the axis before homing")

    previous_mode = axis.mode
    configure(axis)
    axis.set_mode(c.Mode.HOMING)

    # Wait for the drive to acknowledge the mode before starting, otherwise the
    # start bit can be sampled while the drive is still in the old mode.
    if not _wait(lambda: axis.state.mode_display == int(c.Mode.HOMING), 2.0):
        axis.set_mode(previous_mode)
        raise AxisError(
            f"{axis.name}: drive did not enter homing mode "
            f"(mode display = {axis.state.mode_display})"
        )

    log.info("%s: homing started (method %d)", axis.name, cfg.homing.method)
    axis._homing_active = True  # the cyclic loop ORs in the start bit
    try:
        deadline = time.time() + timeout
        while time.time() < deadline:
            status = axis.state.statusword
            if status & c.SW_HOMING_ERROR:
                raise AxisError(
                    f"{axis.name}: homing failed, statusword 0x{status:04X}"
                )
            # Attained (bit 12) together with target reached (bit 10) means the
            # procedure finished; bit 12 alone can appear mid-sequence.
            if status & c.SW_HOMING_ATTAINED and status & c.SW_TARGET_REACHED:
                log.info("%s: homing attained at %.3f deg",
                         axis.name, axis.state.position_deg)
                return True
            time.sleep(0.005)
        raise AxisError(f"{axis.name}: homing timed out after {timeout:.1f} s")
    finally:
        axis._homing_active = False
        axis.set_mode(previous_mode)


def run_all(axes: list[Axis], timeout: float | None = None) -> dict[str, bool]:
    """Home every axis that has homing enabled, one at a time.

    Sequential rather than parallel: a homing move can travel a long way, and
    on a linked mechanism two axes searching at once can collide.
    """
    results: dict[str, bool] = {}
    for axis in axes:
        if not axis.cfg.homing.enabled:
            log.info("%s: homing not enabled, skipping", axis.name)
            continue
        try:
            results[axis.name] = run(axis, timeout)
        except AxisError as exc:
            log.error("%s", exc)
            results[axis.name] = False
    return results


def set_zero_here(axis: Axis) -> None:
    """Make the current position the application zero, without moving.

    This adjusts the master-side offset only; it does not touch the drive's
    own home position. Persist `zero_offset_counts` in the YAML to keep it
    across restarts.

    The drive-side position limits (0x607D) are expressed in raw counts, so
    they are rewritten around the new zero. Otherwise the drive would keep a
    window centred on the old zero, and an axis re-zeroed far from raw 0
    would sit outside its own drive limits.
    """
    axis.cfg.zero_offset_counts = axis.state.position_counts
    axis.reset_fault()
    axis._seed_setpoints()
    if axis.driver is not None:
        axis.driver.apply_motion_limits(axis.slave)
    log.info("%s: zero set at raw count %d", axis.name, axis.cfg.zero_offset_counts)


def _wait(condition, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(0.005)
    return False
