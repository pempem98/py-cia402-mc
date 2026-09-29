"""Axis and bus configuration, loaded from YAML.

The configuration is the single place that knows about physical units. Every
other module works in encoder counts; `AxisConfig` converts between counts and
degrees, applying gear ratio, direction and zero offset.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


def _as_int(value: Any) -> int:
    """Accept 0x-prefixed strings as well as plain ints from YAML."""
    if isinstance(value, str):
        return int(value, 0)
    return int(value)


@dataclass
class LimitConfig:
    """Motion limits, in engineering units (degrees, deg/s, deg/s^2)."""

    min_deg: float | None = None
    max_deg: float | None = None
    max_velocity_deg_s: float = 30.0
    max_acceleration_deg_s2: float = 60.0
    #: Maximum single relative command, a guard against typos in the CLI.
    max_step_deg: float = 360.0
    #: Torque limit as a fraction of rated torque (CST mode).
    max_torque_permille: int = 300
    #: Abort motion if the drive reports this following error, in degrees.
    max_following_error_deg: float = 5.0


@dataclass
class HomingConfig:
    """Homing parameters (CiA 402 mode 6)."""

    enabled: bool = False
    method: int = 35  # 35/37 = "current position becomes home", no switch needed
    speed_search_deg_s: float = 5.0
    speed_zero_deg_s: float = 1.0
    acceleration_deg_s2: float = 10.0
    offset_deg: float = 0.0
    timeout_s: float = 30.0


@dataclass
class AxisConfig:
    """Everything needed to drive one motor."""

    name: str
    #: Position of the slave on the bus, 0-based, in physical wiring order.
    slave_position: int
    #: Driver key, e.g. "erob" or "maxon". Selects the vendor class.
    driver: str = "generic"

    # --- identity check ---
    vendor_id: int | None = None
    product_code: int | None = None

    # --- units ---
    #: Encoder counts per revolution of the *motor* shaft, before the gearbox.
    counts_per_rev: int = 524288
    #: Gearbox reduction: motor revolutions per output revolution. 100.0 for
    #: a 100:1 gearhead. Stays 1.0 when the encoder already reads the output
    #: shaft (as on eRob with its 19-bit output encoder).
    gear_ratio: float = 1.0
    #: +1 or -1, to make positive degrees turn the way the application expects.
    direction: int = 1
    #: Encoder count that corresponds to 0 degrees in application coordinates.
    zero_offset_counts: int = 0

    # --- motion ---
    default_mode: str = "csp"
    limits: LimitConfig = field(default_factory=LimitConfig)
    homing: HomingConfig = field(default_factory=HomingConfig)

    #: Rated torque in mNm, used to convert per-mille torque to physical units.
    rated_torque_mnm: float = 0.0

    #: How close the measured position must get to the goal for a CSP move to
    #: count as arrived. Must exceed the drive's real steady-state error, or
    #: moves time out although the axis is where it can get.
    position_tolerance_deg: float = 0.05

    #: Optional explicit PDO layout; None means the driver's default.
    rx_pdo: list[dict] | None = None
    tx_pdo: list[dict] | None = None
    rx_pdo_index: int = 0x1600
    tx_pdo_index: int = 0x1A00

    #: Extra SDO writes applied during PRE-OP, as {index, subindex, type, value}.
    startup_sdo: list[dict] = field(default_factory=list)

    # --- unit conversion ---------------------------------------------------
    @property
    def counts_per_output_rev(self) -> float:
        """Encoder counts per revolution of the OUTPUT shaft.

        With a reduction of N:1 the motor turns N times per output turn, so
        the encoder accumulates N times its per-motor-revolution count.
        """
        return self.counts_per_rev * self.gear_ratio

    def deg_to_counts(self, deg: float) -> int:
        """Application degrees -> raw encoder counts (absolute)."""
        return int(round(
            self.direction * deg * self.counts_per_output_rev / 360.0
        )) + self.zero_offset_counts

    def counts_to_deg(self, counts: int) -> float:
        """Raw encoder counts -> application degrees (absolute)."""
        return (
            self.direction * (counts - self.zero_offset_counts)
            * 360.0 / self.counts_per_output_rev
        )

    def deg_to_counts_delta(self, deg: float) -> int:
        """A *difference* in degrees -> a difference in counts (no offset)."""
        return int(round(self.direction * deg * self.counts_per_output_rev / 360.0))

    def counts_to_deg_delta(self, counts: float) -> float:
        """A *difference* in counts -> a difference in degrees (no offset)."""
        return self.direction * counts * 360.0 / self.counts_per_output_rev

    def velocity_to_counts(self, deg_s: float) -> int:
        """deg/s -> counts/s. Velocity is a rate, so no zero offset applies."""
        return abs(self.deg_to_counts_delta(deg_s))

    def clamp_position_deg(self, deg: float) -> float:
        """Clamp a target to the configured software limits."""
        if self.limits.min_deg is not None:
            deg = max(deg, self.limits.min_deg)
        if self.limits.max_deg is not None:
            deg = min(deg, self.limits.max_deg)
        return deg

    def within_limits(self, deg: float) -> bool:
        if self.limits.min_deg is not None and deg < self.limits.min_deg:
            return False
        if self.limits.max_deg is not None and deg > self.limits.max_deg:
            return False
        return True


@dataclass
class BusConfig:
    """Master-level settings plus the list of axes."""

    #: Network adapter name (pysoem's `name`, not `desc`). None = ask the user.
    adapter: str | None = None
    #: Process-data cycle time in seconds. 0.002 = 2 ms.
    cycle_time_s: float = 0.002
    #: Enable Distributed Clock synchronisation (DC SYNC0).
    use_dc: bool = False
    #: Timeout for one receive_processdata call, in microseconds.
    pdo_timeout_us: int = 2000
    #: How many consecutive bad working counters before the bus is faulted.
    max_wkc_errors: int = 50
    axes: list[AxisConfig] = field(default_factory=list)

    def axis(self, name: str) -> AxisConfig:
        for a in self.axes:
            if a.name == name:
                return a
        raise KeyError(f"no axis named {name!r}")


# --- YAML loading --------------------------------------------------------

def _limits_from(d: dict) -> LimitConfig:
    return LimitConfig(
        min_deg=d.get("min_deg"),
        max_deg=d.get("max_deg"),
        max_velocity_deg_s=d.get("max_velocity_deg_s", 30.0),
        max_acceleration_deg_s2=d.get("max_acceleration_deg_s2", 60.0),
        max_step_deg=d.get("max_step_deg", 360.0),
        max_torque_permille=d.get("max_torque_permille", 300),
        max_following_error_deg=d.get("max_following_error_deg", 5.0),
    )


def _homing_from(d: dict) -> HomingConfig:
    return HomingConfig(
        enabled=d.get("enabled", False),
        method=d.get("method", 35),
        speed_search_deg_s=d.get("speed_search_deg_s", 5.0),
        speed_zero_deg_s=d.get("speed_zero_deg_s", 1.0),
        acceleration_deg_s2=d.get("acceleration_deg_s2", 10.0),
        offset_deg=d.get("offset_deg", 0.0),
        timeout_s=d.get("timeout_s", 30.0),
    )


def _axis_from(d: dict) -> AxisConfig:
    return AxisConfig(
        name=d["name"],
        slave_position=d["slave_position"],
        driver=d.get("driver", "generic"),
        vendor_id=_as_int(d["vendor_id"]) if d.get("vendor_id") is not None else None,
        product_code=(
            _as_int(d["product_code"]) if d.get("product_code") is not None else None
        ),
        counts_per_rev=_as_int(d.get("counts_per_rev", 524288)),
        gear_ratio=float(d.get("gear_ratio", 1.0)),
        direction=int(d.get("direction", 1)),
        zero_offset_counts=_as_int(d.get("zero_offset_counts", 0)),
        default_mode=d.get("default_mode", "csp"),
        limits=_limits_from(d.get("limits", {})),
        homing=_homing_from(d.get("homing", {})),
        rated_torque_mnm=float(d.get("rated_torque_mnm", 0.0)),
        position_tolerance_deg=float(d.get("position_tolerance_deg", 0.05)),
        rx_pdo=d.get("rx_pdo"),
        tx_pdo=d.get("tx_pdo"),
        rx_pdo_index=_as_int(d.get("rx_pdo_index", 0x1600)),
        tx_pdo_index=_as_int(d.get("tx_pdo_index", 0x1A00)),
        startup_sdo=d.get("startup_sdo", []),
    )


def load(path: str | Path) -> BusConfig:
    """Read a bus configuration from a YAML file."""
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    bus = raw.get("bus", {})
    axes = [_axis_from(a) for a in raw.get("axes", [])]

    names = [a.name for a in axes]
    if len(set(names)) != len(names):
        raise ValueError(f"duplicate axis names in {path}: {names}")
    positions = [a.slave_position for a in axes]
    if len(set(positions)) != len(positions):
        raise ValueError(f"duplicate slave_position in {path}: {positions}")

    for a in axes:
        if a.direction not in (1, -1):
            raise ValueError(f"axis {a.name}: direction must be +1 or -1")
        if a.counts_per_rev <= 0:
            raise ValueError(f"axis {a.name}: counts_per_rev must be positive")
        if a.gear_ratio <= 0:
            raise ValueError(f"axis {a.name}: gear_ratio must be positive")

    return BusConfig(
        adapter=bus.get("adapter"),
        cycle_time_s=float(bus.get("cycle_time_s", 0.002)),
        use_dc=bool(bus.get("use_dc", False)),
        pdo_timeout_us=int(bus.get("pdo_timeout_us", 2000)),
        max_wkc_errors=int(bus.get("max_wkc_errors", 50)),
        axes=axes,
    )
