"""MotionController: the application-facing entry point.

Ties the master, the drivers and the axes together, and adds coordinated
motion across axes. One controller owns one EtherCAT segment.
"""
from __future__ import annotations

import logging
import time

from . import cia402 as c
from . import homing as homing_mod
from .axis import Axis, AxisError
from .config import BusConfig
from .drivers import base as drivers
from .master import BusError, EtherCATMaster
from .trajectory import scale_for_duration, sync_duration

log = logging.getLogger(__name__)


def _fit_to_duration(
    distance: float,
    duration: float,
    max_velocity: float,
    max_acceleration: float,
) -> tuple[float, float]:
    """Find limits that make a trapezoidal move of `distance` take `duration`.

    `scale_for_duration` shapes a 1/3-cruise trapezoid, which is a different
    shape from the one `sync_duration` measures, so its result can be off by
    several percent - enough for two axes to arrive visibly apart. Its answer
    is used as the starting guess and then corrected by bisection on the
    velocity, using `sync_duration` itself as the measure. That closes the
    loop: the returned pair is timed by the same function that set the target.
    """
    distance = abs(distance)
    if distance == 0.0 or duration <= 0.0:
        return 0.0, max_acceleration

    _, acceleration = scale_for_duration(
        distance, duration, max_acceleration, max_velocity
    )

    # A slower velocity always means a longer move, so the duration is
    # monotonic in velocity and bisection converges.
    low, high = 1e-9, max_velocity
    if sync_duration(distance, high, acceleration) >= duration:
        # Even at full speed this axis cannot beat `total`; run it flat out.
        return high, acceleration

    for _ in range(60):
        mid = 0.5 * (low + high)
        if sync_duration(distance, mid, acceleration) > duration:
            low = mid
        else:
            high = mid
    return high, acceleration


class MotionController:
    """Brings up a bus of CiA 402 drives and commands them."""

    def __init__(self, cfg: BusConfig):
        self.cfg = cfg
        self.master = EtherCATMaster(cfg)
        self.axes: list[Axis] = []
        self._by_name: dict[str, Axis] = {}
        self._started = False

    # --- lifecycle -------------------------------------------------------
    def start(self, adapter: str | None = None) -> None:
        """Full bring-up: open, scan, configure, SAFE-OP, cyclic, OP."""
        self.master.open(adapter)
        count = self.master.scan()
        # Fail here with a clear message rather than as a WkcError halfway
        # through writing the PDO mapping.
        self.master.check_mailbox_health()

        highest = max((a.slave_position for a in self.cfg.axes), default=-1)
        if highest >= count:
            raise BusError(
                f"configuration references slave {highest} but only {count} "
                f"slaves are on the bus"
            )

        # Build one driver per axis and hand SOEM its PRE-OP hook.
        config_funcs: list = [None] * count
        pending: list[tuple] = []
        for axis_cfg in self.cfg.axes:
            slave = self.master.master.slaves[axis_cfg.slave_position]
            driver = drivers.get_driver(axis_cfg)
            driver.check_identity(slave)
            config_funcs[axis_cfg.slave_position] = driver.make_config_func(slave)
            pending.append((axis_cfg, slave, driver))

        self.master.configure(config_funcs)

        # PDO mapping is live now, so the axis objects can be built.
        for axis_cfg, slave, driver in pending:
            axis = Axis(axis_cfg, slave, driver.rx_pdo(), driver.tx_pdo())
            driver.apply_motion_limits(slave)
            self.axes.append(axis)
            self._by_name[axis.name] = axis

        task = self.master.start_cyclic()
        for axis in self.axes:
            task.add_callback(axis.on_cycle)

        self.master.go_operational()
        self._started = True

        # Let a few cycles run so every axis has a real statusword before the
        # application starts making decisions from it.
        time.sleep(0.05)
        for axis in self.axes:
            log.info("%s", axis.describe())

    def stop(self) -> None:
        """Disable every axis, then take the bus down. Safe to call twice."""
        for axis in self.axes:
            try:
                axis.stop()
                axis.request_disable()
            except Exception:  # noqa: BLE001 - shutdown must continue
                log.warning("%s: error while disabling", axis.name, exc_info=True)
        if self.axes:
            # Give the cyclic loop time to transmit the disable controlword.
            time.sleep(0.1)
        self.master.close()
        self._started = False

    def __enter__(self) -> "MotionController":
        return self

    def __exit__(self, *exc_info) -> None:
        self.stop()

    # --- access ----------------------------------------------------------
    def axis(self, name: str) -> Axis:
        try:
            return self._by_name[name]
        except KeyError:
            raise KeyError(
                f"no axis named {name!r}; configured: "
                f"{', '.join(self._by_name)}"
            ) from None

    def __getitem__(self, name: str) -> Axis:
        return self.axis(name)

    @property
    def healthy(self) -> bool:
        """True when the cyclic task is running and no axis has latched a fault."""
        task = self.master.task
        return bool(
            task
            and task.running
            and not task.faulted.is_set()
            and not any(a.fault_reason for a in self.axes)
        )

    def check_health(self) -> None:
        """Raise if the bus or any axis has faulted. Call from command paths."""
        task = self.master.task
        if task is None or not task.running:
            raise BusError("cyclic task is not running")
        if task.faulted.is_set():
            raise BusError(f"bus faulted: {task.last_error}")
        faults = [
            f"{a.name}: {a.fault_reason}" for a in self.axes if a.fault_reason
        ]
        if faults:
            raise AxisError("; ".join(faults))

    # --- group operations ------------------------------------------------
    def enable_all(self, timeout: float = 5.0) -> bool:
        """Enable every axis. Returns True only if all of them made it."""
        for axis in self.axes:
            axis.request_enable()
        ok = True
        for axis in self.axes:
            if not axis.wait_enabled(timeout):
                log.error("%s: failed to enable (%s)",
                          axis.name,
                          axis.fault_reason or c.describe_statusword(
                              axis.state.statusword))
                ok = False
        return ok

    def disable_all(self) -> None:
        for axis in self.axes:
            axis.request_disable()

    def set_mode_all(self, mode: c.Mode | str) -> None:
        for axis in self.axes:
            axis.set_mode(mode)

    def stop_all(self) -> None:
        """Decelerate every axis to a stop, staying enabled."""
        for axis in self.axes:
            axis.stop()

    def quick_stop_all(self) -> None:
        """Trigger the drive-side quick stop on every axis."""
        for axis in self.axes:
            axis.quick_stop()

    def home_all(self, timeout: float | None = None) -> dict[str, bool]:
        return homing_mod.run_all(self.axes, timeout)

    # --- coordinated motion ----------------------------------------------
    def move_coordinated(
        self,
        targets: dict[str, float],
        duration: float | None = None,
    ) -> float:
        """Move several axes so they start and finish together.

        Each axis would need a different time for its own move; the slowest one
        sets the duration, and the others are scaled down to match. Returns the
        duration in seconds.

        `duration` forces a specific time instead of using the slowest axis.
        A duration shorter than an axis can achieve is rejected rather than
        silently clipped, because a partially-scaled group is no longer
        coordinated.
        """
        self.check_health()

        plan = []
        for name, target_deg in targets.items():
            axis = self.axis(name)
            if axis.mode is not c.Mode.CSP:
                raise AxisError(
                    f"{name}: coordinated motion needs CSP, mode is "
                    f"{axis.mode.name}"
                )
            if not axis.cfg.within_limits(target_deg):
                raise AxisError(
                    f"{name}: target {target_deg:.3f} deg is outside limits"
                )
            target_counts = axis.cfg.deg_to_counts(target_deg)
            distance = abs(target_counts - axis.profile.position)
            plan.append((axis, target_counts, distance))

        # The natural duration of each move, at that axis's configured limits.
        natural = [
            sync_duration(distance, axis.profile.max_velocity,
                          axis.profile.max_acceleration)
            for axis, _, distance in plan
        ]
        total = duration if duration is not None else max(natural, default=0.0)
        if total <= 0.0:
            return 0.0

        if duration is not None:
            too_fast = [
                axis.name for (axis, _, _), t in zip(plan, natural) if t > duration
            ]
            if too_fast:
                raise AxisError(
                    f"duration {duration:.3f} s is shorter than axes "
                    f"{', '.join(too_fast)} can achieve"
                )

        # The axis that sets the pace keeps its configured limits; the others
        # are slowed to match its duration. Scaling every axis (including the
        # slowest) would change the slowest one's shape too, and it would no
        # longer take `total` - the arrivals would drift apart.
        slowest_index = natural.index(max(natural)) if natural else -1

        for i, (axis, target_counts, distance) in enumerate(plan):
            velocity_limit = axis.cfg.velocity_to_counts(
                axis.cfg.limits.max_velocity_deg_s
            )
            acceleration_limit = axis.cfg.velocity_to_counts(
                axis.cfg.limits.max_acceleration_deg_s2
            )

            if i == slowest_index and duration is None:
                # This axis defines `total`, so it runs exactly as configured.
                velocity, acceleration = velocity_limit, acceleration_limit
            else:
                velocity, acceleration = _fit_to_duration(
                    distance, total, velocity_limit, acceleration_limit
                )

            # A zero-length move returns no usable limits; leave that axis's
            # profile alone so it simply holds position.
            if velocity > 0:
                axis.profile.max_velocity = velocity
                axis.profile.max_acceleration = acceleration
            axis.profile.set_goal(target_counts)

        log.info("Coordinated move over %.3f s: %s", total,
                 ", ".join(f"{n}->{v:.2f}" for n, v in targets.items()))
        return total

    def wait_for_targets(
        self, names: list[str] | None = None, timeout: float = 30.0
    ) -> bool:
        """Block until the named axes (default: all) reach their targets."""
        axes = [self.axis(n) for n in names] if names else self.axes
        deadline = time.time() + timeout
        while time.time() < deadline:
            self.check_health()
            if all(a.at_target for a in axes):
                return True
            time.sleep(0.002)
        return False

    def restore_configured_limits(self) -> None:
        """Undo the per-move scaling from `move_coordinated`."""
        for axis in self.axes:
            axis.profile.max_velocity = axis.cfg.velocity_to_counts(
                axis.cfg.limits.max_velocity_deg_s
            )
            axis.profile.max_acceleration = axis.cfg.velocity_to_counts(
                axis.cfg.limits.max_acceleration_deg_s2
            )

    # --- diagnostics -----------------------------------------------------
    def status_lines(self) -> list[str]:
        lines = [axis.describe() for axis in self.axes]
        task = self.master.task
        if task:
            lines.append(
                f"bus: {task.cycle_count} cycles, WKC {task.actual_wkc}/"
                f"{task.expected_wkc}, max jitter {task.max_jitter_s * 1e3:.2f} ms"
            )
        return lines
