"""One controlled axis: CiA 402 state machine, setpoint generation, safety.

`Axis.on_cycle` runs inside the cyclic thread and must never block. Command
methods (`move_to`, `set_velocity`, ...) are called from the application thread
and only write plain fields, which the cyclic thread reads on its next pass.
Python's GIL makes single-field assignment atomic, so no lock is needed for
scalars; the mode switch uses a small lock because it updates several fields
that must be seen together.
"""
from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field

import pysoem

from . import cia402 as c
from .config import AxisConfig
from .pdo import PdoMap
from .trajectory import TorqueRamp, TrapezoidalProfile, VelocityRamp

log = logging.getLogger(__name__)


class AxisError(RuntimeError):
    """Raised for axis-level faults: enable failures, limit violations."""


@dataclass
class AxisState:
    """Snapshot of an axis, safe to read from any thread."""

    statusword: int = 0
    position_counts: int = 0
    velocity_counts_s: int = 0
    torque_permille: int = 0
    mode_display: int = 0
    position_deg: float = 0.0
    velocity_deg_s: float = 0.0
    state: c.State = c.State.UNKNOWN
    following_error_deg: float = 0.0


class Axis:
    """Drives one CiA 402 slave.

    The axis owns its slice of the process image and one setpoint generator per
    cyclic mode. It does not touch the bus; `on_cycle` is called by the master's
    cyclic task with the slave's input bytes already refreshed.
    """

    def __init__(
        self,
        cfg: AxisConfig,
        slave: pysoem.CdefSlave,
        rx_pdo: PdoMap,
        tx_pdo: PdoMap,
    ):
        self.cfg = cfg
        self.slave = slave
        self.rx_pdo = rx_pdo
        self.tx_pdo = tx_pdo
        self.name = cfg.name

        # --- values published to the drive each cycle ---
        self._outputs: dict[str, int] = {
            "controlword": 0,
            "target_position": 0,
            "target_velocity": 0,
            "target_torque": 0,
            "mode_of_operation": int(self._mode_from_name(cfg.default_mode)),
        }

        # --- latest values read back from the drive ---
        self.state = AxisState()
        self._inputs: dict[str, int] = {}

        # --- setpoint generators ---
        limits = cfg.limits
        self.profile = TrapezoidalProfile(
            max_velocity=cfg.velocity_to_counts(limits.max_velocity_deg_s),
            max_acceleration=cfg.velocity_to_counts(limits.max_acceleration_deg_s2),
            tolerance=max(1.0, abs(cfg.deg_to_counts_delta(0.001))),
        )
        self.velocity_ramp = VelocityRamp(
            max_velocity=cfg.velocity_to_counts(limits.max_velocity_deg_s),
            max_acceleration=cfg.velocity_to_counts(limits.max_acceleration_deg_s2),
        )
        self.torque_ramp = TorqueRamp(max_torque=limits.max_torque_permille)

        self.mode = self._mode_from_name(cfg.default_mode)
        self._mode_lock = threading.Lock()

        #: Set once the axis has reached OPERATION_ENABLED at least once.
        self._enabled = threading.Event()
        #: Latched fault description; cleared by reset_fault().
        self.fault_reason: str | None = None
        #: When True the cyclic loop drives the enable sequence.
        self._want_enable = False
        #: True while a halt is commanded (controlword bit 8).
        self._halt = False
        self._first_cycle = True
        self._homing_active = False
        #: The vendor driver, set by MotionController, so drive-side settings
        #: that depend on the zero offset can be rewritten after re-zeroing.
        self.driver = None
        #: Log the "outside limits" warning once, not every cycle.
        self._outside_limits_warned = False
        #: Distance outside the limit window on the previous cycle, in degrees.
        self._previous_excursion: float | None = None
        #: True while a drive-reported FAULT has already been recorded, so the
        #: reason is not rewritten every cycle.
        self._drive_fault_latched = False

    # --- helpers ---------------------------------------------------------
    @staticmethod
    def _mode_from_name(name: str) -> c.Mode:
        try:
            return c.Mode[name.strip().upper()]
        except KeyError:
            raise ValueError(
                f"unknown mode {name!r}; expected one of "
                f"{', '.join(m.name.lower() for m in c.Mode)}"
            ) from None

    @property
    def position_deg(self) -> float:
        return self.state.position_deg

    @property
    def is_enabled(self) -> bool:
        return self.state.state is c.State.OPERATION_ENABLED

    @property
    def is_faulted(self) -> bool:
        return (
            self.state.state in (c.State.FAULT, c.State.FAULT_REACTION_ACTIVE)
            or self.fault_reason is not None
        )

    # --- SDO access (application thread only; blocks on the mailbox) ------
    def sdo_read(self, index: int, subindex: int = 0) -> bytes:
        return self.slave.sdo_read(index, subindex)

    def sdo_write(self, index: int, subindex: int, data: bytes) -> None:
        self.slave.sdo_write(index, subindex, data)

    def read_error_code(self) -> int:
        import struct

        return struct.unpack("<H", self.slave.sdo_read(c.OD_ERROR_CODE, 0)[:2])[0]

    # --- commands (application thread) -----------------------------------
    def request_enable(self) -> None:
        """Ask the cyclic loop to run the enable sequence.

        Clears any latched fault first, so a single call recovers an axis that
        tripped: the state machine then issues a fault reset if the drive is
        still in FAULT.
        """
        self.reset_fault()
        self._want_enable = True

    def reset_fault(self) -> None:
        """Clear the latched fault so the axis can be commanded again.

        The latch exists to stop motion after a trip; clearing it is an
        explicit operator decision, which is why it is not automatic.
        """
        self.fault_reason = None
        self._outside_limits_warned = False
        self._previous_excursion = None

    def request_disable(self) -> None:
        self._want_enable = False

    def wait_enabled(self, timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_enabled:
                return True
            if self.fault_reason:
                return False
            time.sleep(0.005)
        return False

    def set_mode(self, mode: c.Mode | str) -> None:
        """Switch the mode of operation. The setpoint generators are re-seeded
        from the measured position so the switch does not cause a jump."""
        if isinstance(mode, str):
            mode = self._mode_from_name(mode)
        if not self.rx_pdo.has("mode_of_operation"):
            # Not mapped (e.g. the eRob vendor layout): set it over SDO. This
            # blocks on the mailbox, which is fine on the application thread.
            import struct

            self.slave.sdo_write(c.OD_MODE_OF_OP, 0, struct.pack("<b", int(mode)))
            # 0x6061 is not in the TxPDO either, so read the drive's answer
            # back once; `describe()` and callers rely on mode_display.
            try:
                raw = self.slave.sdo_read(c.OD_MODE_OF_OP_DISPLAY, 0)
                self.state.mode_display = struct.unpack("<b", raw[:1])[0]
            except Exception:  # noqa: BLE001 - display only
                log.debug("%s: could not read back 0x6061", self.name)
        with self._mode_lock:
            self.mode = mode
            self._outputs["mode_of_operation"] = int(mode)
            self._seed_setpoints()
        log.info("%s: mode -> %s", self.name, mode.name)

    def _require_mapped(self, entry: str, what: str) -> None:
        """Refuse a command whose setpoint would never reach the drive."""
        if not self.rx_pdo.has(entry):
            raise AxisError(
                f"{self.name}: {what} needs '{entry}' in the RxPDO, which this "
                f"axis does not map; add it via the axis rx_pdo config"
            )

    def _seed_setpoints(self) -> None:
        """Align every setpoint with the measured state. Call before enabling
        or after a mode change, otherwise the drive jumps to a stale target."""
        position = self.state.position_counts
        self.profile.reset(position)
        self.velocity_ramp.reset()
        self.torque_ramp.reset()
        self._outputs["target_position"] = position
        self._outputs["target_velocity"] = 0
        self._outputs["target_torque"] = 0

    def move_to(self, position_deg: float) -> None:
        """Command an absolute position, in application degrees (CSP)."""
        if not self.cfg.within_limits(position_deg):
            raise AxisError(
                f"{self.name}: target {position_deg:.3f} deg is outside limits "
                f"[{self.cfg.limits.min_deg}, {self.cfg.limits.max_deg}]"
            )
        if self.mode is not c.Mode.CSP:
            raise AxisError(f"{self.name}: move_to needs CSP, mode is {self.mode.name}")
        self.profile.set_goal(self.cfg.deg_to_counts(position_deg))

    def move_by(self, delta_deg: float) -> None:
        """Command a relative move, in application degrees."""
        if abs(delta_deg) > self.cfg.limits.max_step_deg:
            raise AxisError(
                f"{self.name}: step {delta_deg:.3f} deg exceeds max_step_deg "
                f"{self.cfg.limits.max_step_deg}"
            )
        self.move_to(self.state.position_deg + delta_deg)

    def set_velocity(self, velocity_deg_s: float) -> None:
        """Command a velocity, in application deg/s (CSV)."""
        self._require_mapped("target_velocity", "set_velocity")
        if self.mode is not c.Mode.CSV:
            raise AxisError(
                f"{self.name}: set_velocity needs CSV, mode is {self.mode.name}"
            )
        limit = self.cfg.limits.max_velocity_deg_s
        if abs(velocity_deg_s) > limit:
            raise AxisError(
                f"{self.name}: velocity {velocity_deg_s:.3f} deg/s exceeds {limit}"
            )
        self.velocity_ramp.set_target(
            self.cfg.direction * self.cfg.velocity_to_counts(abs(velocity_deg_s))
            * (1 if velocity_deg_s >= 0 else -1)
        )

    def set_torque(self, torque_permille: float) -> None:
        """Command a torque, in per-mille of rated torque (CST)."""
        self._require_mapped("target_torque", "set_torque")
        if self.mode is not c.Mode.CST:
            raise AxisError(
                f"{self.name}: set_torque needs CST, mode is {self.mode.name}"
            )
        self.torque_ramp.set_target(torque_permille)

    def set_profile_limits(
        self, velocity_deg_s: float | None = None,
        acceleration_deg_s2: float | None = None,
    ) -> None:
        """Change the CSP profile speed/acceleration, within configured limits."""
        limits = self.cfg.limits
        if velocity_deg_s is not None:
            if not 0 < velocity_deg_s <= limits.max_velocity_deg_s:
                raise AxisError(
                    f"{self.name}: velocity must be in (0, {limits.max_velocity_deg_s}]"
                )
            self.profile.max_velocity = self.cfg.velocity_to_counts(velocity_deg_s)
        if acceleration_deg_s2 is not None:
            if not 0 < acceleration_deg_s2 <= limits.max_acceleration_deg_s2:
                raise AxisError(
                    f"{self.name}: acceleration must be in "
                    f"(0, {limits.max_acceleration_deg_s2}]"
                )
            self.profile.max_acceleration = self.cfg.velocity_to_counts(
                acceleration_deg_s2
            )

    def halt(self, active: bool = True) -> None:
        """Set or clear the CiA 402 halt bit, stopping motion on the profile."""
        self._halt = active
        if active:
            self.profile.set_goal(self.profile.position)
            self.velocity_ramp.set_target(0.0)
            self.torque_ramp.set_target(0.0)

    def stop(self) -> None:
        """Decelerate to a stop but stay enabled."""
        self.profile.set_goal(self.profile.position)
        self.velocity_ramp.set_target(0.0)
        self.torque_ramp.set_target(0.0)

    def quick_stop(self) -> None:
        """Trigger the drive's quick-stop ramp (controlword bit 2 low)."""
        self._want_enable = False
        self._outputs["controlword"] = c.CW_QUICK_STOP_CMD

    @property
    def at_target(self) -> bool:
        """True when the profile is finished and the drive is within tolerance."""
        if self.mode is not c.Mode.CSP:
            return bool(self.state.statusword & c.SW_TARGET_REACHED)
        tolerance = max(1.0, abs(self.cfg.deg_to_counts_delta(0.05)))
        return (
            self.profile.done
            and abs(self.state.position_counts - self.profile.goal) <= tolerance
        )

    def wait_for_target(self, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.fault_reason:
                return False
            if self.at_target:
                return True
            time.sleep(0.002)
        return False

    # --- cyclic (cyclic thread only) -------------------------------------
    def read_inputs(self) -> None:
        """Decode this slave's process-data inputs into `self.state`."""
        raw = self.slave.input
        if not raw:
            return
        try:
            values = self.tx_pdo.unpack(raw)
        except ValueError:
            # A short image usually means the slave is not in OP yet.
            return
        self._inputs = values

        st = self.state
        st.statusword = values.get("statusword", 0)
        st.position_counts = values.get("position_actual", st.position_counts)
        st.velocity_counts_s = values.get("velocity_actual", 0)
        st.torque_permille = values.get("torque_actual", 0)
        st.mode_display = values.get("mode_display", st.mode_display)
        st.state = c.decode_state(st.statusword)
        st.position_deg = self.cfg.counts_to_deg(st.position_counts)
        st.velocity_deg_s = self.cfg.counts_to_deg_delta(st.velocity_counts_s)
        st.following_error_deg = self.cfg.counts_to_deg_delta(
            self._outputs["target_position"] - st.position_counts
        )

    def write_outputs(self) -> None:
        """Encode `self._outputs` into this slave's process-data outputs."""
        self.slave.output = self.rx_pdo.pack(self._outputs)

    def on_cycle(self, dt: float) -> None:
        """Advance the axis by one cycle. Called from the cyclic thread."""
        self.read_inputs()

        if self._first_cycle and self.state.statusword:
            # The first valid statusword tells us where the axis really is.
            self._seed_setpoints()
            self._first_cycle = False

        self._check_safety()
        self._step_state_machine()
        if self.state.state is c.State.OPERATION_ENABLED and not self.fault_reason:
            self._step_setpoints(dt)
        self.write_outputs()

    def _check_safety(self) -> None:
        """Latch a fault on limit violations. Runs every cycle."""
        st = self.state
        if self.fault_reason is not None and not self._drive_fault_latched:
            # A master-side trip (limits, following error) stays latched until
            # the operator calls reset_fault(). A drive-reported fault falls
            # through, so it can clear itself once the drive recovers.
            return
        limits = self.cfg.limits

        if st.state is c.State.FAULT:
            if self._drive_fault_latched:
                return
            self.fault_reason = f"drive fault, statusword 0x{st.statusword:04X}"
            self._drive_fault_latched = True
            # Keep _want_enable set: the state machine issues a fault reset and
            # walks the axis back up, which is what an operator asking to
            # enable a faulted drive means.
            return
        self._drive_fault_latched = False

        if not self._enabled.is_set():
            # Limits are only meaningful once the axis is under control.
            return

        if abs(st.following_error_deg) > limits.max_following_error_deg:
            self.fault_reason = (
                f"following error {st.following_error_deg:.3f} deg exceeds "
                f"{limits.max_following_error_deg} deg"
            )
            self._want_enable = False
            self.stop()
            return

        # A position outside the software limits does not by itself fault the
        # axis: an axis parked outside its window after a collision or a manual
        # adjustment must stay enableable, or it can never be driven back in.
        # What is forbidden is commanding it *further* out, which `move_to`
        # already rejects. Only a move deeper into the violation trips here.
        if not self.cfg.within_limits(st.position_deg):
            if not self._violation_deepening(st.position_deg):
                if not self._outside_limits_warned:
                    log.warning(
                        "%s: position %.3f deg is outside software limits "
                        "[%s, %s]; motion back toward the window is still "
                        "allowed",
                        self.name, st.position_deg,
                        limits.min_deg, limits.max_deg,
                    )
                    self._outside_limits_warned = True
                return
            self.fault_reason = (
                f"position {st.position_deg:.3f} deg is moving further outside "
                f"the software limits [{limits.min_deg}, {limits.max_deg}]"
            )
            self._want_enable = False
            self.stop()

    def _violation_deepening(self, position_deg: float) -> bool:
        """True when the axis is travelling further outside its limit window.

        Being outside is tolerated so the axis can be recovered; travelling
        further out is not, because that is a runaway rather than a recovery.
        """
        limits = self.cfg.limits
        if limits.min_deg is not None and position_deg < limits.min_deg:
            excursion = limits.min_deg - position_deg
        elif limits.max_deg is not None and position_deg > limits.max_deg:
            excursion = position_deg - limits.max_deg
        else:
            self._previous_excursion = None
            return False

        previous = self._previous_excursion
        self._previous_excursion = excursion
        if previous is None:
            return False
        # A small tolerance keeps encoder noise from reading as a runaway.
        return excursion > previous + abs(
            self.cfg.counts_to_deg_delta(2)
        )

    def _step_state_machine(self) -> None:
        """Drive the CiA 402 transitions toward the requested state."""
        st = self.state.state

        if not self._want_enable:
            if st is c.State.OPERATION_ENABLED:
                self._outputs["controlword"] = c.CW_SHUTDOWN
            elif st in (c.State.READY_TO_SWITCH_ON, c.State.SWITCHED_ON):
                self._outputs["controlword"] = c.CW_DISABLE_VOLTAGE
            self._enabled.clear()
            return

        if st is c.State.OPERATION_ENABLED:
            cw = c.CW_ENABLE_OPERATION_CMD
            if self._halt:
                cw |= c.CW_HALT
            if self._homing_active:
                cw |= c.CW_HOMING_START
            self._outputs["controlword"] = cw
            if not self._enabled.is_set():
                # Just became enabled: make sure setpoints match reality.
                self._seed_setpoints()
                self._enabled.set()
            return

        # Not enabled yet: take the next transition. Keep the setpoints glued
        # to the measured position so enabling does not command a jump.
        self._seed_setpoints()
        cw = c.next_controlword(st)
        if cw is not None:
            self._outputs["controlword"] = cw

    def _step_setpoints(self, dt: float) -> None:
        """Generate this cycle's setpoint for the active mode."""
        with self._mode_lock:
            mode = self.mode

        if mode is c.Mode.CSP:
            self._outputs["target_position"] = int(round(self.profile.update(dt)))
        elif mode is c.Mode.CSV:
            self._outputs["target_velocity"] = int(round(self.velocity_ramp.update(dt)))
        elif mode is c.Mode.CST:
            self._outputs["target_torque"] = int(round(self.torque_ramp.update(dt)))
        # PP, PV and homing are driven by the drive itself; the controlword
        # handshake for those is handled by the caller via SDOs.

    # --- diagnostics -----------------------------------------------------
    def describe(self) -> str:
        st = self.state
        try:
            mode_name = c.Mode(st.mode_display).name
        except ValueError:
            mode_name = str(st.mode_display)
        parts = [
            f"{self.name:<10}",
            f"{st.position_deg:9.3f} deg",
            f"{st.velocity_deg_s:8.2f} deg/s",
            f"mode={mode_name}",
            c.describe_statusword(st.statusword),
        ]
        if self.fault_reason:
            parts.append(f"FAULT: {self.fault_reason}")
        return "  ".join(parts)
