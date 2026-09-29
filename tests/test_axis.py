"""Axis behaviour against a simulated drive.

`FakeSlave` stands in for a pysoem slave: it accepts an output image, decodes
the controlword, walks the CiA 402 state machine the way a real drive would and
reports a statusword plus a position that follows the commanded target. That is
enough to test the enable sequence, mode switching and the safety latches
without hardware.
"""
import struct

import pytest

from ethercat_mc import cia402 as c
from ethercat_mc.axis import Axis, AxisError
from ethercat_mc.config import AxisConfig, LimitConfig
from ethercat_mc.pdo import default_rx_pdo, default_tx_pdo

DT = 0.002


class FakeSlave:
    """A minimal CiA 402 drive simulator."""

    def __init__(self, position=0, follows_target=True):
        self.rx = default_rx_pdo()
        self.tx = default_tx_pdo()
        self.state = c.State.SWITCH_ON_DISABLED
        self.position = position
        self.follows_target = follows_target
        self.mode = 0
        self.output = b""
        self.input = b""
        self.sdo_writes = []
        self._refresh_input()

    # --- the parts Axis uses ---
    def sdo_read(self, index, subindex=0):
        if index == 0x6064:
            return struct.pack("<i", self.position)
        return b"\x00\x00\x00\x00"

    def sdo_write(self, index, subindex, data):
        self.sdo_writes.append((index, subindex, data))

    # --- simulation ---
    def step(self):
        """Consume the output image and produce the next input image."""
        if not self.output:
            return
        values = self.rx.unpack(self.output)
        cw = values["controlword"]
        self.mode = values["mode_of_operation"]

        # CiA 402 transitions, in the order a real drive applies them.
        if cw & c.CW_FAULT_RESET and self.state is c.State.FAULT:
            self.state = c.State.SWITCH_ON_DISABLED
        elif cw & 0x0F == 0x06 and self.state in (
            c.State.SWITCH_ON_DISABLED, c.State.READY_TO_SWITCH_ON,
            c.State.SWITCHED_ON, c.State.OPERATION_ENABLED,
        ):
            self.state = c.State.READY_TO_SWITCH_ON
        elif cw & 0x0F == 0x07 and self.state is c.State.READY_TO_SWITCH_ON:
            self.state = c.State.SWITCHED_ON
        elif cw & 0x0F == 0x0F and self.state in (
            c.State.SWITCHED_ON, c.State.OPERATION_ENABLED
        ):
            self.state = c.State.OPERATION_ENABLED
        elif cw & 0x0F == 0x00:
            self.state = c.State.SWITCH_ON_DISABLED

        if self.state is c.State.OPERATION_ENABLED and self.follows_target:
            self.position = values["target_position"]
        self._refresh_input()

    def _refresh_input(self):
        status = {
            c.State.SWITCH_ON_DISABLED: 0x0040,
            c.State.READY_TO_SWITCH_ON: 0x0021,
            c.State.SWITCHED_ON: 0x0023,
            c.State.OPERATION_ENABLED: 0x0027,
            c.State.FAULT: 0x0008,
        }[self.state]
        self.input = self.tx.pack({
            "statusword": status,
            "position_actual": self.position,
            "velocity_actual": 0,
            "torque_actual": 0,
            "mode_display": self.mode,
        })


def make_axis(slave=None, **overrides):
    """Build an Axis wired to a FakeSlave, with 3600 counts per degree."""
    limits = LimitConfig(
        min_deg=overrides.pop("min_deg", -180.0),
        max_deg=overrides.pop("max_deg", 180.0),
        max_velocity_deg_s=overrides.pop("max_velocity_deg_s", 30.0),
        max_acceleration_deg_s2=overrides.pop("max_acceleration_deg_s2", 60.0),
        max_step_deg=overrides.pop("max_step_deg", 90.0),
        max_following_error_deg=overrides.pop("max_following_error_deg", 5.0),
    )
    cfg = AxisConfig(
        name="j1",
        slave_position=0,
        counts_per_rev=overrides.pop("counts_per_rev", 1_296_000),  # 3600/deg
        **overrides,
        limits=limits,
    )
    slave = slave or FakeSlave()
    return Axis(cfg, slave, default_rx_pdo(), default_tx_pdo()), slave


def run_cycles(axis, slave, n):
    """Run n cycles of the axis/slave pair."""
    for _ in range(n):
        axis.on_cycle(DT)
        slave.step()


class TestEnableSequence:
    def test_reaches_operation_enabled(self):
        axis, slave = make_axis()
        axis.request_enable()
        run_cycles(axis, slave, 10)
        assert axis.is_enabled
        assert slave.state is c.State.OPERATION_ENABLED

    def test_walks_through_every_state(self):
        """The drive must see 0x06, 0x07 then 0x0F, not a jump straight to 0x0F."""
        axis, slave = make_axis()
        axis.request_enable()
        seen = []
        for _ in range(10):
            axis.on_cycle(DT)
            seen.append(slave.state)
            slave.step()
        assert c.State.READY_TO_SWITCH_ON in seen
        assert c.State.SWITCHED_ON in seen
        assert slave.state is c.State.OPERATION_ENABLED

    def test_recovers_from_fault(self):
        axis, slave = make_axis()
        slave.state = c.State.FAULT
        slave._refresh_input()
        axis.request_enable()
        run_cycles(axis, slave, 15)
        assert axis.is_enabled

    def test_disable_drops_out_of_operation(self):
        axis, slave = make_axis()
        axis.request_enable()
        run_cycles(axis, slave, 10)
        assert axis.is_enabled
        axis.request_disable()
        run_cycles(axis, slave, 5)
        assert not axis.is_enabled

    def test_setpoint_matches_position_on_enable(self):
        """Enabling must not command a jump from a stale target."""
        slave = FakeSlave(position=123_456)
        axis, slave = make_axis(slave=slave)
        axis.request_enable()
        run_cycles(axis, slave, 10)
        assert axis._outputs["target_position"] == 123_456
        assert slave.position == 123_456


class TestMotion:
    def _enabled_axis(self, **kw):
        axis, slave = make_axis(**kw)
        axis.request_enable()
        run_cycles(axis, slave, 10)
        assert axis.is_enabled
        return axis, slave

    def test_move_to_reaches_target(self):
        axis, slave = self._enabled_axis()
        axis.move_to(10.0)
        run_cycles(axis, slave, 5000)
        assert axis.position_deg == pytest.approx(10.0, abs=0.01)

    def test_move_by_is_relative(self):
        axis, slave = self._enabled_axis()
        axis.move_to(5.0)
        run_cycles(axis, slave, 5000)
        axis.move_by(3.0)
        run_cycles(axis, slave, 5000)
        assert axis.position_deg == pytest.approx(8.0, abs=0.01)

    def test_move_outside_limits_rejected(self):
        axis, _ = self._enabled_axis(min_deg=-10.0, max_deg=10.0)
        with pytest.raises(AxisError, match="outside limits"):
            axis.move_to(11.0)

    def test_oversized_step_rejected(self):
        axis, _ = self._enabled_axis(max_step_deg=5.0)
        with pytest.raises(AxisError, match="exceeds max_step_deg"):
            axis.move_by(10.0)

    def test_move_to_requires_csp(self):
        axis, _ = self._enabled_axis()
        axis.set_mode(c.Mode.CSV)
        with pytest.raises(AxisError, match="needs CSP"):
            axis.move_to(5.0)

    def test_velocity_requires_csv(self):
        axis, _ = self._enabled_axis()
        with pytest.raises(AxisError, match="needs CSV"):
            axis.set_velocity(5.0)

    def test_velocity_over_limit_rejected(self):
        axis, _ = self._enabled_axis(max_velocity_deg_s=10.0)
        axis.set_mode(c.Mode.CSV)
        with pytest.raises(AxisError, match="exceeds"):
            axis.set_velocity(20.0)

    def test_torque_requires_cst(self):
        axis, _ = self._enabled_axis()
        with pytest.raises(AxisError, match="needs CST"):
            axis.set_torque(100)

    def test_profile_speed_bounded_by_config(self):
        axis, _ = self._enabled_axis(max_velocity_deg_s=10.0)
        with pytest.raises(AxisError, match="velocity must be in"):
            axis.set_profile_limits(velocity_deg_s=50.0)

    def test_stop_holds_position(self):
        axis, slave = self._enabled_axis()
        axis.move_to(20.0)
        run_cycles(axis, slave, 200)
        axis.stop()
        run_cycles(axis, slave, 2000)
        held = axis.position_deg
        run_cycles(axis, slave, 500)
        assert axis.position_deg == pytest.approx(held, abs=0.001)

    def test_mode_switch_reseeds_setpoints(self):
        """Switching mode must not leave a stale position target behind."""
        axis, slave = self._enabled_axis()
        axis.move_to(15.0)
        run_cycles(axis, slave, 5000)
        axis.set_mode(c.Mode.CSV)
        assert axis._outputs["target_velocity"] == 0
        assert axis._outputs["target_position"] == axis.state.position_counts


class TestSafety:
    def test_following_error_latches_fault(self):
        """A drive that does not follow the target must trip the axis."""
        slave = FakeSlave(follows_target=False)
        axis, slave = make_axis(slave=slave, max_following_error_deg=1.0)
        axis.request_enable()
        run_cycles(axis, slave, 10)
        axis.move_to(50.0)
        run_cycles(axis, slave, 3000)
        assert axis.fault_reason is not None
        assert "following error" in axis.fault_reason

    def test_parked_outside_limits_does_not_fault(self):
        """Being outside the window is recoverable, so it must not trip."""
        slave = FakeSlave(position=0)
        axis, slave = make_axis(slave=slave, min_deg=10.0, max_deg=20.0)
        axis.request_enable()
        run_cycles(axis, slave, 20)
        assert axis.is_enabled
        assert axis.fault_reason is None

    def test_moving_further_outside_limits_faults(self):
        """Travelling deeper out of the window is a runaway, not a recovery."""
        slave = FakeSlave(follows_target=False)
        axis, slave = make_axis(
            slave=slave, min_deg=-10.0, max_deg=10.0,
            max_following_error_deg=1000.0,  # isolate the limit check
        )
        axis.request_enable()
        run_cycles(axis, slave, 10)
        # Walk the drive steadily further past the limit.
        for step_deg in (11.0, 13.0, 15.0, 17.0):
            slave.position = axis.cfg.deg_to_counts(step_deg)
            slave._refresh_input()
            run_cycles(axis, slave, 2)
        assert axis.fault_reason is not None
        assert "further outside" in axis.fault_reason

    def test_drive_fault_is_latched(self):
        axis, slave = make_axis()
        axis.request_enable()
        run_cycles(axis, slave, 10)
        slave.state = c.State.FAULT
        slave._refresh_input()
        run_cycles(axis, slave, 3)
        assert axis.is_faulted
        assert "drive fault" in axis.fault_reason

    def test_request_enable_clears_previous_fault(self):
        axis, slave = make_axis()
        axis.fault_reason = "stale"
        axis.request_enable()
        assert axis.fault_reason is None

    def test_reset_fault_allows_motion_again(self):
        """After a trip, reset_fault plus enable must recover the axis."""
        slave = FakeSlave(follows_target=False)
        axis, slave = make_axis(slave=slave, max_following_error_deg=1.0)
        axis.request_enable()
        run_cycles(axis, slave, 10)
        axis.move_to(50.0)
        run_cycles(axis, slave, 3000)
        assert axis.fault_reason is not None

        slave.follows_target = True
        axis.request_enable()
        run_cycles(axis, slave, 20)
        assert axis.fault_reason is None
        assert axis.is_enabled


class TestOutputEncoding:
    def test_controlword_reaches_the_wire(self):
        axis, slave = make_axis()
        axis.request_enable()
        run_cycles(axis, slave, 10)
        values = axis.rx_pdo.unpack(slave.output)
        assert values["controlword"] & 0x0F == 0x0F

    def test_mode_is_published(self):
        axis, slave = make_axis()
        axis.set_mode(c.Mode.CST)
        axis.on_cycle(DT)
        values = axis.rx_pdo.unpack(slave.output)
        assert values["mode_of_operation"] == int(c.Mode.CST)

    def test_state_reflects_input_image(self):
        slave = FakeSlave(position=7200)  # 2 degrees at 3600 counts/deg
        axis, slave = make_axis(slave=slave)
        axis.on_cycle(DT)
        assert axis.state.position_counts == 7200
        assert axis.position_deg == pytest.approx(2.0)


class TestZeroing:
    def test_set_zero_here_rezeroes_and_rewrites_drive_limits(self):
        """An absolute encoder parked at 342 deg must become 0 deg, and the
        drive-side 0x607D window must move with it (seen on real eRob70)."""
        from ethercat_mc import homing
        from ethercat_mc.drivers.erob import ERobDriver

        slave = FakeSlave(position=342 * 3600)  # 342 deg at 3600 counts/deg
        axis, slave = make_axis(slave=slave)
        axis.driver = ERobDriver(axis.cfg)
        axis.on_cycle(DT)
        assert axis.position_deg == pytest.approx(342.0)

        homing.set_zero_here(axis)
        axis.on_cycle(DT)
        assert axis.position_deg == pytest.approx(0.0)

        limits = {sub: struct.unpack("<i", data)[0]
                  for idx, sub, data in slave.sdo_writes if idx == 0x607D}
        # [-180, 180] deg around the new zero, in raw counts.
        assert limits[1] == (342 - 180) * 3600
        assert limits[2] == (342 + 180) * 3600

    def test_move_relative_to_new_zero(self):
        from ethercat_mc import homing

        slave = FakeSlave(position=342 * 3600)
        axis, slave = make_axis(slave=slave)
        axis.on_cycle(DT)
        homing.set_zero_here(axis)
        axis.request_enable()
        run_cycles(axis, slave, 10)
        axis.move_to(2.0)  # would be rejected as 344 deg without re-zeroing
        run_cycles(axis, slave, 3000)
        assert axis.position_deg == pytest.approx(2.0, abs=0.01)
        assert slave.position == pytest.approx(344 * 3600, abs=5)


class TestERobLayout:
    """The eRob driver uses the ZeroErr ESI default PDOs (10 bytes each)."""

    def _axis(self):
        from ethercat_mc.drivers.erob import ERobDriver

        cfg = AxisConfig(name="j", slave_position=0, counts_per_rev=1_296_000)
        d = ERobDriver(cfg)
        slave = FakeSlave()
        return Axis(cfg, slave, d.rx_pdo(), d.tx_pdo()), slave, d

    def test_layout_matches_the_esi(self):
        _, _, d = self._axis()
        assert [e.mapping_value for e in d.rx_pdo().entries] == [
            0x607A0020, 0x60FE0020, 0x60400010]
        assert [e.mapping_value for e in d.tx_pdo().entries] == [
            0x60640020, 0x60FD0020, 0x60410010]
        assert d.rx_pdo().size == d.tx_pdo().size == 10

    def test_mode_is_written_over_sdo_when_not_mapped(self):
        axis, slave, _ = self._axis()
        slave.sdo_writes.clear()
        axis.set_mode(c.Mode.CSP)
        assert (0x6060, 0, struct.pack("<b", 8)) in slave.sdo_writes

    def test_unmapped_velocity_is_refused(self):
        """A CSV setpoint that the PDO cannot carry must not be silently lost."""
        axis, _, _ = self._axis()
        axis.mode = c.Mode.CSV
        with pytest.raises(AxisError, match="does not map"):
            axis.set_velocity(1.0)

    def test_unmapped_torque_is_refused(self):
        axis, _, _ = self._axis()
        axis.mode = c.Mode.CST
        with pytest.raises(AxisError, match="does not map"):
            axis.set_torque(10)

    def test_packs_controlword_and_target(self):
        axis, _, d = self._axis()
        axis._outputs["controlword"] = 0x0F
        axis._outputs["target_position"] = -5
        values = d.rx_pdo().unpack(d.rx_pdo().pack(axis._outputs))
        assert values["controlword"] == 0x0F
        assert values["target_position"] == -5
        assert values["digital_outputs"] == 0


class TestDriveLimitWrites:
    def test_window_is_opened_before_narrowing(self):
        """EPOS4 faults (0x8A82) if the window ever excludes the position,
        which the old min-then-max order did from factory 0/0 limits."""
        from ethercat_mc.drivers.maxon import MaxonDriver

        slave = FakeSlave(position=4351)
        cfg = AxisConfig(name="h", slave_position=0, counts_per_rev=16384,
                         zero_offset_counts=4351,
                         limits=LimitConfig(min_deg=-180.0, max_deg=180.0))
        MaxonDriver(cfg).write_position_limits(slave)
        writes = [(sub, struct.unpack("<i", d)[0])
                  for idx, sub, d in slave.sdo_writes if idx == 0x607D]
        assert writes[:2] == [(1, -2**31), (2, 2**31 - 1)]
        window = [-2**31, 2**31 - 1]
        for sub, value in writes:
            window[sub - 1] = value
            assert window[0] <= 4351 <= window[1]
        assert window == [4351 - 8192, 4351 + 8192]

    def test_window_excluding_position_is_skipped(self):
        from ethercat_mc.drivers.maxon import MaxonDriver

        slave = FakeSlave(position=100_000)
        cfg = AxisConfig(name="h", slave_position=0, counts_per_rev=16384,
                         limits=LimitConfig(min_deg=-180.0, max_deg=180.0))
        MaxonDriver(cfg).write_position_limits(slave)
        assert not [w for w in slave.sdo_writes if w[0] == 0x607D]


class TestHejFixes:
    def test_velocity_is_estimated_when_not_mapped(self):
        from ethercat_mc.drivers.maxon import MaxonDriver

        cfg = AxisConfig(name="h", slave_position=0, counts_per_rev=1_296_000)
        d = MaxonDriver(cfg)
        slave = FakeSlave()
        axis = Axis(cfg, slave, d.rx_pdo(), d.tx_pdo())
        for i in range(300):  # 10 deg/s = 36000 counts/s at 3600 counts/deg
            slave.input = d.tx_pdo().pack({
                "statusword": 0x0027, "position_actual": int(i * 36000 * DT)})
            axis.on_cycle(DT)
        assert axis.state.velocity_deg_s == pytest.approx(10.0, rel=0.02)

    def test_position_tolerance_is_configurable(self):
        slave = FakeSlave(follows_target=False)
        axis, slave = make_axis(slave=slave, max_following_error_deg=1000.0)
        axis.request_enable()
        run_cycles(axis, slave, 10)
        axis.move_to(0.2)             # drive stays at 0: 0.2 deg short
        run_cycles(axis, slave, 2000)
        assert not axis.at_target     # default 0.05 deg
        axis.cfg.position_tolerance_deg = 0.3
        assert axis.at_target
