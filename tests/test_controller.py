"""Coordinated motion across two axes, against simulated drives.

`move_coordinated` is the reason the cyclic loop is shared, so it is tested
here on a two-axis bus built from the same FakeSlave used in test_axis.
"""
import pytest

from ethercat_mc import cia402 as c
from ethercat_mc.axis import Axis, AxisError
from ethercat_mc.config import AxisConfig, BusConfig, LimitConfig
from ethercat_mc.controller import MotionController
from ethercat_mc.pdo import default_rx_pdo, default_tx_pdo
from ethercat_mc.trajectory import sync_duration

from .test_axis import DT, FakeSlave

#: 3600 counts per degree, as on the demo config scaled down for speed.
COUNTS_PER_REV = 1_296_000


def make_controller(**limit_overrides):
    """Build a MotionController with two simulated eRob-like axes.

    `start()` is bypassed: the bus bring-up needs pysoem, but everything under
    test here is the axis/profile layer, which only needs the axes wired up.
    """
    def axis_cfg(name, position):
        return AxisConfig(
            name=name,
            slave_position=position,
            driver="erob",
            counts_per_rev=COUNTS_PER_REV,
            limits=LimitConfig(
                min_deg=limit_overrides.get("min_deg", -180.0),
                max_deg=limit_overrides.get("max_deg", 180.0),
                max_velocity_deg_s=limit_overrides.get("max_velocity_deg_s", 30.0),
                max_acceleration_deg_s2=limit_overrides.get(
                    "max_acceleration_deg_s2", 60.0
                ),
                max_step_deg=180.0,
                max_following_error_deg=1000.0,  # the sim follows exactly
            ),
        )

    cfgs = [axis_cfg("joint1", 0), axis_cfg("joint2", 1)]
    bus = BusConfig(cycle_time_s=DT, axes=cfgs)
    mc = MotionController(bus)
    # The tests step the cycle themselves, so there is no real cyclic thread.
    # check_health() still insists on one - correctly, for production - so a
    # stand-in reports the loop as healthy.
    mc.master.task = FakeCyclicTask()

    slaves = []
    for cfg in cfgs:
        slave = FakeSlave()
        axis = Axis(cfg, slave, default_rx_pdo(), default_tx_pdo())
        mc.axes.append(axis)
        mc._by_name[axis.name] = axis
        slaves.append(slave)
    return mc, slaves


class FakeCyclicTask:
    """Reports a healthy loop to check_health(); the tests drive the cycles."""

    def __init__(self):
        self.running = True
        self.cycle_count = 0
        self.actual_wkc = 2
        self.expected_wkc = 2
        self.max_jitter_s = 0.0
        self.last_error = None

    class _Flag:
        @staticmethod
        def is_set():
            return False

    faulted = _Flag()


def run_cycles(mc, slaves, n):
    for _ in range(n):
        for axis in mc.axes:
            axis.on_cycle(DT)
        for slave in slaves:
            slave.step()


def enable(mc, slaves):
    for axis in mc.axes:
        axis.request_enable()
    run_cycles(mc, slaves, 10)
    assert all(a.is_enabled for a in mc.axes)


def cycles_until_done(mc, slaves, names, limit=200_000):
    """Run until every named axis reports at_target; return cycles taken."""
    axes = [mc.axis(n) for n in names]
    for i in range(limit):
        run_cycles(mc, slaves, 1)
        if all(a.at_target for a in axes):
            return i + 1
    raise AssertionError("axes did not reach their targets")


class TestCoordinatedMotion:
    def test_both_axes_reach_their_targets(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        mc.move_coordinated({"joint1": 60.0, "joint2": 20.0})
        cycles_until_done(mc, slaves, ["joint1", "joint2"])
        assert mc.axis("joint1").position_deg == pytest.approx(60.0, abs=0.01)
        assert mc.axis("joint2").position_deg == pytest.approx(20.0, abs=0.01)

    def test_axes_finish_together(self):
        """The whole point: unequal distances, simultaneous arrival."""
        mc, slaves = make_controller()
        enable(mc, slaves)
        mc.move_coordinated({"joint1": 60.0, "joint2": 20.0})

        finished = {}
        for i in range(200_000):
            run_cycles(mc, slaves, 1)
            for name in ("joint1", "joint2"):
                if name not in finished and mc.axis(name).at_target:
                    finished[name] = i
            if len(finished) == 2:
                break
        assert len(finished) == 2
        gap_cycles = abs(finished["joint1"] - finished["joint2"])
        # Within 5% of the move, the two arrivals count as simultaneous.
        assert gap_cycles * DT < 0.05 * max(finished.values()) * DT + 0.05

    def test_short_axis_is_slowed_not_the_long_one(self):
        """Scaling must slow the shorter move, never speed the longer one past
        its own limit."""
        mc, slaves = make_controller(max_velocity_deg_s=30.0)
        enable(mc, slaves)
        mc.move_coordinated({"joint1": 60.0, "joint2": 20.0})
        long_axis = mc.axis("joint1")
        short_axis = mc.axis("joint2")
        assert short_axis.profile.max_velocity < long_axis.profile.max_velocity
        limit = long_axis.cfg.velocity_to_counts(30.0)
        assert long_axis.profile.max_velocity <= limit * 1.001

    def test_duration_matches_the_slowest_axis(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        axis = mc.axis("joint1")
        expected = sync_duration(
            abs(axis.cfg.deg_to_counts(60.0) - axis.profile.position),
            axis.profile.max_velocity,
            axis.profile.max_acceleration,
        )
        duration = mc.move_coordinated({"joint1": 60.0, "joint2": 20.0})
        assert duration == pytest.approx(expected, rel=0.01)

    def test_explicit_duration_is_honoured(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        duration = mc.move_coordinated({"joint1": 30.0, "joint2": 10.0},
                                       duration=6.0)
        assert duration == 6.0
        cycles = cycles_until_done(mc, slaves, ["joint1", "joint2"])
        assert cycles * DT == pytest.approx(6.0, rel=0.15)

    def test_impossible_duration_rejected(self):
        """Asking for faster than an axis can go must fail loudly, not clip."""
        mc, slaves = make_controller(max_velocity_deg_s=10.0)
        enable(mc, slaves)
        with pytest.raises(AxisError, match="shorter than axes"):
            mc.move_coordinated({"joint1": 90.0}, duration=0.5)

    def test_target_outside_limits_rejected(self):
        mc, slaves = make_controller(min_deg=-45.0, max_deg=45.0)
        enable(mc, slaves)
        with pytest.raises(AxisError, match="outside limits"):
            mc.move_coordinated({"joint1": 90.0, "joint2": 10.0})

    def test_no_axis_moves_when_one_target_is_invalid(self):
        """A rejected group must leave every axis where it was."""
        mc, slaves = make_controller(min_deg=-45.0, max_deg=45.0)
        enable(mc, slaves)
        before = [a.profile.goal for a in mc.axes]
        with pytest.raises(AxisError):
            mc.move_coordinated({"joint1": 10.0, "joint2": 90.0})
        assert [a.profile.goal for a in mc.axes] == before

    def test_requires_csp(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        mc.axis("joint2").set_mode(c.Mode.CSV)
        with pytest.raises(AxisError, match="needs CSP"):
            mc.move_coordinated({"joint1": 10.0, "joint2": 10.0})

    def test_zero_distance_returns_immediately(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        assert mc.move_coordinated({"joint1": 0.0, "joint2": 0.0}) == 0.0

    def test_restore_limits_undoes_scaling(self):
        mc, slaves = make_controller(max_velocity_deg_s=30.0)
        enable(mc, slaves)
        mc.move_coordinated({"joint1": 60.0, "joint2": 20.0})
        assert mc.axis("joint2").profile.max_velocity < mc.axis(
            "joint1"
        ).profile.max_velocity

        mc.restore_configured_limits()
        expected = mc.axis("joint1").cfg.velocity_to_counts(30.0)
        for axis in mc.axes:
            assert axis.profile.max_velocity == pytest.approx(expected)


class TestGroupOperations:
    def test_enable_all_and_disable_all(self):
        mc, slaves = make_controller()
        for axis in mc.axes:
            axis.request_enable()
        run_cycles(mc, slaves, 10)
        assert all(a.is_enabled for a in mc.axes)

        mc.disable_all()
        run_cycles(mc, slaves, 5)
        assert not any(a.is_enabled for a in mc.axes)

    def test_set_mode_all(self):
        mc, slaves = make_controller()
        mc.set_mode_all(c.Mode.CSV)
        assert all(a.mode is c.Mode.CSV for a in mc.axes)

    def test_stop_all_holds_every_axis(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        mc.move_coordinated({"joint1": 60.0, "joint2": 20.0})
        run_cycles(mc, slaves, 300)
        mc.stop_all()
        run_cycles(mc, slaves, 3000)
        held = [a.position_deg for a in mc.axes]
        run_cycles(mc, slaves, 500)
        for axis, previous in zip(mc.axes, held):
            assert axis.position_deg == pytest.approx(previous, abs=0.001)

    def test_axis_lookup_by_name(self):
        mc, _ = make_controller()
        assert mc.axis("joint1").name == "joint1"
        assert mc["joint2"].name == "joint2"
        with pytest.raises(KeyError, match="no axis named"):
            mc.axis("joint9")

    def test_check_health_reports_axis_fault(self):
        mc, slaves = make_controller()
        enable(mc, slaves)
        mc.axis("joint1").fault_reason = "simulated trip"
        # No cyclic task in this harness, so the bus check fires first; the
        # axis fault is what `healthy` reflects.
        assert not mc.healthy


class TestDemoConfig:
    def test_demo_config_is_valid(self):
        """The shipped two-eRob demo config must stay loadable and sane."""
        from ethercat_mc.config import load

        cfg = load("configs/demo_2x_erob70.yaml")
        assert len(cfg.axes) == 2
        assert [a.slave_position for a in cfg.axes] == [0, 1]
        assert all(a.driver == "erob" for a in cfg.axes)
        for a in cfg.axes:
            # eRob reads the output shaft, so no reduction is applied.
            assert a.gear_ratio == 1.0
            assert a.counts_per_output_rev == 524288
            assert a.limits.max_velocity_deg_s == 30.0


class TestSpeedCap:
    def test_speed_cap_applies_to_the_pace_setting_axis(self):
        """A lowered speed must slow every axis, including the slowest one,
        and the move must take the planned time (it ran at the config ceiling
        before, arriving early)."""
        mc, slaves = make_controller(max_velocity_deg_s=30.0)
        enable(mc, slaves)
        duration = mc.move_coordinated({"joint1": 60.0, "joint2": 20.0},
                                       max_velocity_deg_s=15.0)
        lead = mc.axis("joint1")
        assert lead.profile.max_velocity <= lead.cfg.velocity_to_counts(15.0) * 1.001
        cycles = cycles_until_done(mc, slaves, ["joint1", "joint2"])
        assert cycles * DT == pytest.approx(duration, rel=0.02)

    def test_cap_above_config_is_ignored(self):
        mc, slaves = make_controller(max_velocity_deg_s=10.0)
        enable(mc, slaves)
        mc.move_coordinated({"joint1": 30.0}, max_velocity_deg_s=100.0)
        a = mc.axis("joint1")
        assert a.profile.max_velocity <= a.cfg.velocity_to_counts(10.0) * 1.001
