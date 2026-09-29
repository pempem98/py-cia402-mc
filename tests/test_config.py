"""Configuration loading and unit conversion. No hardware needed.

Unit conversion is the highest-risk pure-software part of the system: a wrong
gear ratio or sign sends a real joint the wrong way, so it is tested closely.
"""
import textwrap

import pytest

from ethercat_mc.config import AxisConfig, LimitConfig, load


@pytest.fixture
def mixed_config(tmp_path):
    path = tmp_path / "bus.yaml"
    path.write_text(textwrap.dedent("""
        bus:
          adapter: \\Device\\NPF_{TEST}
          cycle_time_s: 0.004
          use_dc: true
        axes:
          - name: joint1
            driver: erob
            slave_position: 0
            counts_per_rev: 524288
            gear_ratio: 1.0
            direction: 1
            limits:
              min_deg: -180.0
              max_deg: 180.0
              max_velocity_deg_s: 30.0
          - name: joint2
            driver: maxon
            slave_position: 1
            vendor_id: 0x000000FB
            counts_per_rev: 4096
            gear_ratio: 100.0
            direction: -1
            limits:
              min_deg: -90.0
              max_deg: 90.0
            homing:
              enabled: true
              method: 37
    """), encoding="utf-8")
    return load(path)


class TestLoading:
    def test_bus_fields(self, mixed_config):
        assert mixed_config.adapter == r"\Device\NPF_{TEST}"
        assert mixed_config.cycle_time_s == 0.004
        assert mixed_config.use_dc is True

    def test_axes_loaded(self, mixed_config):
        assert [a.name for a in mixed_config.axes] == ["joint1", "joint2"]
        assert mixed_config.axis("joint2").driver == "maxon"

    def test_hex_vendor_id_parsed(self, mixed_config):
        assert mixed_config.axis("joint2").vendor_id == 0xFB

    def test_nested_sections(self, mixed_config):
        j2 = mixed_config.axis("joint2")
        assert j2.homing.enabled is True
        assert j2.homing.method == 37
        assert j2.limits.min_deg == -90.0

    def test_defaults_applied(self, mixed_config):
        j1 = mixed_config.axis("joint1")
        assert j1.homing.enabled is False
        assert j1.default_mode == "csp"
        assert j1.limits.max_following_error_deg == 5.0

    def test_unknown_axis_raises(self, mixed_config):
        with pytest.raises(KeyError):
            mixed_config.axis("nope")

    def test_example_config_loads(self):
        """The shipped example must stay valid."""
        cfg = load("configs/example_mixed.yaml")
        assert len(cfg.axes) == 2
        assert cfg.axis("joint1").driver == "erob"
        assert cfg.axis("joint2").driver == "maxon"


class TestValidation:
    def _write(self, tmp_path, body):
        path = tmp_path / "bad.yaml"
        path.write_text(textwrap.dedent(body), encoding="utf-8")
        return path

    def test_duplicate_names_rejected(self, tmp_path):
        path = self._write(tmp_path, """
            axes:
              - {name: a, slave_position: 0}
              - {name: a, slave_position: 1}
        """)
        with pytest.raises(ValueError, match="duplicate axis names"):
            load(path)

    def test_duplicate_slave_positions_rejected(self, tmp_path):
        path = self._write(tmp_path, """
            axes:
              - {name: a, slave_position: 0}
              - {name: b, slave_position: 0}
        """)
        with pytest.raises(ValueError, match="duplicate slave_position"):
            load(path)

    def test_bad_direction_rejected(self, tmp_path):
        path = self._write(tmp_path, """
            axes:
              - {name: a, slave_position: 0, direction: 2}
        """)
        with pytest.raises(ValueError, match="direction must be"):
            load(path)

    def test_zero_gear_ratio_rejected(self, tmp_path):
        path = self._write(tmp_path, """
            axes:
              - {name: a, slave_position: 0, gear_ratio: 0}
        """)
        with pytest.raises(ValueError, match="gear_ratio must be positive"):
            load(path)

    def test_negative_counts_per_rev_rejected(self, tmp_path):
        path = self._write(tmp_path, """
            axes:
              - {name: a, slave_position: 0, counts_per_rev: -1}
        """)
        with pytest.raises(ValueError, match="counts_per_rev must be positive"):
            load(path)


class TestUnitConversion:
    def test_erob_full_turn(self):
        """One output turn is one encoder turn when gear_ratio is 1."""
        a = AxisConfig(name="j", slave_position=0, counts_per_rev=524288)
        assert a.deg_to_counts(360.0) == 524288
        assert a.counts_to_deg(524288) == pytest.approx(360.0)

    def test_maxon_gearbox_applied(self):
        """A 100:1 gearbox needs 100 motor turns per output turn."""
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=4096, gear_ratio=100.0
        )
        assert a.deg_to_counts(360.0) == 4096 * 100
        assert a.counts_to_deg(4096 * 100) == pytest.approx(360.0)

    def test_direction_inverts_sign(self):
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=3600, direction=-1
        )
        assert a.deg_to_counts(10.0) == -100
        assert a.counts_to_deg(-100) == pytest.approx(10.0)

    def test_zero_offset_shifts_absolute_only(self):
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=3600,
            zero_offset_counts=500,
        )
        assert a.deg_to_counts(0.0) == 500
        assert a.counts_to_deg(500) == pytest.approx(0.0)
        # A delta must NOT include the offset.
        assert a.deg_to_counts_delta(10.0) == 100
        assert a.counts_to_deg_delta(100) == pytest.approx(10.0)

    def test_round_trip_absolute(self):
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=524288,
            gear_ratio=7.0, direction=-1, zero_offset_counts=12345,
        )
        for deg in (-180.0, -33.3, 0.0, 12.75, 179.9):
            assert a.counts_to_deg(a.deg_to_counts(deg)) == pytest.approx(
                deg, abs=1e-3
            )

    def test_round_trip_delta(self):
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=4096,
            gear_ratio=100.0, direction=-1, zero_offset_counts=9999,
        )
        for deg in (-45.0, -0.01, 0.0, 0.25, 90.0):
            assert a.counts_to_deg_delta(a.deg_to_counts_delta(deg)) == pytest.approx(
                deg, abs=1e-3
            )

    def test_velocity_is_unsigned_and_offset_free(self):
        """Velocity is a rate: the zero offset must not leak into it."""
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=3600,
            zero_offset_counts=100_000, direction=-1,
        )
        assert a.velocity_to_counts(10.0) == 100
        assert a.velocity_to_counts(-10.0) == 100

    def test_counts_per_output_rev(self):
        """A 100:1 reduction multiplies the per-motor-turn count by 100."""
        a = AxisConfig(
            name="j", slave_position=0, counts_per_rev=4096, gear_ratio=100.0
        )
        assert a.counts_per_output_rev == pytest.approx(4096 * 100)

    def test_gear_ratio_increases_resolution(self):
        """Higher reduction means more counts per output degree, so a
        geared axis resolves finer than the same encoder direct-driven."""
        direct = AxisConfig(name="d", slave_position=0, counts_per_rev=4096)
        geared = AxisConfig(
            name="g", slave_position=1, counts_per_rev=4096, gear_ratio=50.0
        )
        # A full turn divides exactly, so no rounding noise enters the ratio.
        assert geared.deg_to_counts(360.0) == 50 * direct.deg_to_counts(360.0)


class TestLimits:
    def test_within_limits(self):
        a = AxisConfig(
            name="j", slave_position=0,
            limits=LimitConfig(min_deg=-90.0, max_deg=90.0),
        )
        assert a.within_limits(0.0)
        assert a.within_limits(-90.0)
        assert not a.within_limits(90.1)
        assert not a.within_limits(-90.1)

    def test_unbounded_axis_accepts_anything(self):
        a = AxisConfig(name="j", slave_position=0)
        assert a.within_limits(1e6)
        assert a.within_limits(-1e6)

    def test_clamp(self):
        a = AxisConfig(
            name="j", slave_position=0,
            limits=LimitConfig(min_deg=-10.0, max_deg=10.0),
        )
        assert a.clamp_position_deg(50.0) == 10.0
        assert a.clamp_position_deg(-50.0) == -10.0
        assert a.clamp_position_deg(3.0) == 3.0
