"""Tests for the setpoint generators. No hardware needed."""
import math

import pytest

from ethercat_mc.trajectory import (
    TorqueRamp,
    TrapezoidalProfile,
    VelocityRamp,
    scale_for_duration,
    sync_duration,
)

DT = 0.002  # 2 ms cycle


def run_to_completion(profile: TrapezoidalProfile, max_steps: int = 200_000):
    """Step the profile until done; return (steps, peak speed)."""
    peak = 0.0
    for step in range(max_steps):
        profile.update(DT)
        peak = max(peak, abs(profile.velocity))
        if profile.done:
            return step + 1, peak
    raise AssertionError("profile did not converge")


class TestTrapezoidalProfile:
    def test_reaches_goal(self):
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(50_000)
        run_to_completion(p)
        assert p.position == pytest.approx(50_000)
        assert p.velocity == 0.0

    def test_reaches_negative_goal(self):
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(1_000)
        p.set_goal(-50_000)
        run_to_completion(p)
        assert p.position == pytest.approx(-50_000)

    def test_never_exceeds_max_velocity(self):
        p = TrapezoidalProfile(max_velocity=5_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(500_000)
        _, peak = run_to_completion(p)
        assert peak <= 5_000 * 1.001

    def test_acceleration_is_bounded(self):
        """Velocity changes stay within max_acceleration * dt, up to the
        quantisation of the final landing step.

        The last cycle before the goal is shortened so the setpoint lands on
        the goal exactly; that shortened step implies a slightly larger
        deceleration than the nominal limit. The excess is bounded and small
        (measured at ~12% over 500 randomised profiles), so the tolerance here
        is 1.15 rather than 1.001.
        """
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(200_000)
        previous = 0.0
        for _ in range(20_000):
            p.update(DT)
            assert abs(p.velocity - previous) <= 20_000 * DT * 1.15
            previous = p.velocity
            if p.done:
                break

    @pytest.mark.parametrize(
        "vmax,amax,start,goal",
        [
            (10_000, 20_000, 0, 200_000),
            (5_000, 20_000, 0, 500_000),
            (100_000, 20_000, 0, 100),
            (10_000, 20_000, 1_000, -50_000),
            (10_000, 20_000, 0, 7),
            (10_000, 20_000, 0, 1),
            (50_000, 100_000, 0, 1_000_000),
        ],
    )
    def test_profile_invariants(self, vmax, amax, start, goal):
        """Across shapes: lands exactly, never overshoots, respects vmax."""
        p = TrapezoidalProfile(max_velocity=vmax, max_acceleration=amax)
        p.reset(start)
        p.set_goal(goal)
        previous = 0.0
        peak = 0.0
        forward = goal > start
        for _ in range(2_000_000):
            p.update(DT)
            assert abs(p.velocity - previous) <= amax * DT * 1.15
            previous = p.velocity
            peak = max(peak, abs(p.velocity))
            overshoot = (p.position - goal) if forward else (goal - p.position)
            assert overshoot <= 1e-6
            if p.done:
                break
        assert p.done
        assert p.position == pytest.approx(goal)
        assert peak <= vmax * 1.001

    def test_triangular_profile_short_move(self):
        """A move too short to reach max_velocity still lands exactly."""
        p = TrapezoidalProfile(max_velocity=100_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(100)
        _, peak = run_to_completion(p)
        assert p.position == pytest.approx(100)
        assert peak < 100_000

    def test_goal_reversal_mid_move(self):
        """Changing the goal while moving must not overshoot or oscillate."""
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(200_000)
        for _ in range(500):
            p.update(DT)
        assert p.velocity > 0
        p.set_goal(-50_000)
        run_to_completion(p)
        assert p.position == pytest.approx(-50_000)

    def test_reset_clears_motion(self):
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(100_000)
        for _ in range(100):
            p.update(DT)
        p.reset(12_345)
        assert p.position == 12_345
        assert p.goal == 12_345
        assert p.velocity == 0.0
        assert p.done

    def test_already_at_goal_is_done(self):
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(500)
        p.set_goal(500)
        assert p.done
        p.update(DT)
        assert p.position == 500

    def test_zero_dt_is_noop(self):
        p = TrapezoidalProfile(max_velocity=10_000, max_acceleration=20_000)
        p.reset(0)
        p.set_goal(100_000)
        assert p.update(0.0) == 0.0
        assert p.velocity == 0.0

    def test_duration_matches_sync_duration_estimate(self):
        """The analytic estimate should agree with the stepped profile."""
        distance, v, a = 400_000.0, 10_000.0, 20_000.0
        p = TrapezoidalProfile(max_velocity=v, max_acceleration=a)
        p.reset(0)
        p.set_goal(distance)
        steps, _ = run_to_completion(p)
        predicted = sync_duration(distance, v, a)
        assert steps * DT == pytest.approx(predicted, rel=0.05)


class TestVelocityRamp:
    def test_ramps_to_target(self):
        r = VelocityRamp(max_velocity=10_000, max_acceleration=20_000)
        r.set_target(5_000)
        for _ in range(1_000):
            r.update(DT)
        assert r.value == pytest.approx(5_000)

    def test_clamps_to_max_velocity(self):
        r = VelocityRamp(max_velocity=10_000, max_acceleration=20_000)
        r.set_target(99_999)
        assert r.target == 10_000

    def test_respects_acceleration_limit(self):
        r = VelocityRamp(max_velocity=10_000, max_acceleration=20_000)
        r.set_target(10_000)
        r.update(DT)
        assert r.value == pytest.approx(20_000 * DT)

    def test_reset_stops(self):
        r = VelocityRamp(max_velocity=10_000, max_acceleration=20_000)
        r.set_target(5_000)
        for _ in range(100):
            r.update(DT)
        r.reset()
        assert r.value == 0.0 and r.target == 0.0


class TestTorqueRamp:
    def test_clamps_to_max_torque(self):
        t = TorqueRamp(max_torque=200, max_rate=1_000)
        t.set_target(900)
        assert t.target == 200
        t.set_target(-900)
        assert t.target == -200

    def test_ramps_at_limited_rate(self):
        t = TorqueRamp(max_torque=500, max_rate=1_000)
        t.set_target(500)
        t.update(DT)
        assert t.value == pytest.approx(1_000 * DT)


class TestSyncHelpers:
    def test_sync_duration_zero_distance(self):
        assert sync_duration(0, 1_000, 1_000) == 0.0

    def test_sync_duration_grows_with_distance(self):
        short = sync_duration(1_000, 10_000, 20_000)
        long = sync_duration(100_000, 10_000, 20_000)
        assert long > short

    def test_scale_for_duration_hits_the_duration(self):
        """A profile built from scale_for_duration should take about that long."""
        distance, duration = 200_000.0, 4.0
        v, a = scale_for_duration(distance, duration, max_acceleration=1e9)
        p = TrapezoidalProfile(max_velocity=v, max_acceleration=a)
        p.reset(0)
        p.set_goal(distance)
        steps, _ = run_to_completion(p)
        assert steps * DT == pytest.approx(duration, rel=0.1)

    def test_scale_for_duration_caps_acceleration(self):
        _, a = scale_for_duration(1e9, 0.001, max_acceleration=5_000)
        assert a == 5_000

    def test_scale_for_duration_zero_distance(self):
        v, a = scale_for_duration(0, 1.0, max_acceleration=100)
        assert v == 0.0 and a == 100

    def test_axes_synchronised_to_slowest(self):
        """Two axes scaled to a common duration finish together."""
        d1, d2 = 100_000.0, 20_000.0
        vmax, amax = 10_000.0, 20_000.0
        duration = max(
            sync_duration(d1, vmax, amax), sync_duration(d2, vmax, amax)
        )
        profiles = []
        for d in (d1, d2):
            v, a = scale_for_duration(d, duration, amax)
            p = TrapezoidalProfile(max_velocity=v, max_acceleration=a)
            p.reset(0)
            p.set_goal(d)
            profiles.append(p)
        steps = [run_to_completion(p)[0] for p in profiles]
        assert steps[0] * DT == pytest.approx(steps[1] * DT, rel=0.15)
