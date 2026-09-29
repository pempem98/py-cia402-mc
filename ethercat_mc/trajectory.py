"""Setpoint generators for cyclic modes.

In CSP/CSV the master must produce a new setpoint every cycle. These
generators are pure state machines stepped by `update(dt)`: no threads, no I/O,
so they can be unit tested without hardware.

All quantities are in encoder counts and counts/s.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class TrapezoidalProfile:
    """Trapezoidal velocity profile toward a position goal.

    The profile is re-planned every cycle from the current state, so the goal
    can be changed at any time and the motion follows smoothly.
    """

    position: float = 0.0  #: current setpoint, counts
    velocity: float = 0.0  #: current setpoint velocity, counts/s
    goal: float = 0.0  #: target position, counts
    max_velocity: float = 1000.0  #: counts/s, must be > 0
    max_acceleration: float = 1000.0  #: counts/s^2, must be > 0
    #: Distance below which the goal counts as reached, in counts.
    tolerance: float = 1.0

    def reset(self, position: float) -> None:
        """Snap the profile to `position` and stop. Use before enabling a drive
        so the first setpoint equals the measured position (no jump)."""
        self.position = float(position)
        self.goal = float(position)
        self.velocity = 0.0

    def set_goal(self, goal: float) -> None:
        self.goal = float(goal)

    @property
    def done(self) -> bool:
        return abs(self.goal - self.position) <= self.tolerance and self.velocity == 0.0

    def update(self, dt: float) -> float:
        """Advance the profile by `dt` seconds and return the new setpoint.

        The velocity is chosen first, subject to the acceleration limit, and
        the position follows from it. The braking distance v^2/(2a) decides
        whether to speed up or slow down. Velocity is signed, so reversing the
        goal decelerates through zero and accelerates back without a special
        case.
        """
        if dt <= 0.0:
            return self.position

        error = self.goal - self.position
        dv_max = self.max_acceleration * dt

        if error == 0.0 and self.velocity == 0.0:
            return self.position

        direction = math.copysign(1.0, error) if error != 0.0 else 0.0

        # Speed ceiling that still allows a full-deceleration stop on the goal.
        # sqrt(2*a*|error|) is the speed from which braking at max_acceleration
        # arrives at zero exactly at the goal, so it is the upper bound for
        # this cycle; max_velocity caps it during cruise. Using it as a ceiling
        # (rather than as a separate braking mode) makes the approach
        # continuous, so no cycle needs an out-of-limit deceleration to settle.
        # The continuous bound sqrt(2*a*|e|) is optimistic for a sampled loop:
        # the axis still travels v*dt during this cycle before it can brake
        # again. Solving |e| >= v*dt + v^2/(2a) for v gives the discrete bound
        # below, which keeps the approach inside the acceleration limit all the
        # way to the goal instead of needing an oversized final step.
        a = self.max_acceleration
        discrete_bound = (
            -a * dt + math.sqrt(a * a * dt * dt + 8.0 * a * abs(error))
        ) / 2.0
        ceiling = min(self.max_velocity, discrete_bound)
        desired = direction * ceiling

        # Slew-rate limit: never change speed by more than a * dt in one cycle.
        target_velocity = max(
            min(desired, self.velocity + dv_max), self.velocity - dv_max
        )

        # Never step past the goal: cap at the speed that lands exactly on it.
        # This only reduces |velocity|, so the slew limit above still holds.
        if target_velocity * direction > 0.0 and abs(target_velocity * dt) > abs(error):
            target_velocity = error / dt

        # Settle: zero the speed only when the *current* speed is within one
        # deceleration step of zero. Testing the capped target instead would
        # hide a larger real jump, since the cap has already lowered it.
        if abs(self.velocity) <= dv_max and abs(target_velocity * dt) >= abs(error):
            self.position = self.goal
            self.velocity = 0.0
            return self.position

        self.velocity = target_velocity
        self.position += self.velocity * dt
        return self.position


@dataclass
class VelocityRamp:
    """Rate-limited velocity setpoint, for CSV mode.

    `target` is the commanded velocity; `value` follows it at no more than
    `max_acceleration`, so a step command does not shock the mechanism.
    """

    value: float = 0.0  #: current velocity setpoint, counts/s
    target: float = 0.0  #: commanded velocity, counts/s
    max_velocity: float = 1000.0
    max_acceleration: float = 1000.0

    def reset(self) -> None:
        self.value = 0.0
        self.target = 0.0

    def set_target(self, velocity: float) -> None:
        self.target = max(-self.max_velocity, min(self.max_velocity, float(velocity)))

    def update(self, dt: float) -> float:
        if dt <= 0.0:
            return self.value
        dv = self.target - self.value
        limit = self.max_acceleration * dt
        self.value += math.copysign(min(abs(dv), limit), dv) if dv != 0.0 else 0.0
        return self.value


@dataclass
class TorqueRamp:
    """Rate-limited torque setpoint, for CST mode. Units are per-mille of
    rated torque, matching object 0x6071."""

    value: float = 0.0
    target: float = 0.0
    max_torque: float = 300.0  #: per-mille
    #: Maximum change in per-mille per second.
    max_rate: float = 1000.0

    def reset(self) -> None:
        self.value = 0.0
        self.target = 0.0

    def set_target(self, torque_permille: float) -> None:
        self.target = max(
            -self.max_torque, min(self.max_torque, float(torque_permille))
        )

    def update(self, dt: float) -> float:
        if dt <= 0.0:
            return self.value
        dv = self.target - self.value
        limit = self.max_rate * dt
        self.value += math.copysign(min(abs(dv), limit), dv) if dv != 0.0 else 0.0
        return self.value


def sync_duration(
    distance: float, max_velocity: float, max_acceleration: float
) -> float:
    """Time a trapezoidal move of `distance` counts takes, in seconds.

    Used to scale several axes onto a common duration so they start and finish
    together (coordinated motion).
    """
    distance = abs(distance)
    if distance == 0.0 or max_velocity <= 0.0 or max_acceleration <= 0.0:
        return 0.0
    # Distance covered while ramping up and down at full acceleration.
    ramp_distance = max_velocity * max_velocity / max_acceleration
    if distance < ramp_distance:
        # Triangular profile: never reaches max_velocity.
        peak = math.sqrt(distance * max_acceleration)
        return 2.0 * peak / max_acceleration
    ramp_time = max_velocity / max_acceleration
    cruise_time = (distance - ramp_distance) / max_velocity
    return 2.0 * ramp_time + cruise_time


def scale_for_duration(
    distance: float,
    duration: float,
    max_acceleration: float,
    max_velocity: float | None = None,
) -> tuple[float, float]:
    """Pick (velocity, acceleration) so a move of `distance` takes `duration`.

    Shapes a symmetric trapezoid that spends a third of the time ramping up, a
    third cruising and a third ramping down: for ramp time tr = T/3,
    `distance = v * (T - tr)` and `v = a * tr` give v = 1.5 * d / T and
    a = 4.5 * d / T^2.

    That 1.5 factor means the peak velocity is half again the *average* speed,
    so scaling a move onto a duration can ask for more speed than the axis
    allows even when the duration itself came from `sync_duration`. Both caps
    are therefore applied, and the returned pair is always achievable.
    Returns (0, max_acceleration) for a zero-length move.
    """
    distance = abs(distance)
    if distance == 0.0 or duration <= 0.0:
        return 0.0, max_acceleration

    velocity = 1.5 * distance / duration
    acceleration = 4.5 * distance / (duration * duration)

    if max_velocity is not None and velocity > max_velocity:
        velocity = max_velocity
    if acceleration > max_acceleration:
        acceleration = max_acceleration
    return velocity, acceleration
