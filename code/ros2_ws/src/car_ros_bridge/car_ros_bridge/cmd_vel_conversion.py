"""Pure conversion from REP-103 Twist commands to safe Ackermann targets."""

from __future__ import annotations

from dataclasses import dataclass
import math


WHEELBASE_M = 0.1425
CONSERVATIVE_MIN_TURN_RADIUS_M = 0.49
MAX_LEFT_STEERING_RAD = 0.336
MAX_RIGHT_STEERING_RAD = -0.32


class UnsafeTwist(ValueError):
    """A Twist cannot be represented safely by this Ackermann vehicle."""


@dataclass(frozen=True, slots=True)
class AckermannTarget:
    speed_mm_s: float
    steering_rad: float
    forward: bool


def twist_to_ackermann(
    linear_x_m_s: float,
    angular_z_rad_s: float,
    *,
    max_speed_m_s: float = 0.30,
    wheelbase_m: float = WHEELBASE_M,
    epsilon: float = 1e-4,
) -> AckermannTarget:
    """Convert a planar Twist without silently changing its requested curvature.

    ``linear.x == 0, angular.z != 0`` is an in-place rotation and is rejected.
    With reverse velocity the bicycle relation naturally reverses steering so a
    positive ROS yaw rate remains a left turn in the map frame.
    """

    v = float(linear_x_m_s)
    omega = float(angular_z_rad_s)
    if not all(math.isfinite(value) for value in (v, omega, max_speed_m_s, wheelbase_m)):
        raise UnsafeTwist("Twist and vehicle limits must be finite")
    if max_speed_m_s <= 0.0 or wheelbase_m <= 0.0:
        raise ValueError("max_speed_m_s and wheelbase_m must be positive")
    if abs(v) < epsilon:
        if abs(omega) >= epsilon:
            raise UnsafeTwist("Ackermann base rejects in-place rotation commands")
        return AckermannTarget(0.0, 0.0, True)
    if abs(v) > max_speed_m_s + epsilon:
        raise UnsafeTwist("linear velocity exceeds configured base speed limit")
    steering = math.atan(wheelbase_m * omega / v)
    if steering < MAX_RIGHT_STEERING_RAD - epsilon or steering > MAX_LEFT_STEERING_RAD + epsilon:
        raise UnsafeTwist("requested curvature exceeds steering geometry")
    if abs(steering) >= epsilon:
        radius = abs(wheelbase_m / math.tan(steering))
        if radius + epsilon < CONSERVATIVE_MIN_TURN_RADIUS_M:
            raise UnsafeTwist("requested curvature is below 0.49 m minimum turning radius")
    return AckermannTarget(abs(v) * 1000.0, steering, v >= 0.0)
