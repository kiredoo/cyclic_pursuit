"""Small explicit kinematic model for the historical cyclic-pursuit interface.

The existing controller publishes a Twist whose angular.z is NEGATIVE
DEGREES/SECOND, per its own original conversion. This adapter cancels that
sign and converts to radians/second for mathematical yaw integration.

This is a research-model convention, NOT a verified Crazyswarm2 interface.
"""

import math


def command_to_unicycle(linear_x, angular_z):
    """Interpret controller Twist fields as (forward m/s, yaw rad/s)."""
    v = float(linear_x)
    omega_deg = float(angular_z)
    if not (math.isfinite(v) and math.isfinite(omega_deg)):
        raise ValueError("Non-finite controller command")
    return v, -math.radians(omega_deg)


def integrate_unicycle(state, v, omega, dt):
    """Midpoint update for xdot=v*cos(theta), ydot=v*sin(theta), theta_dot=omega."""
    if dt <= 0 or not all(math.isfinite(float(x)) for x in (*state, v, omega, dt)):
        raise ValueError("Invalid state, command, or time step")
    x, y, theta = state
    theta_mid = theta + omega * dt / 2.0
    theta_new = theta + omega * dt
    return (
        x + v * math.cos(theta_mid) * dt,
        y + v * math.sin(theta_mid) * dt,
        math.atan2(math.sin(theta_new), math.cos(theta_new)),
    )
