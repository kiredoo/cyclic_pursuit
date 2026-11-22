"""The plant's units and motion are tested independently of ROS 2."""

import math

import pytest

from kinematics import command_to_unicycle, integrate_unicycle


def test_historical_yaw_command_conversion():
    v, omega = command_to_unicycle(0.3, -90.0)
    assert v == pytest.approx(0.3)
    assert omega == pytest.approx(math.pi / 2)


def test_straight_motion():
    x, y, yaw = integrate_unicycle((0.0, 0.0, 0.0), 0.3, 0.0, 1.0)
    assert (x, y, yaw) == pytest.approx((0.3, 0.0, 0.0))


def test_turning_motion():
    x, y, yaw = integrate_unicycle((0.0, 0.0, 0.0), 1.0, math.pi / 2, 1.0)
    assert x > 0.0 and y > 0.0
    assert yaw == pytest.approx(math.pi / 2)


def test_nonfinite_input_rejected():
    with pytest.raises(ValueError):
        command_to_unicycle(0.3, float("nan"))
    with pytest.raises(ValueError):
        integrate_unicycle((0, 0, 0), 0.3, 0.0, -0.01)
