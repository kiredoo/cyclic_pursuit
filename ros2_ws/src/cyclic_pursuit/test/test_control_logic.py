"""Pure controller helper tests; runnable without ROS 2 or Crazyflie hardware."""
import math
from types import SimpleNamespace

import pytest

from cyclic_pursuit.functions.utils import (
    gamma_ij,
    pij,
    u_t,
    validate_swarm_configuration,
    yaw_from_quaternion,
)


def test_validate_active_configuration():
    validate_swarm_configuration(['cf1', 'cf2'], ['1', '2'], 2)
    with pytest.raises(ValueError):
        validate_swarm_configuration(['cf1', 'cf2'], ['1'], 2)
    with pytest.raises(ValueError):
        validate_swarm_configuration(['cf1', 'cf1'], ['1', '2'], 2)
    with pytest.raises(ValueError):
        validate_swarm_configuration([], [], 0)


def test_yaw_from_unit_quaternion():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sqrt(0.5), w=math.sqrt(0.5))
    assert yaw_from_quaternion(q) == pytest.approx(math.pi / 2)
    identity = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
    assert yaw_from_quaternion(identity) == pytest.approx(0.0)


def test_pairwise_geometry_uses_other_agent_states():
    # Current agent [0,0,0]; other agents [3,4,0] and [1,0,0].
    others = [3.0, 4.0, 0.0, 1.0, 0.0, 0.0]
    assert pij(others, 3, [0.0, 0.0, 0.0]) == pytest.approx([5.0, 1.0])
    assert gamma_ij(others, 3, [0.0, 0.0, 0.0]) == pytest.approx([
        math.atan2(4, 3), 0.0,
    ])


def test_beacon_control_uses_configured_gain():
    state = [2.0, 0.0]
    target = [0.0, 0.0]
    base = u_t(state, target, 0.0, gain=0.1, psi_limit=2 * math.pi, shape=2.0, target_radius=0.8)
    double = u_t(state, target, 0.0, gain=0.2, psi_limit=2 * math.pi, shape=2.0, target_radius=0.8)
    assert base != 0.0
    assert double == pytest.approx(2 * base)
