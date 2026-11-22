"""Paper equations, numerical diagnostics and ROS 2 control-law parity tests.

This is a pure-Python pytest module; it does not import rclpy or start drones.
"""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from paper_validation import (
    equilibrium_radius, paper_parameter_checks, params_from_yaml,
    model_commands, random_initial_conditions, rollout,
)
from cyclic_pursuit.functions.utils import (
    Bd, gamma_ij, pij, u_t, u_t_j, visibility_zone_detector
)


def p():
    return params_from_yaml(
        Path(__file__).resolve().parents[1] / "src/cyclic_pursuit/config/cyclic.yaml"
    )


def test_equilibrium_radius_matches_equation_14_and_all_paper_checks():
    params = p()
    radius = equilibrium_radius(params)
    assert radius == pytest.approx(2.0033068269701086, abs=1e-8)
    g = math.log(((params["c"]-1)*radius + params["rho_o"]) /
                 (params["c"]*params["rho_o"]))
    assert params["v"]/radius == pytest.approx(
        params["k"]*g*math.pi/2, abs=1e-12
    )
    results = paper_parameter_checks(params)
    assert all(results["checks"].values()), results
    assert results["eq45_lhs"] == pytest.approx(.31/.17)


def test_fast_model_commands_match_original_ros2_math_helpers():
    params = p()
    for seed in (0, 7, 19, 23, 41):
        states = random_initial_conditions(seed)
        expected = []
        for i, state in enumerate(states):
            others = [value for j, other in enumerate(states)
                      if i != j for value in other]
            dists = pij(others, params["n"], state)
            angles = Bd(gamma_ij(others, params["n"], state))
            visible = visibility_zone_detector(
                params["dl"], params["ds"], params["av"], angles, dists
            )
            ui = u_t(state[:2], (0, 0), state[2],
                     gain=params["k"], psi_limit=params["psi"],
                     shape=params["c"], target_radius=params["rho_o"])
            ui += u_t_j(dists, angles, params["cj"], params["d_0"],
                        params["kj"], visible)
            expected.append(float(ui))
        assert model_commands(states, params) == pytest.approx(expected, abs=1e-10)


def test_fast_random_trajectory_matches_previous_ros2_seed_19():
    result = rollout(p(), seed=19, duration=35., dt=.05)
    # Independent previously observed ROS 2 outputs from the draft PR's
    # 35-second seven-case sweep, not a generalized convergence assertion.
    assert result["final_mean_radius_m"] == pytest.approx(2.091184530502538, abs=.001)
    assert result["final_radius_std_m"] == pytest.approx(.1492477971468731, abs=.001)
    assert result["minimum_pairwise_m"] == pytest.approx(.9040193419045954, abs=.001)
    assert result["any_separation_below_ds"] is True
