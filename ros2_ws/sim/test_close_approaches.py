"""Regression tests for controlled close-approach diagnostics."""

import math
from pathlib import Path

import pytest

from diagnose_close_approaches import components, simulate
from paper_validation import (
    g_function, model_commands, params_from_yaml, random_initial_conditions,
)


def params():
    return params_from_yaml(
        Path(__file__).resolve().parents[1] / "src/cyclic_pursuit/config/cyclic.yaml"
    )


@pytest.mark.parametrize("seed", [0, 7, 19, 23, 47])
def test_original_ablation_matches_existing_control_model(seed):
    p = params()
    states = random_initial_conditions(seed)
    commands, breakdown = components(states, p, "original")
    assert commands == pytest.approx(model_commands(states, p), abs=1e-10)
    assert len(breakdown) == p["n"]


def test_repulsion_is_finite_at_contact_and_zero_at_desired_distance():
    p = params()
    strength_at_zero = g_function(0., p["cj"], p["d_0"])
    assert strength_at_zero == pytest.approx(-math.log(p["cj"]))
    assert g_function(p["d_0"], p["cj"], p["d_0"]) == pytest.approx(0.)
    # Two robots pointing directly at one another: beta=0 so pair steering=0,
    # despite the finite negative gain at close range.
    states = [(0., 0., 0.), (.4, 0., math.pi), (5., 5., 0.)]
    _, detailed = components(states, p, "original")
    assert detailed[0]["terms"][1]["visible"]
    assert detailed[1]["terms"][0]["visible"]
    assert detailed[0]["terms"][1]["inter_term_rad_s"] == pytest.approx(0., abs=1e-12)
    assert detailed[1]["terms"][0]["inter_term_rad_s"] == pytest.approx(0., abs=1e-12)


def test_seed23_close_approach_is_reproducible_without_ros2():
    result = simulate(params(), 23, duration=35., dt=.05)
    assert result["initial_min_m"] == pytest.approx(3.0655395642146477, abs=1e-6)
    assert result["closest"]["distance_m"] == pytest.approx(
        0.013637152448107988, abs=.001
    )
    assert result["first_below_ds"] is not None
    assert result["closest"]["time_s"] == pytest.approx(6.30, abs=.05)
