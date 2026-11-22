#!/usr/bin/env python3
"""Paper-aligned convergence and separation evaluation over 100 seeded trials.

All 100 trials use the same 3-agent initial sampling distribution:
angles around equally spaced spokes with +/-0.5 rad jitter, radii sampled
uniformly in [1,3] meters, and headings in [-pi,pi]. This is NOT an exact
reproduction of the paper's 4-/6-vehicle experiment or initial distribution.
"""
import argparse
import csv
import json
import math
import statistics
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from paper_validation import params_from_yaml, paper_parameter_checks, rollout


def main():
    parser = argparse.ArgumentParser(description="Reproducible 3-agent convergence study")
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--duration", type=float, default=160.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/paper-study"))
    parser.add_argument("--compare-ros2", type=Path, default=None,
                        help="Path to an existing ROS 2 random-seed-19 metrics.json")
    args = parser.parse_args()
    if args.trials < 1 or args.trials > 1000 or args.duration <= 10 or not 0 < args.dt <= .2:
        parser.error("Use 1-1000 trials, >10 simulated seconds, dt in (0,.2]")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    p = params_from_yaml(Path(__file__).resolve().parents[1] /
                         "src/cyclic_pursuit/config/cyclic.yaml")
    checks = paper_parameter_checks(p)
    if not all(checks["checks"].values()):
        raise AssertionError("The configured parameters violate a paper assumption: "
                             + repr(checks["checks"]))
    results = [rollout(p, seed=i, duration=args.duration, dt=args.dt)
               for i in range(args.trials)]
    columns = list(results[0])
    with (args.output_dir / "trials.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        for item in results:
            writer.writerow(item)

    dwell = [r["first_five_second_dwell_s"] for r in results
             if r["first_five_second_dwell_s"] is not None]
    final = [r["final_in_radial_heading_band"] for r in results]
    sep = [r["any_separation_below_ds"] for r in results]
    initial_below = [r["separation_below_ds_at_t0"] for r in results]
    post5_below = [r["minimum_pairwise_after_5s_m"] < float(p["ds"]) for r in results]
    summary = {
        "trials": args.trials, "duration_s": args.duration, "dt_s": args.dt,
        "seed_range": [0, args.trials-1],
        "initial_sampling": "3 agents; radial U(1,3)m; heading U(-pi,pi); sector angles 2pi*i/3 plus U(-0.5,0.5) rad",
        "paper_equilibrium_radius_m": checks["equilibrium_radius_m"],
        "paper_checks": checks,
        "metric_definition": (
            "First 5 s continuous period when all agents' absolute radial "
            "errors are <0.1 m and absolute gamma-pi/2 errors are <0.2 rad. "
            "This does not guarantee errors stay small thereafter."
        ),
        "first_dwell_achieved_count": len(dwell),
        "first_dwell_achieved_fraction": len(dwell)/args.trials,
        "median_first_dwell_time_s_when_achieved": (
            statistics.median(dwell) if dwell else None
        ),
        "final_in_band_count": sum(final),
        "final_in_band_fraction": sum(final)/args.trials,
        "min_sep_below_ds_anytime_count": sum(sep),
        "min_sep_below_ds_anytime_fraction": sum(sep)/args.trials,
        "initial_below_ds_count": sum(initial_below),
        "below_ds_after_first_5s_count": sum(post5_below),
        "minimum_observed_separation_m": min(
            r["minimum_pairwise_m"] for r in results
        ),
        "median_final_max_radius_error_m": statistics.median(
            r["final_max_abs_radius_error_m"] for r in results
        ),
        "model_limitations": [
            "Planar nonholonomic point robots; no dynamics, body size or collision contacts",
            "No sensor noise, delays or actuator saturation in this 100-trial baseline",
            "Three robots and different initial sampling from the paper's four/six vehicle experiments",
            "Point-in-time geometric criteria rather than global asymptotic stability",
        ],
    }

    if args.compare_ros2 is not None:
        ros = json.loads(args.compare_ros2.read_text(encoding="utf-8"))
        pure = rollout(p, 19, duration=float(ros["simulated_duration_s"]),
                       dt=float(ros["integration_dt_s"]))
        difference = {
            key: abs(float(pure[key])-float(ros[key]))
            for key in (
                "final_mean_radius_m", "final_radius_std_m",
                "minimum_pairwise_separation_m",
            ) if key in pure
        }
        # Renaming: pure model uses minimum_pairwise_m.
        difference["minimum_pairwise_separation_m"] = abs(
            pure["minimum_pairwise_m"] - ros["minimum_pairwise_separation_m"]
        )
        difference["final_mean_radius_m"] = abs(
            pure["final_mean_radius_m"] - ros["final_mean_radius_m"]
        )
        difference["final_radius_std_m"] = abs(
            pure["final_radius_std_m"] - ros["final_radius_std_m"]
        )
        summary["ros2_seed19_crosscheck"] = {
            "reference": str(args.compare_ros2),
            "absolute_differences_m": difference,
            "max_absolute_difference_m": max(difference.values()),
            "tolerance_m": 0.10,
            "matched": all(x < .10 for x in difference.values()),
        }
        if not summary["ros2_seed19_crosscheck"]["matched"]:
            (args.output_dir/"summary.json").write_text(
                json.dumps(summary, indent=2)+"\n", encoding="utf-8"
            )
            raise AssertionError("Fast model / ROS2 trajectory cross-check exceeded 0.10m")

    (args.output_dir/"summary.json").write_text(
        json.dumps(summary, indent=2)+"\n", encoding="utf-8"
    )

    fig, ax = plt.subplots(figsize=(8, 4.5))
    all_times = [float(x) if x is not None else args.duration
                 for x in (r["first_five_second_dwell_s"] for r in results)]
    ax.hist(all_times, bins=20)
    ax.set(xlabel="First five-second dwell time (s); right edge includes censored runs",
           ylabel="Number of trials", title="100 seeded initial conditions")
    ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(args.output_dir/"convergence_times.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 5))
    x = [r["minimum_pairwise_m"] for r in results]
    y = [r["final_max_abs_radius_error_m"] for r in results]
    ax.scatter(x, y, s=16, alpha=.75)
    ax.axvline(float(p["ds"]), linestyle="--", label="Configured ds")
    ax.axhline(.1, linestyle=":", label="Final radial tolerance")
    ax.set(xlabel="Minimum pairwise distance (m)",
           ylabel="Final maximum absolute radius error (m)",
           title="Radial accuracy vs. closest approach")
    ax.grid(alpha=.25)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir/"distance_vs_error.png", dpi=140)
    plt.close(fig)

    rows = [
        "# Paper-aligned 3-agent convergence study", "",
        "Model: implemented cyclic-pursuit law, ideal planar unicycle dynamics.",
        "100 initial conditions are seeded and drawn from the explicitly recorded",
        "distribution. This is not an exact replication of the paper's experiment.",
        "",
        f"- Expected equilibrium radius: {checks['equilibrium_radius_m']:.6f} m",
        f"- Completed 5-second radial-and-heading dwell: {len(dwell)}/{args.trials}",
        f"- Final snapshot inside radial-and-heading tolerances: {sum(final)}/{args.trials}",
        f"- Minimum pairwise distance below ds at any point: {sum(sep)}/{args.trials}",
        f"- Initially below ds: {sum(initial_below)}/{args.trials}",
        f"- Below ds after t=5s: {sum(post5_below)}/{args.trials}",
        f"- Minimum observed pairwise separation: {summary['minimum_observed_separation_m']:.4f} m",
        "",
        "A run not meeting the finite-horizon test is NOT evidence of asymptotic",
        "nonconvergence; a close pass is NOT equivalent to physical collision.",
    ]
    if "ros2_seed19_crosscheck" in summary:
        rows += ["", "## ROS 2 reference comparison",
                 "Difference in final and minimum-distance metrics (m):",
                 json.dumps(summary["ros2_seed19_crosscheck"], indent=2)]
    (args.output_dir/"summary.md").write_text("\n".join(rows)+"\n", encoding="utf-8")
    print("PAPER_STUDY_SUMMARY="+json.dumps({
        key: value for key, value in summary.items()
        if key not in ("paper_checks", "model_limitations")
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
