#!/usr/bin/env python3
"""Explain close approaches in the existing planar cyclic-pursuit controller.

This is a hardware-free *diagnostic*, not a controller redesign or a proof of
safety. All variants use the same 100 seeds, starting states and kinematics:

1. original: paper-like forward sector plus the omnidirectional ds circle
2. omnidirectional: counterfactual 360-degree perception within dl (4 m)
3. no_interaction: diagnostic removal of inter-agent steering

Neither counterfactual is recommended for physical deployment. No control
parameters are retuned, and the simulated forward speed always remains 0.3 m/s.
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

from paper_validation import (
    g_function, model_commands, params_from_yaml, random_initial_conditions,
)

TAU = 2.0 * math.pi
VARIANTS = ("original", "omnidirectional", "no_interaction")


def components(states, p, variant="original"):
    """Return total angular commands and per-agent bearing/visibility terms."""
    if variant not in VARIANTS:
        raise ValueError(variant)
    cb, cv = float(p["c"]), float(p["cj"])
    kb, kv = float(p["k"]), float(p["kj"])
    rho0, d0 = float(p["rho_o"]), float(p["d_0"])
    ds, dl, av, psi = [float(p[key]) for key in ("ds", "dl", "av", "psi")]
    commands, breakdowns = [], []
    for i, (x, y, theta) in enumerate(states):
        radius = math.hypot(x, y)
        gamma = (math.atan2(-y, -x) - theta) % TAU
        alpha = gamma if gamma <= psi else gamma - TAU
        beacon = (
            0.0 if radius < .1
            else kb * g_function(radius, cb, rho0) * alpha
        )
        terms = {}
        for j, (other_x, other_y, _) in enumerate(states):
            if i == j:
                continue
            distance = math.hypot(other_x - x, other_y - y)
            beta = (math.atan2(other_y - y, other_x - x) - theta + math.pi) % TAU - math.pi
            short_range = distance < ds
            forward_sector = distance <= dl and abs(beta) <= av
            visible = short_range or forward_sector
            if variant == "omnidirectional":
                visible = distance <= dl
            if variant == "no_interaction":
                visible = False
            gain = 0.0 if distance < 1e-9 else g_function(distance, cv, d0)
            term = kv * gain * beta if visible else 0.0
            terms[j] = dict(
                distance_m=distance,
                bearing_rad=beta,
                visible=visible,
                short_range=short_range,
                forward_sector=forward_sector,
                gain=gain,
                inter_term_rad_s=term,
            )
        total_inter = sum(t["inter_term_rad_s"] for t in terms.values())
        commands.append(beacon + total_inter)
        breakdowns.append(dict(beacon_rad_s=beacon, inter_rad_s=total_inter,
                               terms=terms))
    return commands, breakdowns


def pair_snapshot(seed, t, i, j, states, commands, breakdown):
    """Record both steering responses and the range closing velocity."""
    agent_i, agent_j = breakdown[i]["terms"][j], breakdown[j]["terms"][i]
    # Equation (30) of the paper: rho_ij_dot=-v(cos beta_ij+cos beta_ji).
    result = {
        "seed": seed, "time_s": t, "pair": [i, j],
        "distance_m": agent_i["distance_m"],
        "closing_speed_mps": 0.0,
        "visible_i": agent_i["visible"], "visible_j": agent_j["visible"],
        "forward_visible_i": agent_i["forward_sector"],
        "forward_visible_j": agent_j["forward_sector"],
        "beta_i_deg": math.degrees(agent_i["bearing_rad"]),
        "beta_j_deg": math.degrees(agent_j["bearing_rad"]),
        "pair_u_i_deg_s": math.degrees(agent_i["inter_term_rad_s"]),
        "pair_u_j_deg_s": math.degrees(agent_j["inter_term_rad_s"]),
        "beacon_u_i_deg_s": math.degrees(breakdown[i]["beacon_rad_s"]),
        "beacon_u_j_deg_s": math.degrees(breakdown[j]["beacon_rad_s"]),
        "total_u_i_deg_s": math.degrees(commands[i]),
        "total_u_j_deg_s": math.degrees(commands[j]),
        "true_states": [list(map(float, state)) for state in states],
    }
    result["closing_speed_mps"] = 0.3 * (
        math.cos(agent_i["bearing_rad"]) + math.cos(agent_j["bearing_rad"])
    )
    return result


def simulate(p, seed, variant="original", duration=35., dt=.05, trace=False):
    """Integrate midpoint unicycles and identify the globally closest pair."""
    states = [tuple(s) for s in random_initial_conditions(seed, int(p["n"]))]
    minimum = None
    first_crossing = None
    previous = {}
    first_below_d0 = {}
    trace_rows = []
    initial_min = None
    for step in range(round(duration / dt) + 1):
        t = round(step * dt, 9)
        angular, breakdown = components(states, p, variant)
        if variant == "original" and step % 100 == 0:
            expected = model_commands(states, p)
            if max(abs(a - b) for a, b in zip(angular, expected)) > 1e-10:
                raise AssertionError("Diagnostic controller diverged from reference")
        for i in range(len(states)):
            for j in range(i + 1, len(states)):
                distance = breakdown[i]["terms"][j]["distance_m"]
                if step == 0:
                    initial_min = distance if initial_min is None else min(initial_min, distance)
                snap = None
                if minimum is None or distance < minimum["distance_m"]:
                    snap = pair_snapshot(seed, t, i, j, states, angular, breakdown)
                    minimum = snap
                if first_crossing is None and distance < float(p["ds"]):
                    snap = snap or pair_snapshot(seed, t, i, j, states, angular, breakdown)
                    previous_snap = previous.get((i, j))
                    first_crossing = dict(snap)
                    if previous_snap is not None:
                        first_crossing["prior_to_threshold"] = previous_snap
                if (i, j) not in first_below_d0 and distance <= float(p["d_0"]):
                    first_below_d0[(i, j)] = pair_snapshot(
                        seed, t, i, j, states, angular, breakdown
                    )
                # Keep a compact series for plotting and diagnosing worst pair.
                if trace:
                    a, b = breakdown[i]["terms"][j], breakdown[j]["terms"][i]
                    trace_rows.append({
                        "time_s": t, "pair": f"{i}-{j}", "distance_m": distance,
                        "beta_i_deg": math.degrees(a["bearing_rad"]),
                        "beta_j_deg": math.degrees(b["bearing_rad"]),
                        "visible_i": int(a["visible"]),
                        "visible_j": int(b["visible"]),
                        "pair_turn_i_deg_s": math.degrees(a["inter_term_rad_s"]),
                        "pair_turn_j_deg_s": math.degrees(b["inter_term_rad_s"]),
                        "beacon_turn_i_deg_s": math.degrees(breakdown[i]["beacon_rad_s"]),
                        "beacon_turn_j_deg_s": math.degrees(breakdown[j]["beacon_rad_s"]),
                        "closing_speed_mps": float(p["v"]) * (
                            math.cos(a["bearing_rad"]) + math.cos(b["bearing_rad"])
                        ),
                    })
                # Prior state is the last sample before an inward crossing of ds.
                previous[(i, j)] = dict(
                    visible_i=breakdown[i]["terms"][j]["visible"],
                    visible_j=breakdown[j]["terms"][i]["visible"],
                    forward_visible_i=breakdown[i]["terms"][j]["forward_sector"],
                    forward_visible_j=breakdown[j]["terms"][i]["forward_sector"],
                    distance_m=distance, time_s=t,
                )
        if step == round(duration / dt):
            break
        successor = []
        for (x, y, theta), omega in zip(states, angular):
            mid = theta + omega * dt / 2
            successor.append((
                x + float(p["v"]) * math.cos(mid) * dt,
                y + float(p["v"]) * math.sin(mid) * dt,
                math.atan2(math.sin(theta + omega * dt), math.cos(theta + omega * dt)),
            ))
        states = successor
    return dict(
        seed=seed, variant=variant, initial_min_m=initial_min,
        closest=minimum, first_below_ds=first_crossing,
        below_ds=minimum["distance_m"] < float(p["ds"]),
        below_half_meter=minimum["distance_m"] < .5,
        below_quarter_meter=minimum["distance_m"] < .25,
        trace=trace_rows,
        visibility_at_d0=first_below_d0,
    )


def strip_unserializable(event):
    return event


def analyze(p, trials, duration, dt):
    variants = {}
    all_results = {}
    for variant in VARIANTS:
        results = [simulate(p, seed, variant, duration, dt) for seed in range(trials)]
        all_results[variant] = results
        minimums = [result["closest"]["distance_m"] for result in results]
        variants[variant] = {
            "trials": trials,
            "count_below_1m": sum(distance < float(p["ds"]) for distance in minimums),
            "count_below_0p5m": sum(distance < .5 for distance in minimums),
            "count_below_0p25m": sum(distance < .25 for distance in minimums),
            "minimum_distance_m": min(minimums),
            "median_minimum_distance_m": statistics.median(minimums),
            "seed23_minimum_m": (minimums[23] if trials > 23 else None),
        }
    close = [r for r in all_results["original"] if r["below_ds"]]
    original = all_results["original"]
    prior_both_visible = [
        r for r in close if r["first_below_ds"] is not None
        and all(r["first_below_ds"].get("prior_to_threshold", {}).get(k, False)
                for k in ("visible_i", "visible_j"))
    ]
    prior_any_visible = [
        r for r in close if r["first_below_ds"] is not None
        and any(r["first_below_ds"].get("prior_to_threshold", {}).get(k, False)
                for k in ("visible_i", "visible_j"))
    ]
    return {
        "description": "Finite-horizon 3-agent point-unicycle diagnostic, no collision dynamics",
        "seed_range": [0, trials - 1],
        "duration_s": duration,
        "dt_s": dt,
        "constants": {
            "speed_mps": float(p["v"]), "ds_m": float(p["ds"]),
            "dl_m": float(p["dl"]), "d0_m": float(p["d_0"]),
            "half_field_of_view_deg": math.degrees(float(p["av"])),
            "inter_gain": float(p["kj"]), "inter_shape": float(p["cj"]),
        },
        "repulsion_gain_at_zero_distance": -math.log(float(p["cj"])),
        "repulsion_near_zero_is_finite": True,
        "original_count_initially_below_ds": sum(
            r["initial_min_m"] < float(p["ds"]) for r in original
        ),
        "original_close_cases_both_forward_visible_immediately_before_ds_entry": len(prior_both_visible),
        "original_close_cases_at_least_one_visible_immediately_before_ds_entry": len(prior_any_visible),
        "variants": variants,
        "warning": (
            "Omnidirectional and no-interaction variants are counterfactual "
            "diagnostic ablations, not proposed changes to the actual controller."
        ),
    }, all_results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=100)
    parser.add_argument("--duration", type=float, default=35.)
    parser.add_argument("--dt", type=float, default=.05)
    parser.add_argument("--seed", type=int, default=23, help="Detailed trace seed")
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/close-approach-diagnosis"))
    args = parser.parse_args()
    if args.trials < 24 or args.trials > 500 or args.duration < 10 or not 0 < args.dt <= .2:
        parser.error("Use 24-500 seeds, >=10 s duration, dt in (0,.2]")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    p = params_from_yaml(Path(__file__).resolve().parents[1] /
                         "src/cyclic_pursuit/config/cyclic.yaml")
    summary, results = analyze(p, args.trials, args.duration, args.dt)
    detailed = simulate(p, args.seed, "original", args.duration, args.dt, trace=True)
    summary["detailed_seed"] = args.seed
    summary["detailed_initial_separation_m"] = detailed["initial_min_m"]
    summary["detailed_first_below_ds"] = detailed["first_below_ds"]
    summary["detailed_closest"] = detailed["closest"]

    (args.output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    columns = ["seed", "variant", "initial_min_m", "min_distance_m",
               "min_time_s", "closest_pair", "crossed_ds"]
    with (args.output_dir / "all_cases.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        for variant in VARIANTS:
            for r in results[variant]:
                writer.writerow({
                    "seed": r["seed"], "variant": variant,
                    "initial_min_m": r["initial_min_m"],
                    "min_distance_m": r["closest"]["distance_m"],
                    "min_time_s": r["closest"]["time_s"],
                    "closest_pair": "-".join(map(str, r["closest"]["pair"])),
                    "crossed_ds": r["below_ds"],
                })
    trace = detailed["trace"]
    if trace:
        with (args.output_dir / "worst_seed_pair_trace.csv").open(
            "w", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=list(trace[0]))
            writer.writeheader()
            writer.writerows(trace)
    pair = "-".join(map(str, detailed["closest"]["pair"]))
    record = [r for r in trace if r["pair"] == pair]

    fig, ax = plt.subplots(figsize=(9, 4.7))
    ax.plot([r["time_s"] for r in record], [r["distance_m"] for r in record])
    ax.axhline(float(p["ds"]), linestyle="--", label="Proximity threshold ds")
    ax.axhline(float(p["d_0"]), linestyle=":", label="Desired neighbor distance d0")
    ax.set(xlabel="Time (s)", ylabel="Separation (m)",
           title=f"Seed {args.seed}, pair {pair}: approaching and passing")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "worst_pair_distance.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4.7))
    ax.plot([r["time_s"] for r in record],
            [r["pair_turn_i_deg_s"] for r in record], label="First agent inter term")
    ax.plot([r["time_s"] for r in record],
            [r["pair_turn_j_deg_s"] for r in record], label="Second agent inter term")
    ax.plot([r["time_s"] for r in record],
            [r["beacon_turn_i_deg_s"] for r in record],
            linestyle="--", label="First agent beacon term")
    ax.plot([r["time_s"] for r in record],
            [r["beacon_turn_j_deg_s"] for r in record],
            linestyle="--", label="Second agent beacon term")
    ax.set(xlabel="Time (s)", ylabel="Angular steering (deg/s)",
           title=f"Seed {args.seed}, pair {pair}: competing control terms")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "worst_pair_control_terms.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.2, 4.6))
    xs = [r["closest"]["distance_m"] for r in results["original"]]
    ax.hist(xs, bins=np.linspace(0, 3.5, 30))
    ax.axvline(float(p["ds"]), linestyle="--", label="Proximity threshold ds")
    ax.set(xlabel="Smallest separation in each run (m)", ylabel="Number of runs",
           title=f"Original controller: {args.trials} initial conditions")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(args.output_dir / "minimum_distance_histogram.png", dpi=150)
    plt.close(fig)

    report = [
        "# Diagnosis: why do the virtual robots approach too closely?",
        "",
        f"Seeded deterministic baseline: {args.trials} scenarios, {args.duration} s each.",
        "All variants use the same initial states and planar unicycle dynamics.",
        "",
        "| Variant | Under 1 m | Under 0.5 m | Under 0.25 m | Smallest distance |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, data in summary["variants"].items():
        report.append(
            f"| {name} | {data['count_below_1m']} | {data['count_below_0p5m']} | "
            f"{data['count_below_0p25m']} | {data['minimum_distance_m']:.3f} m |"
        )
    close_count = summary["variants"]["original"]["count_below_1m"]
    report += [
        "",
        f"In the original controller, {close_count} trials crossed ds=1.0 m.",
        f"Just before the threshold crossing, both agents were visible in "
        f"{summary['original_close_cases_both_forward_visible_immediately_before_ds_entry']} "
        "of these trials. The circular safety sensor always detects within ds",
        "but does not set a hard lower bound on distance.",
        "",
        f"Seed {args.seed}: minimum separation "
        f"{detailed['closest']['distance_m']:.4f} m at "
        f"{detailed['closest']['time_s']:.2f} s; initially "
        f"{detailed['initial_min_m']:.3f} m apart (closest initial pair).",
        "",
        "Mathematical mechanism: g(0;c_v,d0)=-ln(c_v) is finite,",
        "and the neighbor-induced steering term is proportional to the",
        "relative bearing beta, which vanishes for a directly head-on target.",
        "Forward speed is constant; these commands do not brake or enforce a",
        "control-barrier condition at ds.",
        "",
        "Omnidirectional visibility is a diagnostic ablation, not a safe fix.",
        "Eliminating the inter-agent term is another ablation, not a safe fix.",
        "The results are not proof of collision avoidance or stability.",
        "Check full quadrotor and sensing dynamics before physical experiments.",
    ]
    (args.output_dir / "diagnosis.md").write_text(
        "\n".join(report) + "\n", encoding="utf-8"
    )
    print("CLOSE_APPROACH_DIAGNOSIS=" + json.dumps({
        key: summary[key] for key in (
            "variants", "original_count_initially_below_ds",
            "original_close_cases_both_forward_visible_immediately_before_ds_entry",
            "detailed_seed", "detailed_initial_separation_m",
            "detailed_first_below_ds", "detailed_closest"
        )
    }, sort_keys=True), flush=True)


if __name__ == "__main__":
    main()
