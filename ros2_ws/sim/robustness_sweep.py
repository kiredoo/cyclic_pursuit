#!/usr/bin/env python3
"""Run seven independent hardware-free ROS 2 cyclic-pursuit stress cases.

Execute the unchanged controller with several initial conditions, seeded
measurement noise, and delayed pose observations. Save every scenario, including
ones with disappointing formation results. Do not optimize gains in this sweep.
"""

import argparse
import csv
import json
import subprocess
import sys
from pathlib import Path


SCENARIOS = [
    ("baseline", "baseline", 0.0, 0.0, 0.0, 7),
    ("compact", "compact", 0.0, 0.0, 0.0, 7),
    ("dispersed", "dispersed", 0.0, 0.0, 0.0, 7),
    ("random_seed_19", "random", 0.0, 0.0, 0.0, 19),
    ("noisy_5cm_2deg", "baseline", 0.05, 2.0, 0.0, 7),
    ("delayed_200ms", "baseline", 0.0, 0.0, 200.0, 7),
    ("combined_random_noise_delay", "random", 0.05, 2.0, 200.0, 19),
]
COLUMNS = [
    "scenario_name", "initial_condition", "status", "seed",
    "position_noise_std_m", "yaw_noise_std_deg", "observation_delay_ms",
    "simulated_duration_s", "final_mean_radius_m", "final_radius_std_m",
    "last_five_seconds_mean_radius_change_m", "minimum_pairwise_separation_m",
    "separation_below_ds_1m", "radial_agreement_within_0p15_m",
    "late_radius_change_within_0p15_m", "error",
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=35.0)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/robustness"))
    args = parser.parse_args()
    if args.duration < 10 or args.duration > 120:
        parser.error("Duration must be 10 to 120 simulated seconds")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    experiment = Path(__file__).resolve().parent / "closed_loop_ros2.py"

    table = []
    failures = 0
    for name, initial, sigma_xy, sigma_yaw, delay_ms, seed in SCENARIOS:
        folder = args.output_dir / name
        command = [
            sys.executable, str(experiment),
            "--duration", str(args.duration), "--dt", "0.05",
            "--scenario", initial,
            "--position-noise-std", str(sigma_xy),
            "--yaw-noise-deg", str(sigma_yaw),
            "--observation-delay-ms", str(delay_ms),
            "--seed", str(seed), "--output-dir", str(folder),
        ]
        print("ROBUSTNESS_START=" + name, flush=True)
        record = {
            "scenario_name": name,
            "initial_condition": initial,
            "status": "failed",
            "seed": seed,
            "position_noise_std_m": sigma_xy,
            "yaw_noise_std_deg": sigma_yaw,
            "observation_delay_ms": delay_ms,
            "error": "",
        }
        try:
            result = subprocess.run(command, capture_output=True, text=True, timeout=110)
            if result.returncode:
                raise RuntimeError(
                    "exit " + str(result.returncode) + ": "
                    + (result.stderr + result.stdout)[-4000:]
                )
            metrics = json.loads((folder / "metrics.json").read_text(encoding="utf-8"))
            record.update({key: metrics[key] for key in COLUMNS if key in metrics})
            record["status"] = "completed"
            print("ROBUSTNESS_RESULT=" + json.dumps(
                {key: record.get(key) for key in COLUMNS if key != "error"},
                sort_keys=True,
            ), flush=True)
        except Exception as exc:
            failures += 1
            record["error"] = str(exc).replace("\n", " ")[:1500]
            print("ROBUSTNESS_ERROR=" + name + ": " + record["error"], flush=True)
        table.append(record)

    with (args.output_dir / "summary.csv").open(
        "w", newline="", encoding="utf-8"
    ) as file:
        writer = csv.DictWriter(file, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows([{key: row.get(key, "") for key in COLUMNS} for row in table])
    (args.output_dir / "summary.json").write_text(
        json.dumps(table, indent=2) + "\n", encoding="utf-8"
    )

    lines = [
        "# Closed-loop robustness sweep", "",
        "Diagnostic geometric thresholds are not proofs of convergence or flight safety.",
        "",
        "| Scenario | Execution | Final radial SD (m) | Late radius change (m) | Minimum separation (m) |",
        "| --- | --- | ---: | ---: | ---: |",
    ]
    for row in table:
        def fmt(key):
            v = row.get(key)
            return f"{v:.3f}" if isinstance(v, (int, float)) else "n/a"
        lines.append(
            f"| {row['scenario_name']} | {row['status']} | "
            f"{fmt('final_radius_std_m')} | "
            f"{fmt('last_five_seconds_mean_radius_change_m')} | "
            f"{fmt('minimum_pairwise_separation_m')} |"
        )
    lines.extend([
        "",
        "The 200 ms delay is applied to sensor pose delivery, not commands.",
        "All metrics are computed from true simulated plant states.",
        "The virtual plant halts when commands stop. This does not establish",
        "a physical hardware fail-safe.",
    ])
    (args.output_dir / "summary.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    print(
        f"ROBUSTNESS_FINISHED={len(table)} cases, {failures} execution failures",
        flush=True,
    )
    return int(failures != 0)


if __name__ == "__main__":
    sys.exit(main())
