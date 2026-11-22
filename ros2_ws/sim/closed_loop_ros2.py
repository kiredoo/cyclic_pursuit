#!/usr/bin/env python3
"""Run the actual ROS 2 cyclic-pursuit nodes against three virtual unicycle agents.

The virtual plant is a ROS 2 Node that consumes /<agent>/cmd_vel_hover Twist
and publishes /<agent>/pose PoseStamped. It never imports or launches a drone
driver, radio service, or Crazyswarm2 takeoff sequence.

All three controller nodes are instantiated from the existing
cyclic_pursuit.controller.Crazyflie class. This is closed-loop dynamics, not
firmware or rigid-body flight simulation.
"""

import argparse
import csv
import json
import math
import time
from collections import deque
from pathlib import Path
from itertools import combinations

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from cyclic_pursuit.controller import Crazyflie
from cyclic_pursuit.functions.utils import validate_swarm_configuration
from kinematics import command_to_unicycle, integrate_unicycle


# Deliberately non-identical radii and headings so the controller must respond.
INITIAL = (
    (1.70, 0.00, 1.80),
    (-0.70, 1.60, 3.70),
    (-0.70, -1.50, -0.40),
)


def initial_states_for(scenario, seed):
    """Select reproducible initial-condition experiments."""
    if scenario == "baseline":
        return INITIAL
    if scenario == "compact":
        return ((0.80, 0.00, 1.80), (-0.45, 0.70, 3.70), (-0.40, -0.65, -0.40))
    if scenario == "dispersed":
        return ((3.00, 0.00, 1.80), (-1.60, 2.60, 3.70), (-1.50, -2.80, -0.40))
    if scenario == "random":
        rng = np.random.default_rng(seed)
        angles = np.arange(3) * (2 * math.pi / 3) + rng.uniform(-0.5, 0.5, size=3)
        radii = rng.uniform(1.0, 3.0, size=3)
        headings = rng.uniform(-math.pi, math.pi, size=3)
        return tuple(
            (float(r * math.cos(a)), float(r * math.sin(a)), float(h))
            for r, a, h in zip(radii, angles, headings)
        )
    raise ValueError(f"Unknown initial condition: {scenario}")


def configuration():
    """Use the very same parameter values as the ROS 2 experiment."""
    filename = Path(__file__).resolve().parents[1] / "src/cyclic_pursuit/config/cyclic.yaml"
    params = yaml.safe_load(filename.read_text(encoding="utf-8"))["params"]["ros__parameters"]
    names = tuple(params["cfs_active_names"])
    ids = tuple(params["cfs_active_id"])
    validate_swarm_configuration(names, ids, int(params["n"]))
    if len(names) != len(INITIAL):
        raise ValueError("The initial-state fixture currently supports exactly three agents")
    return params, names, ids


class VirtualSwarm(Node):
    """A ROS-publishing kinematic plant with zero-order-held control inputs."""

    def __init__(self, names, dt, initial_states, position_noise_std, yaw_noise_deg,
                 observation_delay_s, seed):
        super().__init__("closed_loop_virtual_swarm")
        self.names = names
        self.dt = dt
        self.initial_states = tuple(tuple(p) for p in initial_states)
        self.states = {name: tuple(self.initial_states[i]) for i, name in enumerate(names)}
        self.position_noise_std = float(position_noise_std)
        self.yaw_noise_rad = math.radians(float(yaw_noise_deg))
        self.observation_delay_s = float(observation_delay_s)
        self.seed = int(seed)
        self.rng = np.random.default_rng(seed)
        self.delayed_pose_messages = deque()
        self.pose_publishers = {
            name: self.create_publisher(PoseStamped, f"/{name}/pose", 10)
            for name in names
        }
        self.latest = {name: None for name in names}
        self.command_at = {name: None for name in names}
        self.commands_received = {name: 0 for name in names}
        self.max_command_deg_s = {name: 0.0 for name in names}
        self.command_subscriptions = [
            self.create_subscription(
                Twist, f"/{name}/cmd_vel_hover",
                lambda msg, label=name: self.command_callback(label, msg), 10,
            )
            for name in names
        ]
        self.simulated_t = 0.0
        self.steps = 0
        self.rows = []
        self.active = False
        self.last_step = time.monotonic()
        self.timer = self.create_timer(dt, self.step)

    def command_callback(self, name, msg):
        self.latest[name] = (float(msg.linear.x), float(msg.angular.z))
        self.command_at[name] = time.monotonic()
        self.commands_received[name] += 1
        if math.isfinite(float(msg.angular.z)):
            self.max_command_deg_s[name] = max(
                self.max_command_deg_s[name], abs(float(msg.angular.z))
            )

    def publish_poses(self):
        """Delay and perturb observed poses, never the true virtual states."""
        now = time.monotonic()
        sample_stamp = self.get_clock().now().to_msg()
        for name in self.names:
            x, y, theta = self.states[name]
            x_obs = x + self.rng.normal(0.0, self.position_noise_std)
            y_obs = y + self.rng.normal(0.0, self.position_noise_std)
            theta_obs = theta + self.rng.normal(0.0, self.yaw_noise_rad)
            msg = PoseStamped()
            msg.header.stamp = sample_stamp
            msg.header.frame_id = "world"
            msg.pose.position.x = float(x_obs)
            msg.pose.position.y = float(y_obs)
            msg.pose.orientation.z = math.sin(theta_obs / 2.0)
            msg.pose.orientation.w = math.cos(theta_obs / 2.0)
            self.delayed_pose_messages.append(
                (now + self.observation_delay_s, name, msg)
            )
        while self.delayed_pose_messages and self.delayed_pose_messages[0][0] <= now:
            _, name, measurement = self.delayed_pose_messages.popleft()
            self.pose_publishers[name].publish(measurement)

    def begin(self):
        self.rows.clear()
        self.delayed_pose_messages.clear()
        self.steps = 0
        self.simulated_t = 0.0
        self.active = True
        for name in self.names:
            self.commands_received[name] = 0
        self.record({name: (0.0, 0.0) for name in self.names})

    def record(self, applied):
        for name in self.names:
            x, y, theta = self.states[name]
            speed, omega = applied[name]
            self.rows.append((
                round(self.simulated_t, 6), name, x, y, theta, speed, omega,
            ))

    def step(self):
        # During DDS discovery publish initial poses but do not move robots.
        if not self.active:
            self.publish_poses()
            return

        now = time.monotonic()
        applied = {}
        for name in self.names:
            msg = self.latest[name]
            recent = (
                msg is not None
                and self.command_at[name] is not None
                and now - self.command_at[name] < 0.35
            )
            if recent:
                try:
                    applied[name] = command_to_unicycle(*msg)
                except ValueError:
                    applied[name] = (0.0, 0.0)
            else:
                # The virtual plant stops when no fresh command is available.
                # This is a simulator behavior, NOT a safety stop implemented by
                # the physical controller.
                applied[name] = (0.0, 0.0)
        for name in self.names:
            v, omega = applied[name]
            self.states[name] = integrate_unicycle(
                self.states[name], v, omega, self.dt,
            )
        self.steps += 1
        self.simulated_t = self.steps * self.dt
        self.record(applied)
        self.publish_poses()


def export_results(world, output_dir, requested_duration):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = world.rows
    with (output_dir / "states.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["time_s", "agent", "x_m", "y_m", "yaw_rad", "v_mps", "omega_radps"])
        writer.writerows(rows)

    # Every recorded tick includes exactly one state per agent in names order.
    t = np.asarray([row[0] for row in rows[::len(world.names)]])
    states = np.asarray([[[row[2], row[3], row[4]]
                         for row in rows[i:i + len(world.names)]]
                         for i in range(0, len(rows), len(world.names))], dtype=float)
    if len(t) < 30 or not np.all(np.isfinite(states)):
        raise AssertionError("The simulation did not produce sufficient finite states")

    positions = states[:, :, :2]
    radius = np.linalg.norm(positions, axis=2)
    pairs = list(combinations(range(len(world.names)), 2))
    pairwise = np.column_stack([
        np.linalg.norm(positions[:, i] - positions[:, j], axis=1)
        for i, j in pairs
    ])
    mean_radius = radius.mean(axis=1)
    final_window = max(2, int(5.0 / world.dt))
    final_window = min(final_window, len(t))
    start_mean = float(mean_radius[0])
    final_mean = float(mean_radius[-1])
    final_spread = float(np.std(radius[-1]))
    late_change = float(abs(mean_radius[-1] - mean_radius[-final_window]))
    metrics = {
        "model": "planar unicycle, zero-order-held commanded forward speed and yaw rate",
        "command_convention": "historical controller angular.z=-yaw_rate_deg_per_s",
        "using_real_ros2_controller_nodes": True,
        "using_crazyswarm2_or_physical_drones": False,
        "requested_duration_s": requested_duration,
        "simulated_duration_s": float(t[-1]),
        "integration_dt_s": world.dt,
        "scenario": world.scenario,
        "random_seed": world.seed,
        "position_noise_std_m": world.position_noise_std,
        "yaw_noise_std_deg": math.degrees(world.yaw_noise_rad),
        "observation_delay_ms": world.observation_delay_s * 1000.0,
        "initial_states_x_y_yaw": [list(p) for p in world.initial_states],
        "metrics_are_computed_from_ground_truth": True,
        "separation_below_ds_1m": bool(pairwise.min() < 1.0),
        "agents": list(world.names),
        "command_counts": world.commands_received,
        "max_abs_yaw_command_deg_s": world.max_command_deg_s,
        "initial_mean_radius_m": start_mean,
        "final_mean_radius_m": final_mean,
        "final_radius_std_m": final_spread,
        "last_five_seconds_mean_radius_change_m": late_change,
        "minimum_pairwise_separation_m": float(pairwise.min()),
        "final_pairwise_separations_m": {
            f"{world.names[i]}-{world.names[j]}": float(pairwise[-1, n])
            for n, (i, j) in enumerate(pairs)
        },
        "radial_agreement_within_0p15_m": final_spread < 0.15,
        "late_radius_change_within_0p15_m": late_change < 0.15,
        "note": "Geometric diagnostics are not a proof of convergence or physical-flight safety.",
    }
    (output_dir / "metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8",
    )

    fig, ax = plt.subplots(figsize=(7, 7))
    for i, name in enumerate(world.names):
        line, = ax.plot(positions[:, i, 0], positions[:, i, 1], label=name)
        ax.scatter(positions[0, i, 0], positions[0, i, 1],
                   marker="o", facecolors="none", edgecolors=line.get_color(), s=55)
        ax.scatter(positions[-1, i, 0], positions[-1, i, 1],
                   marker="x", color=line.get_color(), s=55)
    ax.scatter(0, 0, marker="*", s=110, label="Reference origin")
    ax.set(xlabel="x (m)", ylabel="y (m)", title="Closed-loop virtual robot trajectories")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "trajectories.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for i, name in enumerate(world.names):
        ax.plot(t, radius[:, i], label=name)
    ax.set(xlabel="Simulation time (s)", ylabel="Distance from origin (m)",
           title="Agent radii")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "radii.png", dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    for n, (i, j) in enumerate(pairs):
        ax.plot(t, pairwise[:, n], label=f"{world.names[i]} to {world.names[j]}")
    ax.set(xlabel="Simulation time (s)", ylabel="Pairwise distance (m)",
           title="Inter-agent separation")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "separation.png", dpi=150)
    plt.close(fig)

    if any(count < 10 for count in world.commands_received.values()):
        raise AssertionError(f"Not all controllers generated commands: {world.commands_received}")
    if t[-1] < requested_duration - world.dt * 1.1:
        raise AssertionError(f"Simulation ended early at {t[-1]:.2f} seconds")
    if not np.all(np.isfinite(pairwise)):
        raise AssertionError("Non-finite inter-agent separation")
    print("CLOSED_LOOP_METRICS=" + json.dumps(metrics, sort_keys=True), flush=True)
    return metrics


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=50.0)
    parser.add_argument("--dt", type=float, default=0.05)
    parser.add_argument("--output-dir", default="artifacts/closed-loop")
    parser.add_argument("--scenario", choices=("baseline", "compact", "dispersed", "random"),
                        default="baseline")
    parser.add_argument("--position-noise-std", type=float, default=0.0)
    parser.add_argument("--yaw-noise-deg", type=float, default=0.0)
    parser.add_argument("--observation-delay-ms", type=float, default=0.0)
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()
    if args.duration <= 0 or not (0.01 <= args.dt <= 0.2):
        parser.error("Duration must be positive and dt must be 0.01 to 0.2 seconds")
    for label, value in (
        ("position-noise-std", args.position_noise_std),
        ("yaw-noise-deg", args.yaw_noise_deg),
        ("observation-delay-ms", args.observation_delay_ms),
    ):
        if not math.isfinite(value) or value < 0:
            parser.error(f"{label} must be finite and nonnegative")
    if args.observation_delay_ms > 1000:
        parser.error("Observation delay must not exceed 1000 ms in this experiment")

    params, names, ids = configuration()
    rclpy.init()
    executor = SingleThreadedExecutor()
    controllers = []
    world = None
    try:
        world = VirtualSwarm(
            names, args.dt, initial_states_for(args.scenario, args.seed),
            args.position_noise_std, args.yaw_noise_deg,
            args.observation_delay_ms / 1000.0, args.seed,
        )
        world.scenario = args.scenario
        for name, ident in zip(names, ids):
            controllers.append(Crazyflie(
                name, ident, names,
                params["v"], params["av"], params["ds"], params["dl"],
                params["psi"], params["d_0"], params["c"], params["cj"],
                params["k"], params["kj"], params["rho_o"], params["n"],
                params["height"],
            ))
        for node in controllers:
            executor.add_node(node)
        executor.add_node(world)
        # Allow DDS discovery/pose handshakes at fixed, unmoving positions.
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.03)
        world.begin()
        deadline = time.monotonic() + max(90.0, args.duration * 3.0)
        while world.simulated_t < args.duration:
            if time.monotonic() > deadline:
                raise TimeoutError("ROS 2 executor did not advance simulation")
            executor.spin_once(timeout_sec=0.025)
        export_results(world, args.output_dir, args.duration)
    finally:
        if world is not None:
            executor.remove_node(world)
            world.destroy_node()
        for node in controllers:
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
