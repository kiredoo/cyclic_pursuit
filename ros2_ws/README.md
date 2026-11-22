# ROS 2 / Crazyswarm2 cyclic pursuit prototype

This folder contains the custom `cyclic_pursuit` package consolidated from `ros2_isaac`. The upstream Crazyswarm2 source, `build/`, `install/`, `log/`, and bag recordings are intentionally excluded.

## Layout

- `src/cyclic_pursuit/cyclic_pursuit/controller.py`: multi-agent controller and ROS 2 pose subscriptions
- `src/cyclic_pursuit/cyclic_pursuit/functions/utils.py`: geometry and control helpers
- `src/cyclic_pursuit/cyclic_pursuit/landing.py`: client for landing Trigger services
- `src/cyclic_pursuit/cyclic_pursuit/variable_pose_publisher.py`: synthetic `PoseStamped` messages
- `src/cyclic_pursuit/config/cyclic.yaml`: active drones and experimental parameters
- `src/cyclic_pursuit/launch/launch.py`: controller launch entry point
- `src/cyclic_pursuit/test/test_control_logic.py`: tests of pure mathematical helpers
- `src/cyclic_pursuit/test/test_example_config.py`: checks example active-agent configuration

## Testing without ROS 2

From the repository root:

```bash
python -m pip install numpy PyYAML pytest
PYTHONPATH=ros2_ws/src/cyclic_pursuit python -m pytest -q \
  ros2_ws/src/cyclic_pursuit/test/test_control_logic.py \
  ros2_ws/src/cyclic_pursuit/test/test_example_config.py
```

These tests do not exercise ROS 2 subscriptions, launches, Crazyswarm2, actual motors, or takeoff/landing. A separate CI workflow runs the same focused tests.

## ROS 2 dependencies and build

Follow [Crazyswarm2's documentation](https://github.com/IMRCLab/crazyswarm2) to install a compatible ROS 2 distribution and Crazyswarm2, including `crazyflie_py`. The exact version of the original upstream checkout remains unknown. Ensure the ROS 2 environment is sourced, then from `ros2_ws/`:

```bash
colcon build --symlink-install --packages-select cyclic_pursuit
source install/setup.bash
```

The controller's current parameters are provided to the node named `params` by the `config/cyclic.yaml` file. Active names and IDs must have matching, nonzero lengths equal to `n`. The example currently uses three agents.

## Launch / flight safety

**The controller launches into a takeoff sequence and publishes flight commands. Do not launch it on real hardware until the configuration, coordinate frame, controller signs, altitude behavior, and landing sequence have been validated in a safe simulator and with supervised flight procedures.**

To prevent accidental launch-induced takeoff, the entry point refuses to start unless `CYCLIC_PURSUIT_ENABLE_FLIGHT=1` is explicitly set in the environment. This is an opt-in guard, **not a simulation mode or a substitute for safety validation**. Do not set it merely to bypass an error. After completing validation, the original launch command is:

```bash
ros2 launch cyclic_pursuit launch.py
```

Synthetic pose messages from `ros2 run cyclic_pursuit pose_vir` are useful only for isolated topic testing. They are **not real localization** and are not a safe basis for flight commands.

The landing client remains `ros2 run cyclic_pursuit landing_client` and reads the installed `cyclic.yaml` instead of a machine-specific path.

## Implementation review notes

This review corrects parameter declaration mismatches, active-drone naming, quaternion yaw conversion and shared pose serialization, waiting for all measured poses, configured beacon gains, synthetic pose-message types, and the landing client's configuration path. ROS 2 Humble compilation, package discovery, synthetic ROS topic integration,
and closed-loop kinematic simulations now pass in cloud CI. They do **not**
establish Crazyswarm2 runtime compatibility or safe physical flight.

Outstanding checks include:
- Validate full Crazyswarm2 runtime and higher-fidelity simulation beyond the passing ROS 2 Humble build and hardware-free integration tests.
- Verify the Crazyflie `cmd_vel_hover` message conventions and angular-velocity units/sign for the installed Crazyswarm2 revision.
- Confirm that each `/<cf-name>/pose` is a valid, timely `PoseStamped` with a normalized quaternion and a common coordinate frame.
- Implement an actual fail-safe stop or landing procedure for lost localization; the current controller withholds commands after stale poses but does not issue a physical stop.
- Review the controller's experimental landing rate/altitude logic with hardware experts; the original logic is retained.
- Fix remaining package license placeholders only after confirming the intended license and collaborator approval.

The code is a research prototype, not a ready-to-fly release.


## Cloud-only ROS 2 integration test

GitHub Actions now runs `test/test_ros2_topics.py` in the ROS 2 Humble
container. It creates three real `rclpy` controller nodes and synthetic
`PoseStamped` publishers, then observes their `Twist` commands. It checks
that commands are withheld until all agents report poses, verifies finite
control output, rejects a zero quaternion, and checks that commands stop after
pose updates become stale.

No Crazyswarm2 driver or drone is started. This integration test does **not**
simulate robot dynamics, guarantee formation convergence, or validate a safe
flight controller. It also does not provide a fail-safe motor-stop command if
real localization is lost. Keep the project in non-flight testing until those
issues are addressed.


## Closed-loop lightweight simulation (ROS 2, no physical drones)

The `ros2_ws/sim/closed_loop_ros2.py` script runs the **actual three ROS 2
`Crazyflie` controller nodes** against a separate virtual plant node.
The plant publishes each robot's `PoseStamped` and integrates each received
`Twist` using a planar unicycle model:

```text
x_dot = v cos(theta), y_dot = v sin(theta), theta_dot = omega
```

The controller's historical `angular.z` convention is *negative degrees per
second*. The virtual plant converts `omega = -radians(angular.z)`, preserving
the existing controller sign and units. **This convention is not confirmed
against a physical Crazyswarm2 driver.** `linear.z` is treated as an
altitude-setpoint field and is **not** used in this 2-D model.

The simulation uses the parameter values from `config/cyclic.yaml` and
slightly asymmetric initial positions/headings. Every controller receives
all other agents' poses through the normal shared ROS topic, then changes
its own virtual motion through ROS `Twist` commands.

To reproduce in a ROS 2 Humble environment (with Python NumPy, PyYAML,
Matplotlib and pytest installed), from the repository root:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=ros2_ws/src/cyclic_pursuit
python3 ros2_ws/sim/closed_loop_ros2.py \
  --duration 50 --dt 0.05 --output-dir artifacts/closed-loop
```

Outputs:

- `states.csv`: all robot states and applied unicycle velocities, at every time step.
- `metrics.json`: initial/final mean radius, final radial spread, minimum
  pairwise distance, late drift and command counts.
- `trajectories.png`, `radii.png`, `separation.png`: automatically saved plots.

An earlier baseline workflow run uploaded `closed-loop-simulation` results.
The current cloud workflow uploads seven ROS 2 scenarios as
`closed-loop-robustness`, plus the 100-seed `paper-convergence-study`.

**Limits:** This is a 2-D kinematic model with idealized poses and
zero-order-held commands. It has no physical collision contacts, actuator
limits, rotor aerodynamics, localization noise, radio delay or fail-safe
behavior. A small radial spread is not by itself a proof of safety or
asymptotic convergence. No physical takeoff API is invoked.

## Robustness sweep: initial formations, noisy poses, and delayed observations

Run `python3 ros2_ws/sim/robustness_sweep.py --duration 35` in ROS 2 Humble
with the same dependencies used by the cloud workflow. The sweep runs
**seven independent closed-loop experiments**, retaining the historical
controller and configuration gains unchanged:

1. Baseline asymmetric configuration.
2. Compact configuration, closer to the reference origin.
3. Dispersed configuration, with some neighbors initially outside sensing range.
4. Randomized initial positions and headings (seed 19).
5. Baseline with independent Gaussian position noise (5 cm standard deviation
   in x/y) and yaw noise (2 degrees standard deviation).
6. Baseline with 200 ms delay before each sampled pose reaches its controller.
7. Randomized initial conditions with both measurement noise and 200 ms delay.

Only *observed poses* are noisy. A FIFO delays pose delivery and preserves the
original sample timestamp. The controller currently judges freshness using
arrival time, not the pose timestamp, so this is not a full validation of its
stale-measurement safety logic. Commands themselves are not delayed.

The GitHub Actions artifact `closed-loop-robustness` contains
`summary.csv`, `summary.json`, `summary.md`, and a directory for each
scenario containing `states.csv`, `metrics.json`, `trajectories.png`,
`radii.png`, and `separation.png`.

The summary reports true minimum pairwise separation, final radial spread,
and late mean-radius variation. Thresholds of 0.15 m for radial spread
and 5-second radius change are **diagnostic**, not proofs of stability.
The distance threshold `ds=1.0 m` is flagged if crossed, but there
are no physical collision contacts in the model.

CI fails on an experiment execution failure. It does *not* discard a
numerically valid run with poor geometric results. Such outcomes must be
interpreted rather than silently retuned. ROS 2 scheduling can cause
small run-to-run differences despite seeded noise.

This is a planar kinematic plant, not a rigid-body flight simulator:
rotor forces, actuator bounds, correlated sensing failures, physical
radio networks, collision physics, and fail-safe landing remain untested.

## Paper-aligned initial-condition convergence study

The module `sim/paper_validation.py` implements a compact deterministic
three-agent planar model using the controller's configured equations. It
computes the equilibrium radius from Equation (14) of Ceccarelli et al.,
*Collective circular motion of multi-vehicle systems* (Automatica, 2008),
and verifies numerical/geometrical conditions 13, 33, 35, and 45.

From the repository root in an environment with NumPy, PyYAML, Matplotlib
and the ROS 2 package on PYTHONPATH:

```bash
export PYTHONPATH=ros2_ws/src/cyclic_pursuit
python3 -m pytest -q ros2_ws/sim/test_paper_validation.py
python3 ros2_ws/sim/run_paper_study.py --trials 100 --duration 160 --dt 0.05
```

The 100 independent seeded initial conditions use **three robots**, radial
positions uniformly sampled between 1 and 3 m, heading angles uniformly
sampled in [-pi, pi], and initial angular sectors perturbed by +/-0.5 rad.
This is **not an exact reproduction** of the original paper's four- and
six-agent random-initial-condition studies.

Convergence diagnostics measure each agent's absolute distance from the
theoretical equilibrium radius, plus its heading deviation from the
counterclockwise tangential heading. The operational `first_five_second_dwell_s`
is the first time all three robots remain within 0.1 m radial error and
0.2 rad heading error for **five consecutive simulated seconds**. Passing this
finite-time criterion does not prove asymptotic stability or that errors
will remain small forever. Trials not meeting it within 160 seconds are
censored, not declared divergent.

We separately record minimum pairwise separation (including whether the
original configuration already violates `ds=1.0 m`), closest approach
after five seconds, and duration spent below `ds`. The simulator uses
point robots, so `ds` is a diagnostic design threshold, not a collision
model.

Pure-Python tests compare the compact model's instantaneous control inputs
against the existing `cyclic_pursuit.functions.utils` functions for
several seeded configurations. GitHub Actions also runs a matching 35-second
ROS 2 random-seed-19 trajectory and checks that final mean radius, final
radial spread, and minimum pairwise separation agree within 0.10 m. This
comparison captures selected aggregate metrics, not identical event timing
or full trajectory equality.

The `paper-convergence-study` GitHub Actions artifact contains
`trials.csv`, `summary.json`, `summary.md`,
`convergence_times.png`, and `distance_vs_error.png`.

**Implementation caveat:** the existing controller suppresses its beacon
control term for `rho < 0.1 m`, whereas the paper's corresponding
discontinuity is stated at `rho = 0`. The study deliberately preserves
the implemented ROS 2 behavior instead of silently modifying the control
law. The simulator's command sign/unit conversion must still be verified
against any future Crazyflie driver.


## Why do some agents get closer than 1 m? Diagnostic investigation

Run the **hardware-free** diagnostic ablation script:

```bash
export PYTHONPATH=ros2_ws/src/cyclic_pursuit
python3 -m pytest -q ros2_ws/sim/test_close_approaches.py
python3 ros2_ws/sim/diagnose_close_approaches.py \
  --trials 100 --duration 35 --dt 0.05 --seed 23 \
  --output-dir artifacts/close-approach-diagnosis
```

The `close-approach-diagnosis` artifact from the separate fast CI workflow
contains `summary.json`, `diagnosis.md`, `all_cases.csv`, the detailed
seed-23 pair trace, and distance/control-component plots.

This script uses the unchanged configured gains and **three distinct
counterfactuals** over the same 100 seeded initial conditions:
- Original limited forward visibility and omnidirectional 1 m proximity region.
- Full 360-degree visibility to the same maximum distance of 4 m, with the
  exact same steering law. This tests the role of late detection.
- Beacon-only steering, ignoring neighbors. This isolates what the
  inter-agent component changes.

Neither counterfactual should be considered a safe proposed controller.
They can change equilibrium behavior and lack physical safety guarantees.

The paper's inter-agent term is `kv * g(distance; cv,d0) * beta`, where
`g(distance)` is negative when `distance < d0`. For `cv = 2`,
`g(0) = -log(2)`, a finite value. Crucially, the steering term tends to
zero as the neighbor bearing `beta` tends to zero, even when two robots
are on a head-on trajectory. Both robots maintain constant forward speed.
The `ds` proximity region ensures **detection**, not guaranteed minimum
separation or a physical safety barrier.

The diagnostic reproduces seed 23: a near-head-on approach with both robots
within forward field of view long before the 1 m threshold is crossed.
Plots distinguish actual repulsive steering from the beacon steering term.
No Crazyswarm2, physical drones, or flight systems are started.

This remains a planar point-robot model without saturation, finite vehicle
size, collision response, or communication failures. It is not safe to
infer drone flight safety from these plots.


## Consolidated results and interpretation

For the observed 100-seed convergence results (86% meeting a five-second
radius-and-heading dwell), the 58% sub-1 m proximity rate, detailed visibility
analysis, counterfactual experiments, reproducible run links and explicit
limitations, see the
[simulation validation and close-approach report](../docs/simulation-validation-results.md).

**Research decision:** Keep the historical controller and all gains unchanged.
Document the successful circular-formation behavior **and** the limitations;
do not describe the algorithm as guaranteeing collision avoidance or as ready
for physical flight.
