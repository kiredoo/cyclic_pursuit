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

This review corrects parameter declaration mismatches, active-drone naming, quaternion yaw conversion and shared pose serialization, waiting for all measured poses, configured beacon gains, synthetic pose-message types, and the landing client's configuration path. It does **not** establish that hardware behavior is safe or that full ROS 2 integration succeeds.

Outstanding checks include:
- A real `colcon build` and ROS 2 integration run in a compatible environment.
- Verify the Crazyflie `cmd_vel_hover` message conventions and angular-velocity units/sign for the installed Crazyswarm2 revision.
- Confirm that each `/<cf-name>/pose` is a valid, timely `PoseStamped` with a normalized quaternion and a common coordinate frame.
- Define stale-pose timeouts and fail-safe behavior before real flight, since the present controller only waits for initial poses.
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

The CI workflow runs the same experiment and uploads all outputs as a
`closed-loop-simulation` GitHub Actions artifact.

**Limits:** This is a 2-D kinematic model with idealized poses and
zero-order-held commands. It has no physical collision contacts, actuator
limits, rotor aerodynamics, localization noise, radio delay or fail-safe
behavior. A small radial spread is not by itself a proof of safety or
asymptotic convergence. No physical takeoff API is invoked.
