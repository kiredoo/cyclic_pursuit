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
