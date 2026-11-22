# ROS 2 / Crazyswarm2 cyclic pursuit prototype

This directory contains the **custom** `cyclic_pursuit` Python package from the historical `ros2_isaac` branch. It does **not** vendor the upstream Crazyswarm2 repository, generated `build/`, `install/`, `log/` directories, or rosbag recordings.

## Layout

- `src/cyclic_pursuit/cyclic_pursuit/controller.py`: ROS 2 controller node
- `src/cyclic_pursuit/cyclic_pursuit/functions/utils.py`: relative-geometry and control helpers
- `src/cyclic_pursuit/launch/launch.py`: launch entry point
- `src/cyclic_pursuit/config/cyclic.yaml`: historical experimental parameters
- `src/cyclic_pursuit/cyclic_pursuit/landing.py`: landing-service client prototype
- `src/cyclic_pursuit/cyclic_pursuit/variable_pose_publisher.py`: synthetic pose publisher

## Dependencies and build

Set up a compatible ROS 2 distribution and follow the official [Crazyswarm2 installation instructions](https://github.com/IMRCLab/crazyswarm2) to install its ROS 2 drivers and `crazyflie_py` dependency. The original branch included an upstream checkout, but its exact compatible revision is not yet recorded here.

After satisfying ROS 2 and Crazyswarm2 dependencies, from `ros2_ws/`:

```bash
colcon build --symlink-install --packages-select cyclic_pursuit
source install/setup.bash
```

The existing launch entry point is:

```bash
ros2 launch cyclic_pursuit launch.py
```

**Do not run this on real hardware without reviewing the implementation and validating it safely.** The historical source has not been built or integration-tested as part of this consolidation. In particular, review the parameter declarations/names, the controller's references to group names, hard-coded paths in the landing helper, pose-message expectations, and takeoff/landing behavior.

This directory is included so that robotics work can be reviewed alongside its simulation and analysis. Functional fixes should be made in a separate, testable pull request.

## Attribution

The code is copied from the repository's `ros2_isaac` branch without deliberately changing its controller algorithms. Crazyswarm2 is an external project and has its own license.
