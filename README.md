# Crazyflie Cyclic Pursuit

Research code for multi-agent cyclic pursuit with **Crazyflie 2.1** quadcopters. The project explores circular formations and motion relative to a moving reference point, from Python simulation to a ROS 2 / Crazyswarm2 controller prototype.

![Moving-centroid simulation](media/moving_centroid.gif)

## Project components

| Component | What it contains |
| --- | --- |
| [Simulation](simulation/) | Jupyter notebooks for the cyclic pursuit controller, parameter tuning, and trajectory visualizations |
| [ROS 2 controller](ros2_ws/) | Custom `cyclic_pursuit` ROS 2 package migrated from the historical `ros2_isaac` branch |
| [Analysis](analysis/) | MATLAB script for trajectory, heading, and inter-agent distance analysis |
| [Documentation](docs/wsl2-tutorial.md) | Crazyradio USB passthrough to WSL2 |

## Python simulation

1. Install Python 3 and the notebook dependencies:
   ```bash
   python -m pip install jupyterlab numpy scipy matplotlib pandas tqdm pillow
   ```
2. From the repository root, open Jupyter:
   ```bash
   python -m jupyterlab
   ```
3. Run [`simulation/cyclic.ipynb`](simulation/cyclic.ipynb) top to bottom. See [`simulation/tuning_params.ipynb`](simulation/tuning_params.ipynb) for example parameter sets.

The notebook demonstrates agent trajectories, convergence and pairwise-distance plots, and an optional animation generator. An example GIF is in [`media/moving_centroid.gif`](media/moving_centroid.gif).

**Note:** In the documentation branch's default five-agent example, the constraint checker reported an unmet first geometric constraint even though the numerical solver completed. Review parameter validity before treating any trajectory as a validated safe formation.

## ROS 2 controller

The [ROS 2 workspace instructions](ros2_ws/README.md) describe the source package, external Crazyswarm2 dependency, and build/launch entry points. This is **historical research code brought into one place for review**, not a verified ready-to-fly release. Inspect the configuration, parameter handling, landing logic, and hardware safety assumptions before executing on real drones.

## Reproducible simulation results

The controller was evaluated with a ROS 2 closed-loop virtual swarm and a
100-seed, three-agent numerical convergence study. The experiments use the
existing control law and unchanged gains.

| Observed result | Three-agent simulation |
| --- | ---: |
| Met radial and tangent-heading tolerances for five continuous seconds | **86 / 100** |
| Still within those tolerances at 160 s | **83 / 100** |
| Approached closer than the configured 1 m threshold | **58 / 100** |
| Smallest simulated separation | **0.0136 m** |

The interaction term improves severe close-approach outcomes compared with
removing it, but it **does not guarantee separation**. These are planar,
hardware-free simulations, not a global stability proof or safe-flight claim.

Read the full [simulation validation and close-approach report](docs/simulation-validation-results.md)
for methodology, ablation comparisons, plots and reproducible CI evidence.

## Physical Crazyflie flight experiments (2024–2025)

During my MURO Lab research at UC San Diego, I deployed the cyclic pursuit
controller on **physical Crazyflie drones**, using ROS 2, Crazyswarm2, and
OptiTrack motion capture. In my May 2025 final report, I documented
**successful two- and three-drone circular-pursuit experiments** and an
**unsuccessful four-drone experiment**. Those hardware results are distinct
from the later hardware-free simulation study.

[![Three Crazyflies: measured trajectory from recorded poses](docs/figures/real-three-crazyflies.svg)](docs/real-flight-results.md)

| Historical test | Outcome reported | Approximate minimum 3-D separation while all agents had z > 0.35 m |
| --- | --- | ---: |
| Two Crazyflies | Successful circular pursuit | 0.164 m |
| Three Crazyflies | Successful circular pursuit | 0.586 m |
| Four Crazyflies | Unsuccessful, including a reported mid-flight flip | 0.108 m |

The distances are estimates from recorded poses interpolated to a 20 Hz
timeline, **not guarantees of physical clearance**. In my final report,
I used a practical 0.5 m minimum-distance guideline, so even the successful
two-drone run warrants caution. I did not establish the root cause of the
four-drone failure.

**[See the complete physical flight-test results, all three trajectory plots,
and measurement methodology](docs/real-flight-results.md).**

I have kept my unredacted final report and the raw rosbags out of the
public repository pending a privacy and sharing review. The GitHub
figures are simplified vector renderings derived from my recorded
flight trajectories.

## MATLAB analysis

[`analysis/cyclic_pursuit_result_analysis.m`](analysis/cyclic_pursuit_result_analysis.m) contains trajectory, heading, and pairwise-distance plotting. It refers to historical rosbag file names. Large recorded `.db3` files were intentionally not copied into this branch.

## Hardware and WSL2

See the [Crazyradio USB / WSL2 guide](docs/wsl2-tutorial.md). No internal Wi-Fi passwords or hardware credentials are included in this consolidated documentation.

## Maintenance and reproducibility

The three GitHub Actions workflows run on pull requests targeting `main`
and on pushes to `main`. They can also be started manually from the
**Actions** tab. The checks validate the mathematical controller, ROS 2
Humble build and topic integration, and synthetic closed-loop experiments;
they do **not** certify flight safety.

Generated simulation data is written under `artifacts/` and excluded from
Git. CI uploads reports and plots as run artifacts. The original ROS 2
controller code and historical experimental source remain in place.

See [repository maintenance notes](docs/repository-maintenance.md) for the
branch-archival plan, documentation layout and unresolved publishing decisions.
The package version remains `0.0.0`; the **license is not yet selected**,
so don't assume that the entire repository can be freely redistributed
or incorporated into other projects.

## My contributions and collaborators

I developed this project at **UC San Diego's MURO Lab from September 2024 to
May 2025**. I built on an initial beacon-controller script and basic simulator
developed by **Sudhanshu Shankar**. I implemented the missing multi-agent
interaction logic, packaged the controller as a ROS 2 application, developed
simulation and MATLAB data-analysis tools, tuned parameters, and conducted
physical Crazyflie experiments.

**Scott Addams and Arnau Marzabal** helped me with Crazyflie hardware and
OptiTrack integration. I credit their contributions and Sudhanshu's earlier
work while distinguishing them from my own controller, software-integration,
and experimental work.

I have preserved the research notebooks and ROS 2/MATLAB code from earlier
repository branches. The controller remains an experimental research
prototype, not a verified flight-ready release. Crazyswarm2 is an upstream
dependency with its own licensing.

## Reference

[Collective circular motion of multi-vehicle systems](https://www.sciencedirect.com/science/article/pii/S0005109808002951).
