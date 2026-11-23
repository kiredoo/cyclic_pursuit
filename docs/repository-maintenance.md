# Repository maintenance and archival notes

This note documents decisions associated with the November 2026 cleanup.
It is **not** a flight-readiness checklist and does not certify any
algorithmic safety property.

## Maintained project layout

| Folder | Purpose |
| --- | --- |
| `ros2_ws/src/cyclic_pursuit/` | ROS 2 controller source, configuration, launch and package tests |
| `ros2_ws/sim/` | Hardware-free closed-loop dynamics and numerical convergence/robustness studies |
| `simulation/` | Historical Jupyter research notebooks |
| `analysis/` | Historical MATLAB rosbag analysis |
| `docs/` | Experimental and numerical results and WSL2 instructions |
| `media/` | Historical demonstration animation |
| `.github/workflows/` | Python tests, ROS 2 integration and diagnostic automation |

No directory moves or controller changes are required for this cleanup.

## Continuous integration

Workflows run on `push` to `main`, `pull_request` targeting `main`,
and manual `workflow_dispatch`. Run results are available in the
GitHub **Actions** tab. The longer ROS 2 job contains an actual
ROS 2 Humble build, three-agent synthetic topic integration, seven
kinematic robustness scenarios, and 100 seeded numerical convergence trials.

A green result means tests executed according to their specified
scope. It does **not** establish Crazyswarm2 hardware compatibility,
collision avoidance, or flight safety.

Local generated data in `artifacts/` is ignored by Git; GitHub
Actions publishes an artifact for reproducibility. The raw experimental
`.db3` rosbags and unredacted final report are intentionally not
committed to this public repo.

## Branch retention

`main` is the maintained default branch. The following are candidates
for removal **only after a separate explicit review**:

- `chore/consolidate-cyclic-pursuit` (merged PR #6)
- `docs/real-flight-experiments` (merged PR #7)
- `docs/real-crazyflie-flight-results` (unused documentation branch)
- `docs/simulation-and-wsl2` (historical documentation consolidation)

The following branches contain research history or changes not fully
represented by a simple branch merge and should **not** be casually deleted:

- `analysis`
- `old_ros_files`
- `ros2_isaac`

Before deleting these, inspect unique commits and data provenance; consider
archival tags. Historic research work should not be rewritten or silently
removed. Stale PR #5 targets the legacy `analysis` branch and must not be
merged into that branch as part of normal `main` maintenance.

## Unresolved package metadata

The package and setup configuration still state version `0.0.0`.
No release or compatibility guarantee has been asserted.

The package manifest and Python setup metadata still contain a placeholder
for the license. **No license is granted or inferred by this cleanup.**
Confirm the intended license, source ownership and collaborator permissions
before choosing one, adding a root `LICENSE` file, or distributing a
release.

Crazyswarm2 is an upstream dependency, not a component bundled in this
repository. The legacy `crazyflie_py` dependency and drone command
conventions require validation against the particular hardware stack
before physical operation.
