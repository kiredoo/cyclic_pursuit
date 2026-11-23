# Cyclic Pursuit: Simulation Validation and Close-Approach Study

**Status:** Reproducible, simulation-only results, November 22, 2026.  
**Code:** [ROS 2 controller and simulated plant](../ros2_ws/README.md) · [Numerical convergence study](../ros2_ws/sim/run_paper_study.py) · [Close-approach diagnostics](../ros2_ws/sim/diagnose_close_approaches.py)  
**Reference:** Ceccarelli, Di Marco, Garulli, and Giannitrapani, *Collective circular motion of multi-vehicle systems*, Automatica (2008), [publisher record](https://www.sciencedirect.com/science/article/pii/S0005109808002951).

## Summary

I kept the cyclic-pursuit controller **as implemented**, without retuning
its gains or changing its visibility, repulsion, or forward-speed policy.
I then evaluated it with planar unicycle simulations. The results show
that the robots often approach the paper's predicted circular formation,
but **minimum separation is not guaranteed** for the tested distribution
of initial conditions.

In 100 deterministic three-agent simulations lasting 160 simulated seconds:

- **86/100** achieved at least one five-second interval meeting *both* radial
  and heading-error tolerances.
- **83/100** were within both tolerances at the final simulated instant.
- **58/100** crossed below the configured **1.0 m** proximity distance during
  motion, although **none** started below that threshold.
- The smallest simulated inter-agent distance was **0.01364 m**, with no
  physical collision modeled.

These observations are **not** a proof of global convergence, collision
avoidance, or safe quadrotor operation.

## Experiment setup and measurements

| Setting | Value |
| --- | --- |
| Agents | 3, virtual planar point-unicycle robots |
| Robot dynamics | `x_dot = v cos(theta)`, `y_dot = v sin(theta)`, `theta_dot = omega` |
| Forward speed | Fixed `v = 0.3 m/s` |
| Numerical integrator | Midpoint heading update, `dt = 0.05 s` |
| Controller gains | Exactly as in [cyclic.yaml](../ros2_ws/src/cyclic_pursuit/config/cyclic.yaml), unchanged |
| Initial-condition study | 100 deterministic random seeds, `0...99`, 160 s per seed |
| Initial radii | Uniform from 1 to 3 m |
| Initial headings | Uniform from -π to π |
| Initial polar angles | Three equally spaced sectors, each perturbed by uniform ±0.5 rad |
| Predicted equilibrium radius | **2.003306827 m**, calculated from paper Equation (14) |
| Radial criterion | All agents within 0.1 m of that radius |
| Tangent-heading criterion | All agents within 0.2 rad of the counterclockwise tangent |
| Operational convergence event | First **5 s continuous interval** satisfying both criteria |
| Proximity threshold | `ds = 1.0 m`, a design threshold, **not** a collision model |

The 100-trial study uses an idealized noise-free plant and direct application of
the same turning law. Separate ROS 2 closed-loop runs verify that the compact
model matches the controller's selected trajectory metrics. The study is
**not an exact reproduction** of the paper's four- and six-vehicle experiments,
their initial-state distribution, or their convergence criterion.

### Convergence results: 100 seeds, 160 s

| Metric | Result |
| --- | ---: |
| Reached a five-second radial **and** heading dwell | **86 / 100** |
| Within radial and heading tolerances at `t = 160 s` | **83 / 100** |
| Median first dwell start time, among 86 successes | **70.15 s** |
| Median final maximum radial error across the 100 runs | **0.02036 m** |
| Initial separation below 1.0 m | **0 / 100** |
| Separation below 1.0 m at any simulated time | **58 / 100** |
| Separation below 1.0 m after the first five seconds | **58 / 100** |
| Minimum distance across all runs | **0.01364 m** |

A case not meeting these tolerances by 160 s should be described as **not
meeting the finite-horizon criterion**, not as mathematically divergent.
Likewise, a successful five-second dwell does not prove the state will remain
within the specified band for all future time.

### Representative ROS 2 stress scenarios: seven runs, 35 s each

These experiments instantiate the **actual ROS 2 controller nodes** and a
synthetic-pose/virtual-plant node, rather than calling only the compact model.
Values are calculated from true virtual-plant states; sensor noise only
disturbs reported poses.

| Initial condition / perturbation | Final radial SD (m) | Minimum separation (m) | Final five-second mean-radius change (m) |
| --- | ---: | ---: | ---: |
| Baseline asymmetric | 0.018 | 2.738 | 0.041 |
| Compact | 0.015 | 1.337 | 0.347 |
| Dispersed | 0.038 | 2.337 | 0.197 |
| Random, seed 19 | 0.149 | **0.904** | 0.139 |
| Baseline + Gaussian 5 cm position / 2° yaw noise | 0.015 | 2.740 | 0.039 |
| Baseline + 200 ms delayed pose delivery | 0.020 | 2.738 | 0.052 |
| Random seed 19 + noise + delay | 0.188 | **0.882** | 0.094 |

A small final radial standard deviation means the robots have *similar*
radii; it is not, by itself, evidence that they have reached the theoretical
radius. A low final five-second mean-radius change is likewise only a
finite-time diagnostic. Delays apply to pose arrival, not the command channel.
The existing controller checks pose freshness at arrival time rather than
using the original measurement timestamp.

## Investigation of the close approaches

The controller adds two turning contributions:

```text
total yaw command = beacon steering + sum(neighbor steering)
neighbor steering = k_j * g(distance; c_j, d_0) * signed_bearing
```

For `c_j = 2`, the function `g` is **negative below the desired
inter-agent distance `d_0 = 2.83 m`**, but its value remains finite as
distance approaches zero: `g(0) = -ln(2)`.

Thus the steering term approaches zero if a neighbor is directly ahead
(`signed_bearing → 0`), even when the neighbor is very close. The plant
continues moving forward at a fixed 0.3 m/s. The controller does not
implement braking or a hard minimum-distance constraint.

### Visibility at the first crossing below 1 m

Among the 58 close-approach trials, at the numerical sample immediately
before the first crossing:

- **5** pairs had **mutual** forward-sector visibility.
- **35** had **one-way** forward-sector visibility.
- **18** had **neither** robot in the other's forward sector.

The controller's short-range omnidirectional detection only applies **inside**
`ds = 1 m`. Mutual detection is therefore often delayed, although it is
**not sufficient** to prevent a near pass.

### Diagnostic ablations: 100 common seeds, 35 s each

These are *counterfactual tests* with identical initial states and unchanged
steering gains. They are **not recommendations to change the controller**.

| Perception / interaction assumption | Runs < 1 m | Runs < 0.5 m | Runs < 0.25 m | Lowest distance |
| --- | ---: | ---: | ---: | ---: |
| Original controller | **58** | **16** | **4** | **0.0136 m** |
| Full 360° detection within the existing 4 m range | **35** | **8** | **4** | **0.0272 m** |
| No inter-agent steering (beacon only) | **56** | **40** | **18** | **0.0081 m** |

Improved visibility reduced the frequency of close passes, but did not
eliminate them. Disabling inter-agent steering substantially increased the
number of very close passes below 0.5 m, indicating that the existing
interaction term has a meaningful protective effect without guaranteeing
minimum separation.

### Worst-case replay: random seed 23

| Event | Time / observation |
| --- | --- |
| Initial minimum pairwise distance | 3.066 m |
| First crossing below `ds = 1.0 m` | `t = 4.45 s`, separation **0.973 m** |
| Relative closing speed at that crossing | **0.553 m/s** |
| Pair-specific interaction steering at that crossing | **+2.80°/s** and **−2.81°/s** |
| Visibility at that crossing | Both agents visible |
| Closest point-robot approach | `t = 6.30 s`, separation **0.01364 m** |

The seed-23 pair was mutually visible before crossing the proximity threshold,
but the small turning response was insufficient to prevent a near-intersection
of the two simulated trajectories. This demonstrates a second limitation
beyond delayed visibility.

**Interpretation:** The close approaches are consistent with limitations of
the *implemented steering-only interaction policy* and its observation
geometry. The diagnostics do **not** establish a sign error in the existing
controller or invalidate the paper's stated *local* multi-vehicle stability
results. Global convergence and hard collision avoidance require separate
claims and evidence.

## Validation and traceability

| Evidence | Verified cloud run | Outputs |
| --- | --- | --- |
| ROS 2 build and baseline 50 s closed-loop run | [Run 45079514761](https://github.com/kiredoo/cyclic_pursuit/actions/runs/45079514761) | `closed-loop-simulation` artifact |
| Seven ROS 2 stress scenarios | [Run 45080165831](https://github.com/kiredoo/cyclic_pursuit/actions/runs/45080165831) | `closed-loop-robustness` artifact |
| Paper equations, 100 seeds, and ROS 2 / compact-model comparison | [Run 45081673964](https://github.com/kiredoo/cyclic_pursuit/actions/runs/45081673964) | `paper-convergence-study` and `closed-loop-robustness` artifacts |
| Worst-seed trace and visibility/interaction ablations | [Run 45082800138](https://github.com/kiredoo/cyclic_pursuit/actions/runs/45082800138) | `close-approach-diagnosis` artifact |

The CI results above were verified as **completed successfully**. A green CI
result means the experiments executed and exported finite outputs; it does
**not** indicate that a safety or convergence guarantee was met. GitHub
Actions artifacts may expire. The important numerical findings are preserved
in this report so they remain reviewable after artifact retention ends.

The compact-model control law was tested against the original ROS 2 utility
functions across multiple seeded poses. The 35-second random-seed-19
comparison matched selected ROS 2 trajectory metrics to numerical
precision. The configured parameters passed the implemented checks for
paper Equations 13, 33, 35, and 45.

One retained implementation difference is that beacon steering is suppressed
inside a 0.1 m radius rather than only at the origin. Command units/signs in
the virtual plant were adapted to reproduce the historical implementation;
Crazyswarm2 driver's actual flight interface is not validated.

## Reproducing the experiments

On a Python environment with NumPy, PyYAML, Matplotlib and pytest installed,
from the repository root:

```bash
export PYTHONPATH=ros2_ws/src/cyclic_pursuit

python3 -m pytest -q ros2_ws/sim/test_paper_validation.py
python3 -m pytest -q ros2_ws/sim/test_close_approaches.py

python3 ros2_ws/sim/run_paper_study.py \
  --trials 100 --duration 160 --dt 0.05 \
  --output-dir artifacts/paper-study

python3 ros2_ws/sim/diagnose_close_approaches.py \
  --trials 100 --duration 35 --dt 0.05 --seed 23 \
  --output-dir artifacts/close-approach-diagnosis
```

For the ROS 2 closed-loop scenarios, use a ROS 2 Humble environment and
follow [the workspace instructions](../ros2_ws/README.md). GitHub Actions
provides a reproducible container build without physical hardware.

## Scope, limitations, and decision

- **Keep the experimental controller unchanged.** No safety-layer,
  visibility, gain, or control-policy revisions were made as part of this
  documentation.
- The model consists of *planar point unicycles*, not 3-D multirotor bodies.
  It excludes finite robot size, actuator saturation, propeller dynamics,
  contact/collision response, radio failures, and real-world localization.
- The seeded distribution is a controlled sample, not a claim of robustness
  to *every* initial configuration. Some trials may need a longer horizon.
- A computed inter-agent separation of 0.01364 m is a near-intersection in
  a point-robot simulation, **not** a measured physical drone collision.
- Neither local stability conditions nor these numerical experiments prove
  global convergence or guaranteed collision avoidance.
- Current results are suitable for documenting the research prototype, not
  for claiming it is safe to fly on physical drones.

**Project disposition:** Preserve the original algorithm and document both
its formation-seeking behavior and the observed close-approach limitations.
