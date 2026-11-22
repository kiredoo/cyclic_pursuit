# Physical Crazyflie experiments: results from recorded flights

**Historical tests:** September 2024 to May 2025, MURO Lab, UC San Diego.  
**Platform:** Crazyflie drones · ROS 2 · Crazyswarm2 · OptiTrack motion capture.  
**Status:** Archive of experimental evidence, **not** a verified flight-ready controller.

The final project report documents implementation of the circular-pursuit
controller described by Ceccarelli *et al.* and physical tests with two,
three, and four Crazyflies. Circular pursuit was **successfully demonstrated
with two and three physical drones**. A four-drone experiment did **not**
succeed: the report describes an unexpected mid-flight flip and deviations
from simulated trajectories. The root cause was **not established**. The
classification of success or failure is taken from the historical report
and the recording labels, not independently inferred from pose curves.

These flights occurred **before** the 2026 cloud-based ROS 2 simulation
and 100-seed numerical evaluation. They demonstrate real hardware integration
but must not be confused with a guarantee of collision avoidance or with
the results of the newer idealized experiments.

## Recorded trajectories

The plots below are **simplified SVG redrawings from the supplied ROS 2
pose recordings**, not copies of the original MATLAB JPEGs. They retain the
chronological pose paths, with line simplification for readability; **use the
quantitative values and original bags for distances, not the illustrations**.
Each ○ denotes a recorded start and × a recorded endpoint.

### Two Crazyflies — successful trial

![Two physical Crazyflies: measured planar trajectories](figures/real-two-crazyflies.svg)

The flight exhibited repeated orbital motion after the initial approach.
The available bag includes **two pose topics and two command topics**.
This is a successful circular-pursuit demonstration as documented by the
original team, but the minimum estimated separation in the recording was
only **0.164 m**, below the project's 0.5 m practical guideline.

### Three Crazyflies — successful trial

![Three physical Crazyflies: measured planar trajectories](figures/real-three-crazyflies.svg)

The recorded paths show repeated motion around a common region. The final
report describes successful approximate circular pursuit in the three-agent
flight, but notes that the experimental equilibrium radius appeared to
differ from the simulation. The supplied bag contains **three pose
topics**, but **no command topics**; the controller commands for this run
cannot be reconstructed from these recordings alone.

### Four Crazyflies — unsuccessful trial

![Four physical Crazyflies: measured planar trajectories](figures/real-four-crazyflies.svg)

The four-drone paths are irregular. The final report describes a robot
flipping during the trial, which affected the rest of the formation. Possible
contributors discussed in the report include flight-controller/firmware
problems and packet loss as swarm size increased. **These are hypotheses,
not confirmed root causes.** This bag contains **four pose topics**
and no recorded controller command topics.

## Measurements from the recorded ROS 2 bags

| Flight recording | Common pose-recording interval | Pose topics | Command topics | Estimated minimum 3-D separation over full recording | Estimated minimum while all drones had z > 0.35 m |
| --- | ---: | ---: | ---: | ---: | ---: |
| Two-drone success | 88.06 s | 2 | 2 | **0.164 m** | **0.164 m** |
| Three-drone success | 104.84 s | 3 | 0 | **0.489 m** | **0.586 m** |
| Four-drone failure | 106.45 s | 4 | 0 | **0.105 m** | **0.108 m** |

These values were computed from the supplied ROS 2 SQLite `.db3` bags.
For every recorded pose topic, positions were linearly interpolated to a
**common 20 Hz timestamp grid**. Euclidean pairwise separation was
computed using all three positional coordinates `x, y, z`. The reported
minimum is the smallest value across all pairs over that interval.

The second minimum restricts samples to times when **every participating
robot had recorded `pose.position.z > 0.35 m`**. This threshold is an
analysis convention, not a confirmed takeoff/landing classification. Full
recordings may include initialization, takeoff, landing, or tracking artifacts.
Interpolation and timestamp alignment also introduce uncertainty.

The historical report used **0.5 m** as a practical minimum separation
rule of thumb, rather than establishing a universal collision threshold.
The low estimated separation in the otherwise successful two-drone test
shows why successful orbiting and safe clearance are **different claims**.
These values **do not establish that any physical collision occurred**.

A compact machine-readable copy of the table is available at
[real-flight-summary.csv](real-flight-summary.csv).

## Experimental architecture and contributions

The hardware experiment used separate Python/ROS 2 controller nodes, the
Crazyswarm2 stack, Crazyradio communication, and OptiTrack position
measurements. The author's final report describes personal contributions
including implementing the missing multi-agent interaction portion of an
existing beacon-controller script, packaging the algorithm for ROS 2,
building simulation and MATLAB-analysis tools, selecting parameters, and
performing laboratory tests with collaborators.

The initial beacon term and early simulation originated with a previous
collaborator. The hardware setup and OptiTrack integration also benefited
from support by other MURO Lab members. The project should therefore be
described as a **collaborative research effort**, with individual
contributions distinguished where possible.

The report describes the following experiment-specific limitations:

- **Initial heading and tracking:** Single-marker OptiTrack tracking
  required a prescribed initial drone orientation and minimal movement
  during initialization. Incorrect alignment could cause position or
  orientation inconsistencies.
- **Scaling the system:** Unexpected flips and increasingly irregular
  flight were observed when testing larger formations.
- **Simulation-to-hardware discrepancy:** The radius seen in the
  three-drone physical test differed from the simulated value. The
  four-drone experiment deviated more strongly.
- **Incomplete command logging:** The provided three- and four-agent
  bags contain poses, not the full commanded-input history. Root-cause
  attribution based on the bag alone is not justified.

## Relation to the newer simulation study

The [2026 simulation validation report](simulation-validation-results.md)
documents ROS 2 closed-loop virtual-plant tests and 100 deterministic
initial-condition trials. Those tests found finite-time convergence behavior
but also cases where pairwise distance fell below the configured
simulation threshold of 1 m.

Those simulations use **planar point-unicycle dynamics** with idealized
actuation; the physical flights used quadrotors, actual communication,
firmware, and motion capture. Parameters, initial conditions, and robot
counts also differ. It is **not appropriate to compare simulated and
experimental distances as matched trials**.

## Provenance and publication choices

The figures in this page were regenerated from user-supplied laboratory
recordings labeled `2_crazyflie_success.db3`,
`3_crazyflie_success.db3`, and `4_crazyflie_fail.db3`. The original
final project report (May 2025), the original MATLAB screenshots, and
raw bags were consulted to prepare this documentation.

The raw `.db3` files and unredacted PDF **are not included** in this
public repository update. They were intentionally kept separate for
privacy, attribution, data-management, and redistribution review. The
SVG plots are derived visualizations; they are **not** the original
recorded data. The original experimental report remains the source
for the team's qualitative success/failure statements and observations.

**Limitations:** The experiments do not demonstrate stable four- or
five-drone operation, guaranteed collision avoidance, global theoretical
convergence, or that the historical flight controller is currently
ready for safe physical deployment.

**Further reference:** Ceccarelli, Di Marco, Garulli, and Giannitrapani,
“Collective circular motion of multi-vehicle systems,” *Automatica*
44(12), 3109–3114 (2008), doi:10.1016/j.automatica.2008.04.009.
