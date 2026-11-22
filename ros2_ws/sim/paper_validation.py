"""Paper-aligned planar model and diagnostics for Ceccarelli et al. (Automatica 2008).

Implements equations (4)-(6) and (24)-(27) with the existing ROS 2 project's
configured parameters. The numerical integrator is the same midpoint unicycle
integrator as the ROS 2 virtual plant. This is a separate, hardware-free model
for parameter studies; never interpreted as a formal stability proof.

The historical ROS implementation additionally zeroes the beacon term when
rho < 0.1 m, whereas the paper zeros it only at rho == 0. We retain the
implemented behavior for comparisons, and flag that discrepancy.
"""
import math
import statistics

import numpy as np

TAU = 2 * math.pi


def params_from_yaml(path):
    import yaml
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)["params"]["ros__parameters"]


def g_function(rho, c, rho0):
    return math.log(((c - 1) * rho + rho0) / (c * rho0))


def equilibrium_radius(p):
    """Unique positive solution of the paper's equation (14)."""
    v, kb, cb, rho0 = float(p["v"]), float(p["k"]), float(p["c"]), float(p["rho_o"])
    if min(v, kb, rho0) <= 0 or cb <= 1:
        raise ValueError("Equation (14) requires v,kb,rho0>0 and cb>1")
    def residual(rho):
        return v / rho - kb * g_function(rho, cb, rho0) * math.pi / 2
    lo, hi = rho0, max(1.0, 2 * rho0)
    for _ in range(80):
        if residual(hi) < 0:
            break
        hi *= 2
    else:
        raise ValueError("Could not bracket equation (14)")
    for _ in range(90):
        mid = (lo + hi) / 2
        if residual(mid) > 0:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def paper_parameter_checks(p):
    """Check the sufficient geometrical and stability inequalities.

    Equation (13) uses a one-dimensional golden-section minimization over
    the interval where g(rho)<0; its reported margin is numeric.
    """
    v, kb, kv = float(p["v"]), float(p["k"]), float(p["kj"])
    cb, cv = float(p["c"]), float(p["cj"])
    rho0, d0, ds, dl = (float(p[k]) for k in ("rho_o", "d_0", "ds", "dl"))
    av, psi, n = float(p["av"]), float(p["psi"]), int(p["n"])
    rhoe = equilibrium_radius(p)
    upper = rho0  # g changes sign precisely at rho0
    a, b = 0.0, upper
    golden = (math.sqrt(5) - 1) / 2
    objective = lambda r: r * g_function(r, cb, rho0)
    c1 = b - golden * (b - a)
    d1 = a + golden * (b - a)
    for _ in range(90):
        if objective(c1) < objective(d1):
            b = d1
        else:
            a = c1
        c1 = b - golden * (b - a)
        d1 = a + golden * (b - a)
    min_rho_g = min(objective(0.0), objective(rho0), objective((a + b)/2))
    rhs13 = -2 * v / (3 * math.pi * kb)
    ratio = d0 / (2 * rhoe)
    geometric_domain = 0 < ratio < 1 and 0 < av <= math.pi/2
    phi = min(av, math.asin(min(1.0, dl/(2*rhoe)))) if geometric_domain else float("nan")
    alpha = math.asin(ratio) if geometric_domain else float("nan")
    limit45 = 2 * (cv / cb) * ((cb - 1)/(cv - 1))
    checks = {
        "eq13": min_rho_g > rhs13,
        "eq33": geometric_domain and (n - 1)*alpha + phi < math.pi,
        "eq35": geometric_domain and 2*alpha > phi,
        "eq45": kv / kb <= limit45,
        "visibility_domain": geometric_domain and ds < d0 < dl and alpha < av,
        "psi_domain": 1.5*math.pi < psi < 2*math.pi,
    }
    return {
        "equilibrium_radius_m": rhoe,
        "checks": checks,
        "eq13_min_rho_g": min_rho_g,
        "eq13_rhs": rhs13,
        "eq33_lhs_rad": (n-1)*alpha + phi,
        "eq33_rhs_rad": math.pi,
        "eq35_lhs_rad": 2*alpha,
        "eq35_rhs_rad": phi,
        "eq45_lhs": kv/kb,
        "eq45_rhs": limit45,
        "implementation_discrepancies": [
            "ROS 2 controller suppresses the beacon command for rho < 0.1 m; paper uses rho == 0.",
            "ROS 2 Twist yaw sign and deg/s were mapped to CCW rad/s in the virtual unicycle plant; hardware driver convention is not verified.",
        ],
        "scope": "These sufficient conditions concern specified equilibria, not global multivehicle convergence or guaranteed collision avoidance.",
    }


def random_initial_conditions(seed, count=3):
    """Same seeded initial sampling convention as closed_loop_ros2.py."""
    rng = np.random.default_rng(seed)
    angles = np.arange(count)*(2*math.pi/count) + rng.uniform(-.5, .5, size=count)
    radii = rng.uniform(1., 3., size=count)
    headings = rng.uniform(-math.pi, math.pi, size=count)
    return [
        (float(r*math.cos(a)), float(r*math.sin(a)), float(h))
        for r, a, h in zip(radii, angles, headings)
    ]


def model_commands(states, p):
    """Historical ROS2 turning law expressed in counterclockwise rad/s."""
    out = []
    kb, kv = float(p["k"]), float(p["kj"])
    cb, cv = float(p["c"]), float(p["cj"])
    rho0, d0 = float(p["rho_o"]), float(p["d_0"])
    ds, dl, av, psi = float(p["ds"]), float(p["dl"]), float(p["av"]), float(p["psi"])
    for i, (x, y, theta) in enumerate(states):
        rho = math.hypot(x, y)
        gamma = (math.atan2(-y, -x) - theta) % TAU
        alpha = gamma if gamma <= psi else gamma - TAU
        u_beacon = 0.0 if rho < 0.1 else kb * g_function(rho, cb, rho0) * alpha
        u_inter = 0.0
        for j, (other_x, other_y, _) in enumerate(states):
            if i == j:
                continue
            dx, dy = other_x-x, other_y-y
            dist = math.hypot(dx, dy)
            beta = (math.atan2(dy, dx) - theta + math.pi) % TAU - math.pi
            if dist < ds or (dist <= dl and abs(beta) <= av):
                strength = 0.0 if dist < 1e-9 else g_function(dist, cv, d0)
                u_inter += kv * strength * beta
        out.append(u_beacon + u_inter)
    return out


def rollout(p, seed, duration=160.0, dt=0.05, radial_tol=0.1, heading_tol=0.2, dwell=5.0):
    """Run one zero-noise deterministic, fully observed initial-condition trial.

    'Convergence time' is first instant of a 5-second continuous in-band
    interval, not evidence of stability for all subsequent time.
    """
    if not 0 < dt <= 0.2 or duration <= dwell:
        raise ValueError("Invalid numerical time horizon")
    states = random_initial_conditions(seed, int(p["n"]))
    rhoe = equilibrium_radius(p)
    nsteps = round(duration / dt)
    min_sep = math.inf
    min_sep_after_5s = math.inf
    initial_min_sep = None
    first_dwell_t = None
    streak = 0
    late_radii = []
    final_errors = []
    final_heading = []
    below_limit_steps = 0
    for step in range(nsteps + 1):
        t = step * dt
        rho = [math.hypot(x, y) for x, y, _ in states]
        headerr = []
        for x, y, theta in states:
            gamma = (math.atan2(-y, -x) - theta) % TAU
            headerr.append(abs(math.atan2(
                math.sin(gamma - math.pi/2), math.cos(gamma - math.pi/2)
            )))
        final_errors = [abs(r - rhoe) for r in rho]
        final_heading = headerr
        in_band = all(e < radial_tol for e in final_errors) and all(
            e < heading_tol for e in headerr
        )
        if in_band:
            streak += 1
            if first_dwell_t is None and streak*dt >= dwell:
                first_dwell_t = round(t - dwell + dt, 3)
        else:
            streak = 0
        pair_dists = [
            math.hypot(states[i][0]-states[j][0], states[i][1]-states[j][1])
            for i in range(len(states)) for j in range(i+1, len(states))
        ]
        current_min = min(pair_dists)
        if step == 0:
            initial_min_sep = current_min
        min_sep = min(min_sep, current_min)
        if t >= 5.0:
            min_sep_after_5s = min(min_sep_after_5s, current_min)
        if current_min < float(p["ds"]):
            below_limit_steps += 1
        if t >= duration - 5.0:
            late_radii.append(statistics.mean(rho))
        if step == nsteps:
            break
        next_states = []
        for (x, y, theta), omega in zip(states, model_commands(states, p)):
            theta_mid = theta + omega * dt / 2.0
            next_states.append((
                x + float(p["v"]) * math.cos(theta_mid) * dt,
                y + float(p["v"]) * math.sin(theta_mid) * dt,
                math.atan2(math.sin(theta + omega*dt), math.cos(theta + omega*dt)),
            ))
        states = next_states
    return {
        "seed": seed,
        "duration_s": duration,
        "first_five_second_dwell_s": first_dwell_t,
        "final_in_radial_heading_band": (
            max(final_errors) < radial_tol and max(final_heading) < heading_tol
        ),
        "final_max_abs_radius_error_m": max(final_errors),
        "final_max_abs_heading_error_rad": max(final_heading),
        "final_mean_radius_m": statistics.mean(rho),
        "final_radius_std_m": statistics.pstdev(rho),
        "initial_min_pairwise_m": initial_min_sep,
        "minimum_pairwise_m": min_sep,
        "minimum_pairwise_after_5s_m": min_sep_after_5s,
        "any_separation_below_ds": min_sep < float(p["ds"]),
        "separation_below_ds_at_t0": initial_min_sep < float(p["ds"]),
        "fraction_steps_below_ds": below_limit_steps/(nsteps+1),
        "last_five_seconds_mean_radius_change_m": abs(late_radii[-1]-late_radii[0]),
    }
