"""
Centipede Robot — Gait Parameter Sweep
=======================================
Batch simulation of a 4-module centipede robot in MuJoCo.
Sweeps over gait parameters (spine frequency, phase offsets, amplitudes,
duty factor) and records locomotion distance for each combination.

Usage:
    python param_sweep.py          # from centipede_sim/scripts/
    python scripts/param_sweep.py  # from centipede_sim/
"""

import mujoco
import mujoco.viewer
import numpy as np
import csv
from datetime import datetime
from pathlib import Path


# ─────────────────────────────────────────────
# Path setup (all paths relative to project root)
# ─────────────────────────────────────────────
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_PATH   = PROJECT_ROOT / "model" / "centipede_4module.xml"
OUTPUT_DIR   = PROJECT_ROOT / "output"


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────
def deg2rad(d):
    return d * np.pi / 180


def leg_angle(phase, A_leg, leg_offset, dutyfactor):
    """
    Returns leg angle based on sine wave phase.
    Sine wave shifted by -pi/2 so the transition happens at the
    shoulder's zero crossing (peak of swing -> start of stance).
    Up (in air) when sin > threshold, down (on ground) otherwise.
    duty > 0.5 lowers the threshold -> longer ground contact.
    """
    A_leg_up   = -A_leg         # angle when leg is in air
    A_leg_down = -leg_offset    # angle when leg is on ground
    duty       = 1 - dutyfactor # 0.5 = equal time up/down, increase for more ground time
    leg_threshold = np.cos(duty * np.pi)
    s = np.sin(phase - np.pi / 2)  # shift pi/2: leg lifts after shoulder peaks
    return A_leg_up if s > leg_threshold else A_leg_down


# ─────────────────────────────────────────────
# Load model
# ─────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
data  = mujoco.MjData(model)


# ─────────────────────────────────────────────
# Joint / actuator indices (4 modules)
# ─────────────────────────────────────────────
SHOULDER_R = [0,  6,  12, 18]
LEG_R      = [1,  7,  13, 19]
SHOULDER_L = [2,  8,  14, 20]
LEG_L      = [3,  9,  15, 21]
LINK_Y     = [4,  10, 16]
LINK_P     = [5,  11, 17]

N_CTRL = model.nu


# ─────────────────────────────────────────────
# Build ctrl command for a given time t
# ─────────────────────────────────────────────
def compute_ctrl(t, omega_spine, phi_spine, phi_bodyleg,
                 A_spine_y, A_shoulder, dutyfactor, leg_offset, A_leg):
    ctrl = np.zeros(N_CTRL)

    # Spine lateral undulation (Link_Y)
    for i, idx in enumerate(LINK_Y):
        ctrl[idx] = A_spine_y * np.sin(omega_spine * t + i * phi_spine)

    # Spine vertical undulation (Link_P) — currently disabled
    for i, idx in enumerate(LINK_P):
        ctrl[idx] = 0

    # Shoulder swing
    for i in range(4):
        phase = omega_spine * t + i * phi_spine + phi_bodyleg
        ctrl[SHOULDER_R[i]] =  leg_offset - A_shoulder * np.sin(phase)
        ctrl[SHOULDER_L[i]] = -leg_offset - A_shoulder * np.sin(phase)

    # Leg raise / plant
    for i in range(4):
        phase = omega_spine * t + i * phi_spine + phi_bodyleg
        ctrl[LEG_R[i]] = leg_angle(phase,          A_leg, leg_offset, dutyfactor)
        ctrl[LEG_L[i]] = leg_angle(phase + np.pi,  A_leg, leg_offset, dutyfactor)

    return ctrl


# ─────────────────────────────────────────────
# Simulation runner
# ─────────────────────────────────────────────
def run_simulation(omega_spine, phi_spine, phi_bodyleg,
                   A_spine_y, A_shoulder, dutyfactor,
                   num_cycles=10, show_viewer=False,
                   leg_offset=deg2rad(-5), A_leg=deg2rad(40)):
    """
    Run a single simulation with given parameters.
    Returns: (travel_x, travel_y, abs_dist, total_time) or None on failure.
    """
    mujoco.mj_resetData(model, data)

    dt              = model.opt.timestep
    gait_period     = 2 * np.pi / omega_spine
    simulation_time = num_cycles * gait_period
    steps           = int(simulation_time / dt)

    # Optional viewer
    viewer = None
    if show_viewer:
        viewer = mujoco.viewer.launch_passive(model, data)

    # Ramp-up phase (1.5 s) — smoothly interpolate to initial pose
    target_init = compute_ctrl(
        0.0, omega_spine, phi_spine, phi_bodyleg,
        A_spine_y, A_shoulder, dutyfactor, leg_offset, A_leg)
    init_steps = int(1.5 / dt)

    for k in range(init_steps):
        data.ctrl[:] = (k / init_steps) * target_init
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()
            if not viewer.is_running():
                viewer.close()
                return None

    start_pos = data.qpos[0:3].copy()

    # Main gait loop
    try:
        for step in range(steps):
            t = step * dt
            data.ctrl[:] = compute_ctrl(
                t, omega_spine, phi_spine, phi_bodyleg,
                A_spine_y, A_shoulder, dutyfactor, leg_offset, A_leg)
            mujoco.mj_step(model, data)
            if viewer:
                viewer.sync()
                if not viewer.is_running():
                    viewer.close()
                    return None
    except KeyboardInterrupt:
        if viewer:
            viewer.close()
        return None

    if viewer:
        viewer.close()

    final_pos = data.qpos[0:3].copy()
    travel_x  = final_pos[0] - start_pos[0]
    travel_y  = final_pos[1] - start_pos[1]
    abs_dist  = np.sqrt(travel_x**2 + travel_y**2)

    return travel_x, travel_y, abs_dist, simulation_time


# ─────────────────────────────────────────────
# Parameter sweep configuration
# ─────────────────────────────────────────────

# Baseline parameters (used when a parameter is not being swept)
BASE_PARAMS = {
    "omega_spine":             4,
    "phi_spine":              -0.5 * 2 * np.pi,
    "bodyleg_coupling_factor": 0.5,
    "A_spine_y":               deg2rad(0),
    "A_shoulder":              deg2rad(30),
    "dutyfactor":              0.5,
}

# Which parameters to sweep (1 = sweep, 0 = hold at baseline)
# [omega_spine, phi_spine, bodyleg_coupling_factor, A_spine_y & A_shoulder, dutyfactor]
VARY_FLAGS = [0, 0, 0, 0, 0]

# Viewer toggle
SHOW_VIEWER = True

# Sweep pools
PARAM_POOLS = {
    "omega_spine": np.arange(1, 8.5, 0.25).tolist(),
    "phi_spine": np.arange(-0.1 * 2 * np.pi,
                           -0.55 * 2 * np.pi,
                           -0.0125 * 2 * np.pi).tolist(),
    "bodyleg_coupling_factor": np.arange(0, 1.1, 0.1).tolist(),
    "A_spine_y_A_shoulder": [
        (deg2rad(0),  deg2rad(40)),
        (deg2rad(5),  deg2rad(38)),
        (deg2rad(10), deg2rad(33)),
        (deg2rad(15), deg2rad(30)),
        (deg2rad(20), deg2rad(26)),
        (deg2rad(25), deg2rad(23)),
        (deg2rad(30), deg2rad(20)),
        (deg2rad(35), deg2rad(16)),
        (deg2rad(40), deg2rad(12)),
    ],
    "dutyfactor": [0.35, 0.375, 0.4, 0.425, 0.45, 0.475,
                   0.5, 0.525, 0.55, 0.575, 0.6],
}

# Simulation length
NUM_CYCLES = 8


# ─────────────────────────────────────────────
# Combination builder
# ─────────────────────────────────────────────
def build_combinations(vary_flags, base, pools):
    """
    Build the Cartesian product of all swept parameters.
    phi_bodyleg is derived:  -pi/2 + (-phi_spine * bodyleg_coupling_factor)
    """
    omega_vals   = pools["omega_spine"]             if vary_flags[0] else [base["omega_spine"]]
    phi_vals     = pools["phi_spine"]               if vary_flags[1] else [base["phi_spine"]]
    blc_vals     = pools["bodyleg_coupling_factor"]  if vary_flags[2] else [base["bodyleg_coupling_factor"]]
    amp_pairs    = pools["A_spine_y_A_shoulder"]     if vary_flags[3] else [(base["A_spine_y"], base["A_shoulder"])]
    duty_vals    = pools["dutyfactor"]               if vary_flags[4] else [base["dutyfactor"]]

    combos = []
    for omega in omega_vals:
        for phi in phi_vals:
            for blc in blc_vals:
                phi_bodyleg = -np.pi / 2 + (-phi * blc)
                for A_sy, A_sh in amp_pairs:
                    for duty in duty_vals:
                        combos.append({
                            "omega_spine": omega,
                            "phi_spine": phi,
                            "bodyleg_coupling_factor": blc,
                            "phi_bodyleg": phi_bodyleg,
                            "A_spine_y": A_sy,
                            "A_shoulder": A_sh,
                            "dutyfactor": duty,
                        })
    return combos


# ─────────────────────────────────────────────
# Main sweep
# ─────────────────────────────────────────────
if __name__ == "__main__":

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Build output filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    param_tags = ["omega", "phi", "blc", "amp", "duty"]
    swept = "_".join(param_tags[i] for i, f in enumerate(VARY_FLAGS) if f) or "baseline"
    csv_path = OUTPUT_DIR / f"sweep_{swept}_{NUM_CYCLES}cyc_{timestamp}.csv"

    CSV_FIELDS = [
        "omega_spine", "phi_spine", "bodyleg_coupling_factor", "phi_bodyleg",
        "A_spine_y", "A_shoulder", "dutyfactor",
        "travel_x", "travel_y", "abs_dist", "sim_time",
    ]

    combos = build_combinations(VARY_FLAGS, BASE_PARAMS, PARAM_POOLS)

    print("=" * 72)
    print("CENTIPEDE GAIT — PARAMETER SWEEP")
    print("=" * 72)
    print(f"  Model:      {MODEL_PATH}")
    print(f"  Output:     {csv_path}")
    print(f"  Vary flags: {VARY_FLAGS}  (omega, phi, blc, amp, duty)")
    print(f"  Cycles:     {NUM_CYCLES}")
    print(f"  Viewer:     {SHOW_VIEWER}")
    print(f"  Total sims: {len(combos)}")
    print("=" * 72 + "\n")

    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        writer.writeheader()

        for idx, p in enumerate(combos, 1):
            period = 2 * np.pi / p["omega_spine"]
            print(f"\n[{idx}/{len(combos)}]  "
                  f"omega={p['omega_spine']:.2f} rad/s (T={period:.3f}s)  "
                  f"phi={np.degrees(p['phi_spine']):.1f} deg  "
                  f"A_sy={np.degrees(p['A_spine_y']):.1f} deg  "
                  f"A_sh={np.degrees(p['A_shoulder']):.1f} deg  "
                  f"blc={p['bodyleg_coupling_factor']:.2f}  "
                  f"duty={p['dutyfactor']:.3f}")

            result = run_simulation(
                omega_spine=p["omega_spine"],
                phi_spine=p["phi_spine"],
                phi_bodyleg=p["phi_bodyleg"],
                A_spine_y=p["A_spine_y"],
                A_shoulder=p["A_shoulder"],
                dutyfactor=p["dutyfactor"],
                num_cycles=NUM_CYCLES,
                show_viewer=SHOW_VIEWER,
            )

            if result is None:
                print("  >> interrupted / failed")
                continue

            tx, ty, dist, t_sim = result
            print(f"  >> x={tx:.4f} m  y={ty:.4f} m  dist={dist:.4f} m")

            writer.writerow({
                "omega_spine":             f"{p['omega_spine']:.6f}",
                "phi_spine":               f"{p['phi_spine'] / (-2*np.pi):.6f}",
                "bodyleg_coupling_factor":  f"{p['bodyleg_coupling_factor']:.6f}",
                "phi_bodyleg":             f"{p['phi_bodyleg']:.6f}",
                "A_spine_y":               f"{p['A_spine_y']:.6f}",
                "A_shoulder":              f"{p['A_shoulder']:.6f}",
                "dutyfactor":              f"{p['dutyfactor']:.6f}",
                "travel_x":               f"{tx:.6f}",
                "travel_y":               f"{ty:.6f}",
                "abs_dist":               f"{dist:.6f}",
                "sim_time":               f"{t_sim:.3f}",
            })
            f.flush()

    print("\n" + "=" * 72)
    print(f"SWEEP COMPLETE — results saved to {csv_path}")
    print("=" * 72)
