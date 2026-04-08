import mujoco
import mujoco_viewer
import numpy as np
import csv
from datetime import datetime
from pathlib import Path

# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────
def deg2rad(d):
    return d * np.pi / 180


def leg_angle(phase, A_leg, leg_offset, dutyfactor):
    """
    Returns leg angle based on sine wave phase.
    Sine wave shifted by -pi/2 so the transition happens at the
    shoulder's zero crossing (peak of swing → start of stance).
    Up (in air) when sin > threshold, down (on ground) otherwise.
    duty > 0.5 lowers the threshold → longer ground contact.
    """
    A_leg_up   = -A_leg    # angle when leg is in air
    A_leg_down = -leg_offset    # angle when leg is on ground
    duty       = 1-dutyfactor            # 0.5 = equal time up/down, increase for more ground time
    leg_threshold = np.cos(duty * np.pi)
    s = np.sin(phase - np.pi / 2)   # shift pi/2: leg lifts after shoulder peaks
    return A_leg_up if s > leg_threshold else A_leg_down

# ─────────────────────────────────────────────
# Load model
# ─────────────────────────────────────────────
model = mujoco.MjModel.from_xml_path(
    r'D:\PSU\Robots\unit_main_for urdf.SLDASM\urdf\4module_floating.xml')
data  = mujoco.MjData(model)

# ─────────────────────────────────────────────
# Joint indices
# ─────────────────────────────────────────────
SHOULDER_R = [0,  6,  12, 18]
LEG_R      = [1,  7,  13, 19]
SHOULDER_L = [2,  8,  14, 20]
LEG_L      = [3,  9,  15, 21]
LINK_Y     = [4,  10, 16]
LINK_P     = [5,  11, 17]

N_CTRL = model.nu

# ─────────────────────────────────────────────
# Fixed gait parameters (will be overridden by sweep)
# ─────────────────────────────────────────────
# omega_spine is now a swept parameter, see param_pools below

# ─────────────────────────────────────────────
# Build ctrl command for a given time t
# ─────────────────────────────────────────────
def compute_ctrl(t, omega_spine, phi_spine, phi_bodyleg, A_spine_y, A_shoulder, dutyfactor, leg_offset, A_leg):
    ctrl = np.zeros(N_CTRL)

    # ── Spine lateral undulation (Link_Y) ──
    for i, idx in enumerate(LINK_Y):
        ctrl[idx] = A_spine_y * np.sin(omega_spine * t + i * phi_spine)
        
    # ── Spine vertical undulation (Link_P) ──
    for i, idx in enumerate(LINK_P):
        ctrl[idx] = 0

    # ── Shoulder swing ──
    for i in range(4):
        ctrl[SHOULDER_R[i]] = leg_offset - A_shoulder * np.sin(omega_spine * t + i * phi_spine + phi_bodyleg)
        ctrl[SHOULDER_L[i]] = -leg_offset - A_shoulder * np.sin(omega_spine * t + i * phi_spine + phi_bodyleg)

    # ── Leg raise / plant ──
    for i in range(4):
        ctrl[LEG_R[i]] = leg_angle(omega_spine * t + i * phi_spine + phi_bodyleg, A_leg, leg_offset, dutyfactor)
        ctrl[LEG_L[i]] = leg_angle(omega_spine * t + i * phi_spine + phi_bodyleg + np.pi, A_leg, leg_offset, dutyfactor)

    return ctrl

# ─────────────────────────────────────────────
# Simulation function
# ─────────────────────────────────────────────
def run_simulation(omega_spine, phi_spine, phi_bodyleg, A_spine_y, A_shoulder, dutyfactor, 
                   num_cycles=10, show_viewer=False, leg_offset=deg2rad(-5), A_leg=deg2rad(25)):
    """
    Run a single simulation with given parameters.
    Returns: (travel_x, travel_y, abs_dist, total_time)
    """
    # Reset data
    mujoco.mj_resetData(model, data)
    
    # Simulation setup
    dt = model.opt.timestep
    gait_period = 2 * np.pi / omega_spine
    simulation_time = num_cycles * gait_period
    steps = int(simulation_time / dt)
    
    # Initialize viewer if requested
    viewer = None
    if show_viewer:
        viewer = mujoco_viewer.MujocoViewer(model, data)
    
    # Init phase
    target_init = compute_ctrl(0.0, omega_spine, phi_spine, phi_bodyleg, A_spine_y, A_shoulder, dutyfactor, leg_offset, A_leg)
    init_duration = 1.5
    init_steps = int(init_duration / dt)
    
    for k in range(init_steps):
        alpha = k / init_steps
        data.ctrl[:] = alpha * target_init
        mujoco.mj_step(model, data)
        if viewer:
            viewer.render()
            if not viewer.is_alive:
                viewer.close()
                return None
    
    # Record start position
    start_pos = data.qpos[0:3].copy()
    
    # Main loop
    try:
        for step in range(steps):
            t = step * dt
            data.ctrl[:] = compute_ctrl(t, omega_spine, phi_spine, phi_bodyleg, A_spine_y, A_shoulder, dutyfactor, leg_offset, A_leg)
            mujoco.mj_step(model, data)
            
            if viewer:
                viewer.render()
                if not viewer.is_alive:
                    viewer.close()
                    return None
    
    except KeyboardInterrupt:
        if viewer:
            viewer.close()
        return None
    
    if viewer:
        viewer.close()
    
    # Extract results
    final_pos = data.qpos[0:3].copy()
    travel_x = final_pos[0] - start_pos[0]
    travel_y = final_pos[1] - start_pos[1]
    abs_dist = np.sqrt(travel_x**2 + travel_y**2)
    
    return travel_x, travel_y, abs_dist, simulation_time

# ─────────────────────────────────────────────
# PARAMETER SWEEP CONFIGURATION
# ─────────────────────────────────────────────

# Define base parameters
base_params = {
    'omega_spine': 3,
    'phi_spine': -0.25*2*np.pi,
    'bodyleg_coupling_factor': 0.5,
    'A_spine_y': deg2rad(30),
    'A_shoulder': deg2rad(20),
    'dutyfactor': 0.375
}

# Which parameters to vary (1 = vary, 0 = keep fixed)
# Format: [omega_spine, phi_spine, bodyleg_coupling_factor, A_spine_y & A_shoulder, dutyfactor]
# NOTE: phi_bodyleg is automatically derived from phi_spine and bodyleg_coupling_factor
#       phi_bodyleg = -π/2 + (-phi_spine * bodyleg_coupling_factor)
vary_flags = [1, 0, 0, 0, 1]  # Example: vary omega_spine, phi_spine, and dutyfactor

# Parameter pools (what values to loop over)
param_pools = {
    'omega_spine': [1, 1.5, 2, 2.5, 3, 3.5, 4, 4.5, 5, 5.5, 6],  # rad/s
    'phi_spine': [-0.05*2*np.pi, -0.1*2*np.pi, -0.15*2*np.pi, -0.2*2*np.pi, -0.25*2*np.pi, -0.3*2*np.pi, -0.35*2*np.pi, -0.4*2*np.pi, -0.45*2*np.pi, -0.5*2*np.pi],
    'bodyleg_coupling_factor': [0, 0.1, 0.2, 0.3, 0.4, 1/2, 0.6, 0.7, 0.8 , 0.9, 1],
    'A_spine_y_A_shoulder': [  # Paired parameter - loops together
        (deg2rad(0), deg2rad(40)),
        (deg2rad(25), deg2rad(25)),
        (deg2rad(40), deg2rad(5)),
    ],
    'dutyfactor': [0.35, 0.375, 0.4, 0.425, 0.450, 0.475, 0.5, 0.525, 0.55, 0.575, 0.6]
}

# Number of cycles per simulation
num_cycles_per_sim = 10

# Show viewer during simulations? (slow but visual)
show_viewer = False
# show_viewer = True

# ─────────────────────────────────────────────
# Output directory setup
# ─────────────────────────────────────────────
output_dir = Path(r'D:\PSU\Robots\Sim\Centipide_4')
output_dir.mkdir(parents=True, exist_ok=True)

# CSV filename with timestamp and vary flags
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
param_names = ['omega_spine', 'phi_spine', 'bodyleg_coupling', 'A_spine_A_shoulder', 'dutyfactor']
varying_params = '_'.join([param_names[i] for i in range(len(vary_flags)) if vary_flags[i] == 1])
if not varying_params:
    varying_params = 'all_fixed'
csv_filename = output_dir / f"sweep_{varying_params}_{num_cycles_per_sim}cycles_{timestamp}.csv"

# CSV headers
csv_headers = ['omega_spine', 'phi_spine', 'bodyleg_coupling_factor', 'phi_bodyleg', 'A_spine_y', 'A_shoulder', 'dutyfactor', 
               'travel_x', 'travel_y', 'abs_dist', 'sim_time']

# ─────────────────────────────────────────────
# Build parameter combinations based on vary_flags
# ─────────────────────────────────────────────

def build_parameter_combinations(vary_flags, base_params, param_pools):
    """
    Build list of all parameter combinations to test based on vary_flags.
    vary_flags[0] = omega_spine (1=vary, 0=fixed)
    vary_flags[1] = phi_spine (1=vary, 0=fixed)
    vary_flags[2] = bodyleg_coupling_factor (1=vary, 0=fixed)
    vary_flags[3] = A_spine_y & A_shoulder (1=vary, 0=fixed) [PAIRED]
    vary_flags[4] = dutyfactor (1=vary, 0=fixed)
    
    phi_bodyleg is automatically derived: phi_bodyleg = -π/2 + (-phi_spine * bodyleg_coupling_factor)
    """
    combinations = []
    
    # Build lists of values for each parameter
    omega_spine_vals = param_pools['omega_spine'] if vary_flags[0] else [base_params['omega_spine']]
    phi_spine_vals = param_pools['phi_spine'] if vary_flags[1] else [base_params['phi_spine']]
    bodyleg_coupling_factor_vals = param_pools['bodyleg_coupling_factor'] if vary_flags[2] else [base_params['bodyleg_coupling_factor']]
    A_spine_y_A_shoulder_pairs = param_pools['A_spine_y_A_shoulder'] if vary_flags[3] else [(base_params['A_spine_y'], base_params['A_shoulder'])]
    dutyfactor_vals = param_pools['dutyfactor'] if vary_flags[4] else [base_params['dutyfactor']]
    
    # Generate all combinations
    for omega_spine in omega_spine_vals:
        for phi_spine in phi_spine_vals:
            for bodyleg_coupling_factor in bodyleg_coupling_factor_vals:
                phi_bodyleg = -np.pi / 2 + (-phi_spine * bodyleg_coupling_factor)  # Derive phi_bodyleg
                for A_spine_y, A_shoulder in A_spine_y_A_shoulder_pairs:
                    for dutyfactor in dutyfactor_vals:
                        combinations.append({
                            'omega_spine': omega_spine,
                            'phi_spine': phi_spine,
                            'bodyleg_coupling_factor': bodyleg_coupling_factor,
                            'phi_bodyleg': phi_bodyleg,
                            'A_spine_y': A_spine_y,
                            'A_shoulder': A_shoulder,
                            'dutyfactor': dutyfactor
                        })
    
    return combinations

# ─────────────────────────────────────────────
# MAIN PARAMETER SWEEP
# ─────────────────────────────────────────────

print("=" * 80)
print("PARAMETER SWEEP SIMULATION")
print("=" * 80)
print(f"Output directory: {output_dir}")
print(f"CSV file: {csv_filename}")
print(f"Vary flags: {vary_flags}")
print(f"  [0] omega_spine (rad/s)")
print(f"  [1] phi_spine")
print(f"  [2] bodyleg_coupling_factor")
print(f"  [3] A_spine_y & A_shoulder (paired)")
print(f"  [4] dutyfactor")
print(f"Note: phi_bodyleg = -π/2 + (-phi_spine * bodyleg_coupling_factor)")
print(f"Cycles per sim: {num_cycles_per_sim}")
print(f"Show viewer: {show_viewer}")

# Build combinations
combinations = build_parameter_combinations(vary_flags, base_params, param_pools)
total_sims = len(combinations)

print(f"Total simulations to run: {total_sims}")
print("=" * 80 + "\n")

# Open CSV file for writing
with open(csv_filename, 'w', newline='') as csvfile:
    writer = csv.DictWriter(csvfile, fieldnames=csv_headers)
    writer.writeheader()
    
    # Run each simulation
    for idx, params in enumerate(combinations, 1):
        # Detailed progress output
        gait_period = 2 * np.pi / params['omega_spine']
        print(f"\n{'='*80}")
        print(f"{total_sims} sims in total, current sim number {idx}")
        print(f"{'='*80}")
        print(f"[omega_spine = {params['omega_spine']:.4f} rad/s (period = {gait_period:.3f}s)]; "
              f"[phi_spine = {params['phi_spine']:.4f} ({np.degrees(params['phi_spine']):.1f}°)]; "
              f"[A_spine_y = {np.degrees(params['A_spine_y']):.1f}°, A_shoulder = {np.degrees(params['A_shoulder']):.1f}°]; "
              f"[bodyleg_coupling_factor = {params['bodyleg_coupling_factor']:.4f}, "
              f"dutyfactor = {params['dutyfactor']:.2f}]")
        print(f"phi_bodyleg (derived): {params['phi_bodyleg']:.4f} ({np.degrees(params['phi_bodyleg']):.1f}°)")
        print(f"{'='*80}")
        
        result = run_simulation(
            omega_spine=params['omega_spine'],
            phi_spine=params['phi_spine'],
            phi_bodyleg=params['phi_bodyleg'],
            A_spine_y=params['A_spine_y'],
            A_shoulder=params['A_shoulder'],
            dutyfactor=params['dutyfactor'],
            num_cycles=num_cycles_per_sim,
            show_viewer=show_viewer
        )
        
        if result is None:
            print("  ⚠ Simulation interrupted or failed!")
            continue
        
        travel_x, travel_y, abs_dist, sim_time = result
        
        print(f"  ✓ Result: travel_x={travel_x:.4f}m, travel_y={travel_y:.4f}m, abs_dist={abs_dist:.4f}m")
        
        # Write to CSV
        row = {
            'omega_spine': f"{params['omega_spine']:.6f}",
            'phi_spine': f"{params['phi_spine']:.6f}",
            'bodyleg_coupling_factor': f"{params['bodyleg_coupling_factor']:.6f}",
            'phi_bodyleg': f"{params['phi_bodyleg']:.6f}",
            'A_spine_y': f"{params['A_spine_y']:.6f}",
            'A_shoulder': f"{params['A_shoulder']:.6f}",
            'dutyfactor': f"{params['dutyfactor']:.6f}",
            'travel_x': f"{travel_x:.6f}",
            'travel_y': f"{travel_y:.6f}",
            'abs_dist': f"{abs_dist:.6f}",
            'sim_time': f"{sim_time:.3f}"
        }
        writer.writerow(row)
        csvfile.flush()  # Flush to disk after each row

print("\n" + "=" * 80)
print(f"PARAMETER SWEEP COMPLETE")
print(f"Results saved to: {csv_filename}")
print("=" * 80)