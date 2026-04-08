"""
Parametric MuJoCo Model Generator for N-Segment Centipede Robot
================================================================
Generates centipede_Nseg.xml models with:
  - N chassis segments, each with 2 legs (L/R)
  - (N-1) inter-segment 2-DOF links (yaw + pitch)
  - Full sensor suite (joint pos/vel, IMU, body tracking, foot contact)
  - Flat ground for locomotion testing

All geometry, inertia, and actuator definitions are identical to the
validated 4-segment model (SolidWorks CAD ground truth).

Usage:
    python scripts/generate_models.py
"""

from pathlib import Path
import textwrap

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = PROJECT_ROOT / "model"


# ═══════════════════════════════════════════════════════════════════════
# Geometry / inertia constants (from SolidWorks CAD export)
# ═══════════════════════════════════════════════════════════════════════

# --- Chassis ---
CHASIS1_INERTIAL = 'pos="-3.493e-09 -0.023000 0.015312" mass="0.08226" diaginertia="5.225439e-05 9.491986e-05 4.634139e-05"'
CHASIS2_INERTIAL = 'pos="-3.499e-09 -3.896e-09 0.046308" mass="0.08226" diaginertia="5.225439e-05 9.491986e-05 4.634139e-05"'

# --- Shoulder motors (same for L and R, all modules) ---
SHOULDER_INERTIAL = 'pos="-0.012002 -0.020404 0.000443" mass="0.05692" diaginertia="1.023035e-05 1.439249e-05 1.439243e-05"'

# --- Legs ---
LEG_R_INERTIAL = 'pos="-0.001850 -0.004012 0.052394" mass="0.02994" fullinertia="2.066460e-05 1.494986e-05 1.495677e-05 -9.058248e-10 -1.249030e-06 3.628917e-06"'
LEG_L_INERTIAL = 'pos="0.001835 0.004012 -0.052394" mass="0.02994" fullinertia="2.066461e-05 1.494987e-05 1.495677e-05 9.001094e-10 -1.259626e-06 3.628917e-06"'

# --- Link motors ---
LINK_PNY_INERTIAL_MOD1 = 'pos="-0.012002 -0.020404 0.000443" mass="0.05692" diaginertia="1.023035e-05 1.439249e-05 1.439243e-05"'
LINK_PNY_INERTIAL_MOD2P = 'pos="-0.012002 0.000496 0.000443" mass="0.05692" diaginertia="1.023035e-05 1.439249e-05 1.439243e-05"'
LINK_R_INERTIAL = 'pos="-8.115e-06 0.008650 0.033659" mass="0.05923" diaginertia="1.491979e-05 1.648496e-05 1.708782e-05"'

# --- Placement constants ---
# Module 1 shoulder placement (relative to Chasis1)
MOD1_SHOULDER_R_POS = '0.029 0.004996 -0.0021'
MOD1_SHOULDER_R_QUAT = '0.00155303 0.00155304 -0.707106 -0.707104'
MOD1_SHOULDER_L_POS = '-0.029 0.004996 -0.0021'
MOD1_SHOULDER_L_QUAT = '0.707104 0.707106 0.00154994 0.00154994'
MOD1_LINK_PNY_POS = '0 -0.046004 -0.0021'
MOD1_LINK_PNY_QUAT = '0.498897 0.498899 0.501101 0.501099'

# Module 2+ shoulder placement (relative to Chasis N)
MOD2_SHOULDER_R_POS = '-0.029 -0.0209 0.026'
MOD2_SHOULDER_R_QUAT = '0 1 0 0'
MOD2_SHOULDER_L_POS = '0.029 -0.0209 0.026'
MOD2_SHOULDER_L_QUAT = '0 0 0 -1'
MOD2_LINK_PNY_POS = '0 -0.0209 0.077'
MOD2_LINK_PNY_QUAT = '-2.60305e-06 0.708659 -2.59163e-06 -0.705551'

# Leg placement (same for all modules, relative to shoulder)
LEG_R_POS = '-0.024 -0.0209 -0.00010521'
LEG_R_QUAT = '0.705551 1.14188e-08 -0.708659 -1.14188e-08'
LEG_L_POS = '-0.024 -0.0209 -0.00010521'
LEG_L_QUAT = '-2.60305e-06 0.705551 2.59163e-06 0.708659'

# Link motor R placement (relative to PnY)
LINK_R_POS = '-0.024 -0.0209 -0.00010521'
LINK_R_QUAT = '0.705551 1.14188e-08 -0.708659 -1.14188e-08'

# Next chassis placement (relative to LinkMotor R)
NEXT_CHASIS_POS = '0 0 0.059'
NEXT_CHASIS_QUAT = '-3.67321e-06 -0 0 1'

# Colors
COLOR_CHASIS1 = '1 0.596078431372549 0.137254901960784 1'
COLOR_BODY = '0.792156862745098 0.819607843137255 0.933333333333333 1'
COLOR_LEG = '1 1 1 1'


def indent(text, level):
    """Indent text by `level` * 2 spaces."""
    prefix = "  " * level
    return "\n".join(prefix + line if line.strip() else "" for line in text.split("\n"))


def gen_leg_pair(mod_idx, base_indent, is_head=False):
    """Generate shoulder + leg bodies for module `mod_idx` (1-indexed)."""
    i = mod_idx
    d = base_indent

    sh_r_pos = MOD1_SHOULDER_R_POS if is_head else MOD2_SHOULDER_R_POS
    sh_r_quat = MOD1_SHOULDER_R_QUAT if is_head else MOD2_SHOULDER_R_QUAT
    sh_l_pos = MOD1_SHOULDER_L_POS if is_head else MOD2_SHOULDER_L_POS
    sh_l_quat = MOD1_SHOULDER_L_QUAT if is_head else MOD2_SHOULDER_L_QUAT

    return f"""\
{d}<body name="ShoulderMotor{i}_R" pos="{sh_r_pos}" quat="{sh_r_quat}">
{d}  <joint name="Shoulder{i}_R" pos="0 0 0" axis="0 1 0" range="-1.5 1.5" actuatorfrcrange="-1.5 1.5"/>
{d}  <inertial {SHOULDER_INERTIAL}/>
{d}  <geom type="mesh" rgba="{COLOR_BODY}" mesh="ShoulderMotor{i}_R" contype="2" conaffinity="0"/>
{d}  <site name="foot{i}_R" pos="0 0 0.085" size="0.005" rgba="1 0 0 0.5"/>
{d}  <body name="Leg{i}_R" pos="{LEG_R_POS}" quat="{LEG_R_QUAT}">
{d}    <joint name="Leg{i}_R" pos="0 0 0" axis="1 0 0" range="-1.5 1.5" actuatorfrcrange="-1.5 1.5"/>
{d}    <inertial {LEG_R_INERTIAL}/>
{d}    <geom type="mesh" rgba="{COLOR_LEG}" mesh="Leg{i}_R" contype="4" conaffinity="5"/>
{d}  </body>
{d}</body>
{d}<body name="ShoulderMotor{i}_L" pos="{sh_l_pos}" quat="{sh_l_quat}">
{d}  <joint name="Shoulder{i}_L" pos="0 0 0" axis="0 1 0" range="-1.5 1.5" actuatorfrcrange="-1.5 1.5"/>
{d}  <inertial {SHOULDER_INERTIAL}/>
{d}  <geom type="mesh" rgba="{COLOR_BODY}" mesh="ShoulderMotor{i}_L" contype="2" conaffinity="0"/>
{d}  <site name="foot{i}_L" pos="0 0 0.085" size="0.005" rgba="1 0 0 0.5"/>
{d}  <body name="Leg{i}_L" pos="{LEG_L_POS}" quat="{LEG_L_QUAT}">
{d}    <joint name="Leg{i}_L" pos="0 0 0" axis="1 0 0" range="-1.5 1.5" actuatorfrcrange="-1.5 1.5"/>
{d}    <inertial {LEG_L_INERTIAL}/>
{d}    <geom type="mesh" rgba="{COLOR_LEG}" mesh="Leg{i}_L" contype="4" conaffinity="5"/>
{d}  </body>
{d}</body>"""


def generate_model(n_segments, output_path=None):
    """
    Generate a MuJoCo XML for an N-segment centipede robot.

    Parameters
    ----------
    n_segments : int
        Number of body segments (each with 2 legs).
    output_path : Path, optional
        Write destination. Defaults to model/centipede_{n}seg.xml
    """
    N = n_segments
    if output_path is None:
        output_path = MODEL_DIR / f"centipede_{N}seg.xml"

    # ── Asset section: mesh declarations ──
    mesh_lines = []
    # Chassis meshes
    mesh_lines.append(f'    <mesh name="Chasis1" content_type="model/stl" file="Chasis1.STL"/>')
    for i in range(2, N + 1):
        mesh_lines.append(f'    <mesh name="Chasis{i}" content_type="model/stl" file="Chasis2.STL"/>')

    # Per-module meshes (all reuse module-1 geometry)
    for i in range(1, N + 1):
        mesh_lines.append(f'    <!-- Module {i} -->')
        mesh_lines.append(f'    <mesh name="ShoulderMotor{i}_R" content_type="model/stl" file="ShoulderMotor1_R.STL"/>')
        mesh_lines.append(f'    <mesh name="Leg{i}_R" content_type="model/stl" file="Leg1_R.STL"/>')
        mesh_lines.append(f'    <mesh name="ShoulderMotor{i}_L" content_type="model/stl" file="ShoulderMotor1_L.STL"/>')
        mesh_lines.append(f'    <mesh name="Leg{i}_L" content_type="model/stl" file="Leg1_L.STL"/>')
        if i < N:
            mesh_lines.append(f'    <mesh name="LinkMotor{i}_PnY" content_type="model/stl" file="LinkMotor1_PnY.STL"/>')
            mesh_lines.append(f'    <mesh name="LinkMotor{i}_R" content_type="model/stl" file="LinkMotor1_R.STL"/>')

    meshes_block = "\n".join(mesh_lines)

    # ── Worldbody: build kinematic chain recursively ──
    # We build the body tree as nested strings
    # The nesting depth increases with each link

    body_lines = []

    def add_module(mod_idx, depth):
        """Add module `mod_idx` (1-indexed) at indentation `depth`."""
        d = "  " * depth
        is_head = (mod_idx == 1)
        is_tail = (mod_idx == N)

        if is_head:
            # Head module: robot_root with freejoint + Chasis1
            body_lines.append(f'{d}<body name="robot_root" pos="0 0 0.15">')
            body_lines.append(f'{d}  <freejoint name="root"/>')
            body_lines.append(f'{d}  <!-- ============ MODULE {mod_idx}: Chasis{mod_idx} (HEAD) ============ -->')
            body_lines.append(f'{d}  <inertial {CHASIS1_INERTIAL}/>')
            body_lines.append(f'{d}  <geom type="mesh" rgba="{COLOR_CHASIS1}" mesh="Chasis1" contype="2" conaffinity="0"/>')
            body_lines.append(f'{d}  <site name="imu_site" pos="0 0 0.015" size="0.003"/>')
            body_lines.append(f'{d}  <site name="chasis{mod_idx}_site" pos="0 0 0" size="0.003"/>')

            # Legs
            legs = gen_leg_pair(mod_idx, "  " * (depth + 1), is_head=True)
            body_lines.append(legs)

        else:
            # Interior / tail module: Chasis body
            body_lines.append(f'{d}<!-- ============ MODULE {mod_idx}: Chasis{mod_idx}{" (TAIL)" if is_tail else ""} ============ -->')
            body_lines.append(f'{d}<body name="Chasis{mod_idx}" pos="{NEXT_CHASIS_POS}" quat="{NEXT_CHASIS_QUAT}">')
            body_lines.append(f'{d}  <inertial {CHASIS2_INERTIAL}/>')
            body_lines.append(f'{d}  <geom type="mesh" rgba="{COLOR_BODY}" mesh="Chasis{mod_idx}" contype="2" conaffinity="0"/>')
            body_lines.append(f'{d}  <site name="chasis{mod_idx}_site" pos="0 0 0.046" size="0.003"/>')

            # Legs
            legs = gen_leg_pair(mod_idx, "  " * (depth + 1), is_head=False)
            body_lines.append(legs)

        # Inter-segment link (if not tail)
        if not is_tail:
            ld = "  " * (depth + 1)
            link_pny_pos = MOD1_LINK_PNY_POS if is_head else MOD2_LINK_PNY_POS
            link_pny_quat = MOD1_LINK_PNY_QUAT if is_head else MOD2_LINK_PNY_QUAT
            link_pny_inertial = LINK_PNY_INERTIAL_MOD1 if is_head else LINK_PNY_INERTIAL_MOD2P

            body_lines.append(f'{ld}<!-- Link {mod_idx}: connecting Chasis{mod_idx} to Chasis{mod_idx+1} -->')
            body_lines.append(f'{ld}<body name="LinkMotor{mod_idx}_PnY" pos="{link_pny_pos}" quat="{link_pny_quat}">')
            body_lines.append(f'{ld}  <joint name="Link{mod_idx}_Y" pos="0 0 0" axis="0 1 0" range="-1.5 1.5" actuatorfrcrange="-1.5 1.5"/>')
            body_lines.append(f'{ld}  <inertial {link_pny_inertial}/>')
            body_lines.append(f'{ld}  <geom type="mesh" rgba="{COLOR_BODY}" mesh="LinkMotor{mod_idx}_PnY" contype="2" conaffinity="0"/>')
            body_lines.append(f'{ld}  <body name="LinkMotor{mod_idx}_R" pos="{LINK_R_POS}" quat="{LINK_R_QUAT}">')
            body_lines.append(f'{ld}    <joint name="Link{mod_idx}_P" pos="0 0 0" axis="1 0 0" range="-1.5 1.5" actuatorfrcrange="-1.5 1.5"/>')
            body_lines.append(f'{ld}    <inertial {LINK_R_INERTIAL}/>')
            body_lines.append(f'{ld}    <geom type="mesh" rgba="{COLOR_BODY}" mesh="LinkMotor{mod_idx}_R" contype="2" conaffinity="0"/>')

            # Recurse into next module
            add_module(mod_idx + 1, depth + 3)

            # Close LinkMotor_R and LinkMotor_PnY
            body_lines.append(f'{ld}  </body>')
            body_lines.append(f'{ld}</body>')

        # Close this chassis body (head closes robot_root)
        if is_head:
            body_lines.append(f'{d}</body> <!-- end robot_root -->')
        elif not is_tail:
            body_lines.append(f'{d}</body>')
        else:
            body_lines.append(f'{d}</body>')

    add_module(1, depth=2)
    worldbody_content = "\n".join(body_lines)

    # ── Actuators ──
    act_lines = []
    for i in range(1, N + 1):
        act_lines.append(f'    <!-- Module {i} -->')
        act_lines.append(f'    <position name="act_Shoulder{i}_R" joint="Shoulder{i}_R" kp="0.5" ctrllimited="true" ctrlrange="-0.57 0.8"/>')
        act_lines.append(f'    <position name="act_Leg{i}_R"      joint="Leg{i}_R"      kp="2"   ctrllimited="true" ctrlrange="-1.5 1.5"/>')
        act_lines.append(f'    <position name="act_Shoulder{i}_L" joint="Shoulder{i}_L" kp="0.5" ctrllimited="true" ctrlrange="-0.8 0.57"/>')
        act_lines.append(f'    <position name="act_Leg{i}_L"      joint="Leg{i}_L"      kp="2"   ctrllimited="true" ctrlrange="-1.5 1.5"/>')
        if i < N:
            act_lines.append(f'    <!-- Link {i} -->')
            act_lines.append(f'    <position name="act_Link{i}_Y" joint="Link{i}_Y" kp="200" ctrllimited="true" ctrlrange="-0.7 0.7"/>')
            act_lines.append(f'    <position name="act_Link{i}_P" joint="Link{i}_P" kp="10"  ctrllimited="true" ctrlrange="-1.5 1.5"/>')
    actuators_block = "\n".join(act_lines)

    # ── Sensors ──
    sensor_lines = []
    sensor_lines.append('    <!-- IMU on head chassis -->')
    sensor_lines.append('    <accelerometer name="imu_acc" site="imu_site"/>')
    sensor_lines.append('    <gyro name="imu_gyro" site="imu_site"/>')
    sensor_lines.append('')
    sensor_lines.append('    <!-- Chassis body tracking (position + orientation) -->')
    sensor_lines.append('    <framepos name="pos_head" objtype="body" objname="robot_root"/>')
    sensor_lines.append('    <framequat name="quat_head" objtype="body" objname="robot_root"/>')
    for i in range(2, N + 1):
        sensor_lines.append(f'    <framepos name="pos_chasis{i}" objtype="body" objname="Chasis{i}"/>')
        sensor_lines.append(f'    <framequat name="quat_chasis{i}" objtype="body" objname="Chasis{i}"/>')

    sensor_lines.append('')
    sensor_lines.append('    <!-- Joint position sensors -->')
    for i in range(1, N + 1):
        sensor_lines.append(f'    <jointpos name="jpos_Sh{i}R" joint="Shoulder{i}_R"/>')
        sensor_lines.append(f'    <jointpos name="jpos_Lg{i}R" joint="Leg{i}_R"/>')
        sensor_lines.append(f'    <jointpos name="jpos_Sh{i}L" joint="Shoulder{i}_L"/>')
        sensor_lines.append(f'    <jointpos name="jpos_Lg{i}L" joint="Leg{i}_L"/>')
        if i < N:
            sensor_lines.append(f'    <jointpos name="jpos_Lk{i}Y" joint="Link{i}_Y"/>')
            sensor_lines.append(f'    <jointpos name="jpos_Lk{i}P" joint="Link{i}_P"/>')

    sensor_lines.append('')
    sensor_lines.append('    <!-- Joint velocity sensors -->')
    for i in range(1, N + 1):
        sensor_lines.append(f'    <jointvel name="jvel_Sh{i}R" joint="Shoulder{i}_R"/>')
        sensor_lines.append(f'    <jointvel name="jvel_Lg{i}R" joint="Leg{i}_R"/>')
        sensor_lines.append(f'    <jointvel name="jvel_Sh{i}L" joint="Shoulder{i}_L"/>')
        sensor_lines.append(f'    <jointvel name="jvel_Lg{i}L" joint="Leg{i}_L"/>')
        if i < N:
            sensor_lines.append(f'    <jointvel name="jvel_Lk{i}Y" joint="Link{i}_Y"/>')
            sensor_lines.append(f'    <jointvel name="jvel_Lk{i}P" joint="Link{i}_P"/>')

    sensor_lines.append('')
    sensor_lines.append('    <!-- Actuator force sensors -->')
    for i in range(1, N + 1):
        sensor_lines.append(f'    <actuatorfrc name="afrc_Sh{i}R" actuator="act_Shoulder{i}_R"/>')
        sensor_lines.append(f'    <actuatorfrc name="afrc_Lg{i}R" actuator="act_Leg{i}_R"/>')
        sensor_lines.append(f'    <actuatorfrc name="afrc_Sh{i}L" actuator="act_Shoulder{i}_L"/>')
        sensor_lines.append(f'    <actuatorfrc name="afrc_Lg{i}L" actuator="act_Leg{i}_L"/>')
        if i < N:
            sensor_lines.append(f'    <actuatorfrc name="afrc_Lk{i}Y" actuator="act_Link{i}_Y"/>')
            sensor_lines.append(f'    <actuatorfrc name="afrc_Lk{i}P" actuator="act_Link{i}_P"/>')

    sensor_lines.append('')
    sensor_lines.append('    <!-- Foot contact (touch) sensors -->')
    for i in range(1, N + 1):
        sensor_lines.append(f'    <touch name="touch_{i}R" site="foot{i}_R"/>')
        sensor_lines.append(f'    <touch name="touch_{i}L" site="foot{i}_L"/>')

    sensors_block = "\n".join(sensor_lines)

    # ── Assemble full XML ──
    xml = f"""\
<?xml version="1.0" ?>
<!-- Auto-generated {N}-segment centipede model -->
<mujoco model="centipede_{N}seg">
  <compiler angle="radian" meshdir="../meshes"/>
  <visual>
    <headlight ambient="0.4 0.4 0.4" diffuse="0.8 0.8 0.8" specular="0.1 0.1 0.1"/>
    <rgba haze="0.15 0.25 0.35 1"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.2 0.2 0.2" rgb2="0.9 0.9 0.9"/>
    <material name="grid_mat" texture="grid" texrepeat="8 8" reflectance="0.2"/>
{meshes_block}
  </asset>
  <default>
    <joint damping="0.5" armature="0.05"/>
    <geom condim="4" friction="0.4 0.005 0.0001" solref="0.02 1" solimp="0.98 0.98 0.001"/>
  </default>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1" material="grid_mat" pos="0 0 0"
          contype="1" conaffinity="4"/>
    <light pos="0 0 3" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
{worldbody_content}
  </worldbody>
  <actuator>
{actuators_block}
  </actuator>
  <sensor>
{sensors_block}
  </sensor>
</mujoco>
"""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(xml)

    n_joints = N * 4 + (N - 1) * 2
    n_actuators = n_joints
    total_mass_g = N * 82.3 + N * 2 * (56.9 + 29.9) + (N - 1) * 2 * (56.9 + 59.2)
    print(f"  [{N:2d} seg]  {output_path.name:30s}  "
          f"joints={n_joints:3d}  actuators={n_actuators:3d}  "
          f"mass~{total_mass_g/1000:.2f}kg  bodies={N*5 + (N-1)*2}")

    return output_path


# ═══════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 80)
    print("Generating centipede models...")
    print("=" * 80)

    segment_counts = [5, 6, 7, 8, 15]
    paths = []
    for n in segment_counts:
        p = generate_model(n)
        paths.append(p)

    print("\n" + "=" * 80)
    print(f"Generated {len(paths)} models in {MODEL_DIR}")
    print("=" * 80)
