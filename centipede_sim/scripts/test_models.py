"""
Multi-Model Locomotion Test for N-Segment Centipede Robots
===========================================================
Loads each generated model, applies the CPG gait controller (scaled to
N segments), runs simulation on flat ground, and outputs:
  - Sensor data  → output/sensors/<model_name>_sensors.npz
  - Video        → output/videos/<model_name>.mp4
  - Summary CSV  → output/test_summary.csv

Usage:
    python scripts/test_models.py                        # headless, all models
    python scripts/test_models.py --viewer 5             # viewer mode, 5-seg only
    python scripts/test_models.py --no-video             # skip video recording
    python scripts/test_models.py --segments 5 8 15      # specific models only
    python scripts/test_models.py --cycles 12            # more gait cycles

Output directory: centipede_sim/output/
"""

import argparse
import csv
import sys
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR = PROJECT_ROOT / "model"
OUTPUT_DIR = PROJECT_ROOT / "output"
SENSOR_DIR = OUTPUT_DIR / "sensors"
VIDEO_DIR = OUTPUT_DIR / "videos"

# Canonical lateral body-yaw half-amplitude for every segment-count model.
# Keep this fixed in reliability/noise experiments so morphology and channel
# statistics are the only intended independent variables.
LOCKED_YAW_HALF_AMPLITUDE_DEG = 15.0


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────
def deg2rad(d):
    return d * np.pi / 180


def leg_angle(phase, A_leg, leg_offset, dutyfactor):
    A_leg_up = -A_leg
    A_leg_down = -leg_offset
    duty = 1 - dutyfactor
    leg_threshold = np.cos(duty * np.pi)
    s = np.sin(phase - np.pi / 2)
    return A_leg_up if s > leg_threshold else A_leg_down


# ─────────────────────────────────────────────
# CPG controller (generalised to N segments)
# ─────────────────────────────────────────────
def build_actuator_indices(n_segments):
    """
    Build actuator index arrays for N-segment model.
    Per module: Shoulder_R, Leg_R, Shoulder_L, Leg_L, [Link_Y, Link_P]
    Last module has no link actuators.
    """
    SHOULDER_R, LEG_R, SHOULDER_L, LEG_L = [], [], [], []
    LINK_Y, LINK_P = [], []

    idx = 0
    for i in range(n_segments):
        SHOULDER_R.append(idx); idx += 1
        LEG_R.append(idx);      idx += 1
        SHOULDER_L.append(idx); idx += 1
        LEG_L.append(idx);      idx += 1
        if i < n_segments - 1:
            LINK_Y.append(idx); idx += 1
            LINK_P.append(idx); idx += 1

    return SHOULDER_R, LEG_R, SHOULDER_L, LEG_L, LINK_Y, LINK_P


def compute_phi_bodyleg(phi_spine, bodyleg_coupling_factor=0.5):
    """
    Derive the body-leg phase offset from the inter-segment phase and
    the coupling factor, matching the formula in param_sweep.py:
        phi_bodyleg = -pi/2 + (-phi_spine * coupling_factor)
    This ensures the legs are phased with the spine travelling wave.
    """
    return -np.pi / 2 + (-phi_spine * bodyleg_coupling_factor)


def compute_ctrl(t, n_ctrl, n_segments, indices,
                 omega_spine=4.0,
                 phi_spine=-np.pi,
                 phi_bodyleg=None,
                 bodyleg_coupling=0.5,
                 A_shoulder=None,
                 dutyfactor=0.5,
                 leg_offset=None,
                 A_leg=None):
    """
    Compute control vector for N-segment centipede CPG gait.

    The gait is driven by a single travelling sine wave at frequency
    omega_spine.  Each segment i is shifted by i * phi_spine:

      spine_phase_i = omega_spine * t + i * phi_spine

    Body undulation:
      LINK_Y[i] = A_spine_y * sin(spine_phase_i)
      This creates the lateral body wave that propels the centipede.

    Leg coordination:
      leg_phase_i = spine_phase_i + phi_bodyleg
      phi_bodyleg couples the leg swing to the body wave so the legs
      push when the body curve is favourable.
    """
    if A_shoulder is None:
        A_shoulder = deg2rad(30)
    if leg_offset is None:
        leg_offset = deg2rad(-5)
    if A_leg is None:
        A_leg = deg2rad(40)
    A_spine_y = deg2rad(LOCKED_YAW_HALF_AMPLITUDE_DEG)
    if phi_bodyleg is None:
        phi_bodyleg = compute_phi_bodyleg(phi_spine, bodyleg_coupling)

    SHOULDER_R, LEG_R, SHOULDER_L, LEG_L, LINK_Y, LINK_P = indices
    ctrl = np.zeros(n_ctrl)

    # ── Spine lateral undulation (travelling wave along LINK_Y joints) ──
    for i, idx in enumerate(LINK_Y):
        ctrl[idx] = A_spine_y * np.sin(omega_spine * t + i * phi_spine)

    # ── Spine vertical undulation (disabled — can be enabled later) ──
    for i, idx in enumerate(LINK_P):
        ctrl[idx] = 0

    # ── Shoulder swing (phased with body wave + phi_bodyleg offset) ──
    N = n_segments
    for i in range(N):
        phase = omega_spine * t + i * phi_spine + phi_bodyleg
        ctrl[SHOULDER_R[i]] =  leg_offset - A_shoulder * np.sin(phase)
        ctrl[SHOULDER_L[i]] = -leg_offset - A_shoulder * np.sin(phase)

    # ── Leg raise / plant (same phase as shoulder, R/L in anti-phase) ──
    for i in range(N):
        phase = omega_spine * t + i * phi_spine + phi_bodyleg
        ctrl[LEG_R[i]] = leg_angle(phase,         A_leg, leg_offset, dutyfactor)
        ctrl[LEG_L[i]] = leg_angle(phase + np.pi, A_leg, leg_offset, dutyfactor)

    return ctrl


# ─────────────────────────────────────────────
# Sensor data extraction helpers
# ─────────────────────────────────────────────
def get_sensor_map(model):
    """
    Build a dict mapping sensor name → (address, dimension).
    Also returns grouped sensor categories for structured output.
    """
    sensor_map = {}
    for i in range(model.nsensor):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SENSOR, i)
        adr = model.sensor_adr[i]
        dim = model.sensor_dim[i]
        sensor_map[name] = (adr, dim)
    return sensor_map


def extract_sensor_data(data, sensor_map):
    """Extract current sensor values into a flat dict."""
    row = {}
    for name, (adr, dim) in sensor_map.items():
        vals = data.sensordata[adr:adr + dim]
        if dim == 1:
            row[name] = vals[0]
        else:
            for j in range(dim):
                row[f"{name}_{j}"] = vals[j]
    return row


# ─────────────────────────────────────────────
# Video recorder (offscreen rendering)
# ─────────────────────────────────────────────
class VideoRecorder:
    """Records frames via MuJoCo offscreen renderer, writes MP4 via imageio."""

    def __init__(self, model, data, n_segments=4,
                 width=1280, height=720, fps=30):
        self.model = model
        self.data = data
        self.width = width
        self.height = height
        self.fps = fps
        self.frames = []

        # Ensure the offscreen framebuffer is large enough for our resolution
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
        model.vis.global_.offheight = max(model.vis.global_.offheight, height)

        # Create offscreen renderer
        self.renderer = mujoco.Renderer(model, height=height, width=width)

        # Camera setup: tracking the robot from an elevated 3/4 view
        # Scale distance based on robot length so the full body always
        # fits in frame, including outstretched legs at the tail.
        # Each segment is ~0.09m long, so total body ≈ n_seg * 0.09m
        body_length = n_segments * 0.09
        cam_dist = max(0.8, body_length * 2.0)   # 2× body length, min 0.8m

        self.camera = mujoco.MjvCamera()
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        self.camera.trackbodyid = 1  # track head chassis
        self.camera.distance = cam_dist
        self.camera.azimuth = 150     # slightly more top-down 3/4 view
        self.camera.elevation = -45   # steeper downward tilt to see full body
        self.camera.lookat[:] = [0, 0, 0.0]

        # Scene option flags — disable visual clutter
        self.scene_option = mujoco.MjvOption()
        # Turn off contact point visualisation
        self.scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        # Turn off contact force arrows
        self.scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False
        # Turn off site visualisation (the red balls from sensor sites)
        self.scene_option.sitegroup[:] = [0, 0, 0, 0, 0, 0]

    def capture_frame(self):
        """Render one frame and store it."""
        self.renderer.update_scene(self.data, camera=self.camera,
                                   scene_option=self.scene_option)
        pixels = self.renderer.render()
        self.frames.append(pixels.copy())

    def save(self, filepath):
        """Write all captured frames to MP4."""
        import imageio
        writer = imageio.get_writer(str(filepath), fps=self.fps,
                                    codec='libx264', quality=8,
                                    pixelformat='yuv420p')
        for frame in self.frames:
            writer.append_data(frame)
        writer.close()
        print(f"    Video saved: {filepath.name} "
              f"({len(self.frames)} frames, {len(self.frames)/self.fps:.1f}s)")

    def close(self):
        self.renderer.close()


# ─────────────────────────────────────────────
# Single model test
# ─────────────────────────────────────────────
def test_model(xml_path, n_segments, num_cycles=8,
               show_viewer=False, record_video=True,
               video_fps=30, video_width=1280, video_height=720,
               sensor_downsample=5):
    """
    Load model, run CPG gait, return results dict plus sensor data.

    Args:
        xml_path: Path to MuJoCo XML model file.
        n_segments: Number of body segments.
        num_cycles: Number of gait cycles to simulate.
        show_viewer: Launch interactive viewer (cannot coexist with video).
        record_video: Record offscreen MP4 video.
        video_fps: Target video frame rate.
        sensor_downsample: Record sensor data every N steps (1=every step).

    Returns:
        dict with 'result' (metrics), 'sensor_data' (dict of arrays),
        'sensor_names' (ordered list), 'recorder' (VideoRecorder or None).
    """
    # Load
    try:
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        data = mujoco.MjData(model)
    except Exception as e:
        return {"result": {"status": "LOAD_FAIL", "error": str(e)}}

    n_ctrl = model.nu
    expected_ctrl = n_segments * 4 + (n_segments - 1) * 2
    if n_ctrl != expected_ctrl:
        return {"result": {"status": "CTRL_MISMATCH",
                "error": f"expected {expected_ctrl} actuators, got {n_ctrl}"}}

    indices = build_actuator_indices(n_segments)

    # Gait parameters (matching param_sweep.py baseline with body undulation)
    omega_spine = 4.0
    phi_spine = -np.pi
    bodyleg_coupling = 0.5
    phi_bodyleg = compute_phi_bodyleg(phi_spine, bodyleg_coupling)

    dt = model.opt.timestep
    gait_period = 2 * np.pi / omega_spine
    sim_time = num_cycles * gait_period
    steps = int(sim_time / dt)

    # Sensor map
    sensor_map = get_sensor_map(model) if model.nsensor > 0 else {}

    # Pre-allocate sensor storage
    # We record every sensor_downsample steps
    n_records = steps // sensor_downsample + 1
    sensor_time = []  # timestamps
    sensor_rows = []  # list of dicts

    # We also record base pose (qpos[0:7]) and velocity (qvel[0:6])
    base_pose_log = []

    # Video recorder
    recorder = None
    frame_interval = max(1, int(1.0 / (video_fps * dt)))  # steps between frames
    if record_video and not show_viewer:
        try:
            import imageio
        except ImportError:
            print("    Warning: 'imageio' not installed. Run: pip install imageio imageio-ffmpeg")
            record_video = False
        if record_video:
            try:
                recorder = VideoRecorder(model, data,
                                         n_segments=n_segments,
                                         width=video_width, height=video_height,
                                         fps=video_fps)
            except Exception as e:
                print(f"    Warning: video recording unavailable — {e}")
                print(f"    (Ensure you have a working OpenGL driver. "
                      f"Try: pip install imageio imageio-ffmpeg)")
                recorder = None

    # Viewer
    viewer = None
    if show_viewer:
        viewer = mujoco.viewer.launch_passive(model, data)

    # ─── Ramp-up phase ───
    target = compute_ctrl(0.0, n_ctrl, n_segments, indices,
                          omega_spine=omega_spine, phi_spine=phi_spine,
                          phi_bodyleg=phi_bodyleg)
    init_steps = int(1.5 / dt)
    for k in range(init_steps):
        data.ctrl[:] = (k / init_steps) * target
        mujoco.mj_step(model, data)
        if viewer:
            viewer.sync()
            if not viewer.is_running():
                viewer.close()
                return {"result": {"status": "VIEWER_CLOSED"}}
        if recorder and k % frame_interval == 0:
            recorder.capture_frame()

    start_pos = data.qpos[0:3].copy()
    start_z = start_pos[2]

    # Track z-height
    z_min = start_z
    z_history = []

    # ─── Main locomotion loop ───
    try:
        for step in range(steps):
            t = step * dt
            data.ctrl[:] = compute_ctrl(t, n_ctrl, n_segments, indices,
                                        omega_spine=omega_spine,
                                        phi_spine=phi_spine,
                                        phi_bodyleg=phi_bodyleg)
            mujoco.mj_step(model, data)

            # Z tracking
            if step % 100 == 0:
                z_history.append(data.qpos[2])
                z_min = min(z_min, data.qpos[2])

            # Sensor recording
            if step % sensor_downsample == 0:
                sensor_time.append(data.time)
                if sensor_map:
                    sensor_rows.append(extract_sensor_data(data, sensor_map))
                base_pose_log.append(data.qpos[0:7].copy())

            # Video frame
            if recorder and step % frame_interval == 0:
                recorder.capture_frame()

            # Viewer sync
            if viewer:
                viewer.sync()
                if not viewer.is_running():
                    viewer.close()
                    return {"result": {"status": "VIEWER_CLOSED"}}

    except KeyboardInterrupt:
        if viewer:
            viewer.close()
        if recorder:
            recorder.close()
        return {"result": {"status": "INTERRUPTED"}}

    if viewer:
        viewer.close()

    # ─── Compile results ───
    final_pos = data.qpos[0:3].copy()
    travel_x = final_pos[0] - start_pos[0]
    travel_y = final_pos[1] - start_pos[1]
    dist = np.sqrt(travel_x**2 + travel_y**2)
    z_final = final_pos[2]
    collapsed = z_min < start_z * 0.5

    result = {
        "status": "OK",
        "travel_x": travel_x,
        "travel_y": travel_y,
        "abs_dist": dist,
        "z_start": start_z,
        "z_final": z_final,
        "z_min": z_min,
        "collapsed": collapsed,
        "sim_time": sim_time,
        "n_actuators": n_ctrl,
        "n_sensors": model.nsensor,
        "n_records": len(sensor_time),
    }

    # Build structured sensor arrays
    sensor_data = {
        "time": np.array(sensor_time),
        "base_pose": np.array(base_pose_log),  # (N, 7): x,y,z,qw,qx,qy,qz
    }

    # Convert sensor_rows (list of dicts) → dict of arrays
    if sensor_rows:
        all_keys = list(sensor_rows[0].keys())
        for key in all_keys:
            sensor_data[key] = np.array([row[key] for row in sensor_rows])

    return {
        "result": result,
        "sensor_data": sensor_data,
        "recorder": recorder,
    }


# ─────────────────────────────────────────────
# Save sensor data to .npz
# ─────────────────────────────────────────────
def save_sensor_data(sensor_data, filepath, n_segments):
    """
    Save all sensor data to a compressed .npz file.
    Structure:
      - time: (N,)         simulation timestamps
      - base_pose: (N,7)   root body pose [x,y,z,qw,qx,qy,qz]
      - imu_acc_*: (N,)    accelerometer channels
      - imu_gyro_*: (N,)   gyroscope channels
      - pos_*: (N,)        body position channels
      - quat_*: (N,)       body quaternion channels
      - jpos_*: (N,)       joint position sensors
      - jvel_*: (N,)       joint velocity sensors
      - afrc_*: (N,)       actuator force sensors
      - touch_*: (N,)      foot contact sensors
    """
    np.savez_compressed(str(filepath),
                        n_segments=n_segments,
                        **sensor_data)
    size_mb = filepath.stat().st_size / (1024 * 1024)
    n_records = len(sensor_data.get("time", []))
    n_channels = len(sensor_data) - 1  # exclude 'time'
    print(f"    Sensors saved: {filepath.name} "
          f"({n_records} samples, {n_channels} channels, {size_mb:.2f} MB)")


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Test centipede models — outputs sensor data + video",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scripts/test_models.py                         # all models, headless
  python scripts/test_models.py --viewer 5              # interactive viewer, 5-seg
  python scripts/test_models.py --no-video              # sensor data only
  python scripts/test_models.py --segments 5 8 15       # specific models
  python scripts/test_models.py --cycles 12 --fps 60    # longer sim, 60fps video
        """)
    parser.add_argument("--viewer", type=int, default=0,
                        help="Show viewer for this segment count (0=headless)")
    parser.add_argument("--segments", type=int, nargs="*",
                        default=[3, 4, 5, 6, 7, 8, 10, 15],
                        help="Segment counts to test (default: 3 4 5 6 7 8 10 15)")
    parser.add_argument("--cycles", type=int, default=8,
                        help="Gait cycles per test (default: 8)")
    parser.add_argument("--no-video", action="store_true",
                        help="Skip video recording")
    parser.add_argument("--fps", type=int, default=30,
                        help="Video frame rate (default: 30)")
    parser.add_argument("--sensor-rate", type=int, default=5,
                        help="Record sensors every N sim steps (default: 5)")
    parser.add_argument("--width", type=int, default=1280,
                        help="Video width in pixels (default: 1280)")
    parser.add_argument("--height", type=int, default=720,
                        help="Video height in pixels (default: 720)")
    args = parser.parse_args()

    # Create output directories
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    SENSOR_DIR.mkdir(parents=True, exist_ok=True)
    VIDEO_DIR.mkdir(parents=True, exist_ok=True)

    all_segments = sorted(set(args.segments))

    print("=" * 80)
    print("CENTIPEDE MULTI-MODEL LOCOMOTION TEST")
    print("=" * 80)
    print(f"  Models:       {all_segments}")
    print(f"  Cycles:       {args.cycles}")
    print(f"  Viewer:       {args.viewer if args.viewer else 'off'}")
    print(f"  Video:        {'off' if args.no_video else f'{args.width}x{args.height} @ {args.fps}fps'}")
    print(f"  Sensor rate:  every {args.sensor_rate} steps")
    print(f"  Output dir:   {OUTPUT_DIR}")
    print("=" * 80)

    results = {}
    for n_seg in all_segments:
        xml_path = MODEL_DIR / f"centipede_{n_seg}seg.xml"

        if not xml_path.exists():
            print(f"\n  [{n_seg:2d} seg]  SKIP — {xml_path.name} not found")
            continue

        show = (args.viewer == n_seg)
        record = (not args.no_video) and (not show)
        model_name = xml_path.stem

        print(f"\n  [{n_seg:2d} seg]  Testing {xml_path.name} ...")
        t_start = time.time()

        out = test_model(xml_path, n_seg,
                         num_cycles=args.cycles,
                         show_viewer=show,
                         record_video=record,
                         video_fps=args.fps,
                         video_width=args.width,
                         video_height=args.height,
                         sensor_downsample=args.sensor_rate)

        result = out["result"]
        elapsed = time.time() - t_start
        results[n_seg] = result

        if result["status"] != "OK":
            print(f"           FAIL: {result['status']} — {result.get('error', '')}")
            continue

        status = "COLLAPSED" if result["collapsed"] else "STABLE"
        print(f"           {status}  "
              f"dist={result['abs_dist']:.4f}m  "
              f"x={result['travel_x']:.4f}m  "
              f"y={result['travel_y']:.4f}m  "
              f"z: {result['z_start']:.3f}->{result['z_final']:.3f} "
              f"(min={result['z_min']:.3f})  "
              f"[{elapsed:.1f}s]")

        # Save sensor data → output/sensors/
        sensor_data = out.get("sensor_data")
        if sensor_data and len(sensor_data.get("time", [])) > 0:
            sensor_path = SENSOR_DIR / f"{model_name}_sensors.npz"
            save_sensor_data(sensor_data, sensor_path, n_seg)

        # Save video → output/videos/
        recorder = out.get("recorder")
        if recorder and recorder.frames:
            video_path = VIDEO_DIR / f"{model_name}.mp4"
            try:
                recorder.save(video_path)
            except Exception as e:
                print(f"    Video error: {e}")
            finally:
                recorder.close()

    # ─── Summary table ───
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"{'Seg':>4} {'Status':>10} {'Dist':>8} {'X':>8} {'Y':>8} "
          f"{'Z-fin':>7} {'Z-min':>7} {'Act':>5} {'Sens':>5} {'Samples':>8}")
    print("-" * 80)

    for n_seg in all_segments:
        r = results.get(n_seg)
        if r is None:
            print(f"{n_seg:>4}   {'SKIP':>10}")
            continue
        if r["status"] != "OK":
            print(f"{n_seg:>4}   {r['status']:>10}")
            continue

        tag = "COLLAPSE" if r["collapsed"] else "OK"
        print(f"{n_seg:>4} {tag:>10} {r['abs_dist']:>8.4f} {r['travel_x']:>8.4f} "
              f"{r['travel_y']:>8.4f} {r['z_final']:>7.3f} {r['z_min']:>7.3f} "
              f"{r['n_actuators']:>5} {r['n_sensors']:>5} {r['n_records']:>8}")

    # ─── Summary CSV ───
    csv_path = OUTPUT_DIR / "test_summary.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["segments", "status", "distance_m", "travel_x_m",
                         "travel_y_m", "z_start_m", "z_final_m", "z_min_m",
                         "collapsed", "sim_time_s", "n_actuators", "n_sensors",
                         "n_records"])
        for n_seg in all_segments:
            r = results.get(n_seg)
            if r is None or r["status"] != "OK":
                continue
            writer.writerow([
                n_seg, r["status"], f"{r['abs_dist']:.6f}",
                f"{r['travel_x']:.6f}", f"{r['travel_y']:.6f}",
                f"{r['z_start']:.6f}", f"{r['z_final']:.6f}",
                f"{r['z_min']:.6f}", r["collapsed"], f"{r['sim_time']:.4f}",
                r["n_actuators"], r["n_sensors"], r["n_records"],
            ])
    print(f"\n  Summary CSV: {csv_path}")

    print("=" * 80)
    print(f"  Output directory: {OUTPUT_DIR}/")
    print(f"    Sensors: {SENSOR_DIR}/")
    print(f"    Videos:  {VIDEO_DIR}/")
    print("=" * 80)
