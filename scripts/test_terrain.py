#!/usr/bin/env python3
"""
test_terrain.py — Rough terrain locomotion benchmark
=====================================================
Tests all centipede models (4–15 segments) on generated rough terrains.
Each (model × terrain_seed) combination is run once. Multiple seeds
provide the statistical replicates needed for error bars.

Metrics (per trial):
  - distance:        Euclidean distance traveled (m)
  - travel_x:        Forward distance (m)
  - mean_pitch_dev:  Mean absolute body pitch deviation from horizontal (deg)
  - mean_roll_dev:   Mean absolute body roll deviation from horizontal (deg)
  - z_stability:     Std of body z-height over time (lower = more stable)
  - collapsed:       True if body dropped below 50% of start height
  - composite_score: distance × (1 / (1 + orientation_penalty))

Output:
  - Per-trial results CSV with all metrics
  - Summary CSV with mean ± std per model per terrain difficulty
  - Videos for each trial

Usage:
    python scripts/test_terrain.py                                  # default
    python scripts/test_terrain.py --base 0.5 --levels 3            # specific terrain
    python scripts/test_terrain.py --segments 4 8 15 --n-seeds 5    # subset of models
    python scripts/test_terrain.py --no-video                       # faster, no video

Prerequisite: run generate_terrain.py first to create the heightfield PNGs.
"""

import argparse
import csv
import json
import re
import sys
import time
from pathlib import Path

import mujoco
import numpy as np

# ── Project paths ──
PROJECT_ROOT = Path(__file__).resolve().parent.parent
MODEL_DIR    = PROJECT_ROOT / "model"
TERRAIN_DIR  = PROJECT_ROOT / "terrain"
OUTPUT_DIR   = PROJECT_ROOT / "output" / "terrain_tests"

sys.path.insert(0, str(PROJECT_ROOT / "scripts"))
from test_models import (
    build_actuator_indices, compute_ctrl, compute_phi_bodyleg,
    VideoRecorder, get_sensor_map, extract_sensor_data,
    deg2rad,
)

# ─────────────────────────────────────────────
# Unit conversions
# ─────────────────────────────────────────────
INCH_TO_M = 0.0254


def quat_to_euler(quat):
    """
    Convert MuJoCo quaternion [w, x, y, z] to euler angles [roll, pitch, yaw] in radians.
    """
    w, x, y, z = quat
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)
    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ─────────────────────────────────────────────
# Patch XML to swap terrain heightfield
# ─────────────────────────────────────────────
def patch_xml_terrain(xml_path, terrain_png_path, z_top, arena_length, arena_width,
                      n_segments=1, spawn_y=0.0):
    """
    Read an XML model and replace ANY existing floor/heightfield with our
    uniform terrain. All models get identical treatment:
      1. Strip any existing <hfield ...> asset
      2. Strip any existing ground geom (floor plane or hfield geom)
      3. Inject our new hfield asset + hfield geom + backup floor plane

    This guarantees every model sees exactly the same terrain, arena size,
    and backup floor. The backup floor sits just below the heightfield
    (z=-0.01) so the robot can never fall into void.
    """
    with open(xml_path, 'r', encoding='utf-8') as f:
        xml = f.read()

    # Get image dimensions for nrow/ncol
    from PIL import Image
    img = Image.open(terrain_png_path)
    ncol, nrow = img.size  # PIL gives (width, height)

    # Half-sizes for MuJoCo hfield size attribute
    x_half = arena_width / 2.0    # lateral (X)
    y_half = arena_length / 2.0   # forward (Y)

    terrain_rel = os.path.relpath(terrain_png_path, xml_path.parent)
    terrain_rel = terrain_rel.replace('\\', '/')

    hfield_asset = (f'<hfield name="terrain" file="{terrain_rel}" '
                    f'nrow="{nrow}" ncol="{ncol}" '
                    f'size="{x_half:.4f} {y_half:.4f} {z_top:.5f} 0.001"/>')

    hfield_geom = ('<geom name="terrain_geom" type="hfield" hfield="terrain" '
                   'pos="0 0 0" contype="1" conaffinity="4" condim="4" '
                   'friction="0.4 0.005 0.0001" '
                   'solref="0.01 1" solimp="0.9 0.95 0.001"/>')

    # ── 1) Strip ANY pre-existing hfield asset (multi-line safe) ──
    xml = re.sub(r'<hfield\s+[^>]*?/>', '', xml, flags=re.DOTALL)

    # ── 2) Strip ANY pre-existing ground geoms:
    #       - geom named "floor"
    #       - geom of type "hfield" (the treadmill in 4-seg, etc.)
    # Remove floor-named plane geom
    xml = re.sub(r'<geom\s+name="floor"[^>]*?/>\s*', '', xml, flags=re.DOTALL)
    # Remove any hfield-type geom
    xml = re.sub(r'<geom[^>]*type="hfield"[^>]*?/>\s*', '', xml, flags=re.DOTALL)

    # ── 3) Inject the new hfield asset before </asset> ──
    xml = re.sub(
        r'(</asset>)',
        '    ' + hfield_asset + '\n  \\1',
        xml,
        count=1
    )

    # ── 4) Inject ONLY the hfield geom right after <worldbody> ──
    #     NO backup floor plane — the tiled terrain IS the ground.
    injection = '\n    ' + hfield_geom + '\n    '
    xml = re.sub(r'(<worldbody>)', r'\1' + injection, xml, count=1)

    # Spawn at the CENTER of the arena. With a 14m × 14m terrain, every
    # model (including 15-seg ~1.35m) has plenty of runway in all directions
    # and the whole body (curved or not) lands entirely on tiles.
    #
    # Spawn height = max tile height (z_top) + robot standing height + margin.
    # The XML's default `robot_root pos="0 0 Z"` gives the verified flat-ground
    # standing height of the robot (legs extend DOWN from the root body, so
    # the root must sit that far above the ground). We extract it from the XML
    # so it stays in sync with any future kinematic changes.
    m = re.search(r'<body\s+name="robot_root"\s+pos="[^"]*?\s([0-9.+-eE]+)\s*"', xml)
    robot_standing_h = float(m.group(1)) if m else 0.15
    safety_margin = 0.05  # 5 cm extra air before drop
    drop_z = z_top + robot_standing_h + safety_margin
    xml = re.sub(
        r'(<body\s+name="robot_root"\s+pos=")([^"]+)(")',
        rf'\g<1>0 {spawn_y:.4f} {drop_z:.4f}\3',
        xml
    )

    return xml


import os


# ─────────────────────────────────────────────
# Single terrain trial
# ─────────────────────────────────────────────
def run_terrain_trial(xml_path, n_segments, terrain_png, z_top,
                      arena_length, arena_width,
                      duration=10.0, record_video=True,
                      video_fps=30, video_width=1280, video_height=720,
                      spawn_y=0.0):
    """
    Run one trial: load model with terrain, simulate, return metrics.
    """
    # Patch XML with terrain
    xml_str = patch_xml_terrain(xml_path, terrain_png, z_top,
                                arena_length, arena_width,
                                n_segments=n_segments,
                                spawn_y=spawn_y)

    # Load from string — need to write to a temp file so MuJoCo resolves
    # mesh paths relative to the original XML location
    import tempfile
    tmp_xml = Path(xml_path).parent / "_terrain_tmp.xml"
    try:
        with open(tmp_xml, 'w', encoding='utf-8') as f:
            f.write(xml_str)
        model = mujoco.MjModel.from_xml_path(str(tmp_xml))
        data = mujoco.MjData(model)
    except Exception as e:
        if tmp_xml.exists():
            tmp_xml.unlink()
        return {'status': 'LOAD_FAIL', 'error': str(e)}
    finally:
        if tmp_xml.exists():
            tmp_xml.unlink()

    n_ctrl = model.nu
    expected = n_segments * 4 + (n_segments - 1) * 2
    if n_ctrl != expected:
        return {'status': 'CTRL_MISMATCH',
                'error': f'expected {expected} actuators, got {n_ctrl}'}

    # Compute arena half-sizes (matching patch_xml_terrain)
    x_half = arena_width / 2.0
    y_half = arena_length / 2.0

    indices = build_actuator_indices(n_segments)

    # Sensor logging setup
    sensor_map = get_sensor_map(model) if model.nsensor > 0 else {}
    sensor_time = []
    sensor_rows = []
    base_pose_log = []

    # Gait parameters
    omega_spine = 4.0
    phi_spine = -np.pi
    bodyleg_coupling = 0.5
    phi_bodyleg = compute_phi_bodyleg(phi_spine, bodyleg_coupling)

    dt = model.opt.timestep
    steps = int(duration / dt)
    sensor_downsample = max(1, int(0.005 / dt))  # log sensors at ~200 Hz

    # Video recorder
    recorder = None
    frame_interval = max(1, int(1.0 / (video_fps * dt)))
    if record_video:
        try:
            import imageio
            recorder = VideoRecorder(model, data, n_segments=n_segments,
                                     width=video_width, height=video_height,
                                     fps=video_fps)
        except Exception:
            recorder = None

    # ── Drop & settle phase (1.0s free-fall + 1.5s ramp-up) ──
    # Let robot free-fall onto terrain with zero control
    drop_steps = int(1.0 / dt)
    for k in range(drop_steps):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        if recorder and k % frame_interval == 0:
            recorder.capture_frame()

    # Ramp-up gait gradually
    target = compute_ctrl(0.0, n_ctrl, n_segments, indices,
                          omega_spine=omega_spine, phi_spine=phi_spine,
                          phi_bodyleg=phi_bodyleg)
    init_steps = int(1.5 / dt)
    for k in range(init_steps):
        data.ctrl[:] = (k / init_steps) * target
        mujoco.mj_step(model, data)
        if recorder and k % frame_interval == 0:
            recorder.capture_frame()

    start_pos = data.qpos[0:3].copy()
    start_z = start_pos[2]

    # ── Main loop ──
    z_history = []
    pitch_history = []
    roll_history = []
    rec_interval = max(1, int(0.01 / dt))  # record every 10ms

    for step in range(steps):
        t = step * dt
        data.ctrl[:] = compute_ctrl(t, n_ctrl, n_segments, indices,
                                    omega_spine=omega_spine,
                                    phi_spine=phi_spine,
                                    phi_bodyleg=phi_bodyleg)
        mujoco.mj_step(model, data)

        # Record orientation at ~100Hz
        if step % rec_interval == 0:
            z_history.append(data.qpos[2])
            quat = data.qpos[3:7]
            roll_rad, pitch_rad, _ = quat_to_euler(quat)
            pitch_history.append(abs(np.degrees(pitch_rad)))
            roll_history.append(abs(np.degrees(roll_rad)))

        # Sensor recording (~200 Hz)
        if step % sensor_downsample == 0:
            sensor_time.append(data.time)
            base_pose_log.append(data.qpos[0:7].copy())
            if sensor_map:
                sensor_rows.append(extract_sensor_data(data, sensor_map))

        # Video frame
        if recorder and step % frame_interval == 0:
            recorder.capture_frame()

    # ── Metrics ──
    final_pos = data.qpos[0:3].copy()
    travel_x = final_pos[0] - start_pos[0]
    travel_y = final_pos[1] - start_pos[1]
    dist = np.sqrt(travel_x**2 + travel_y**2)
    z_final = final_pos[2]
    z_min = min(z_history) if z_history else start_z
    collapsed = z_min < start_z * 0.5

    z_arr = np.array(z_history)
    pitch_arr = np.array(pitch_history)
    roll_arr = np.array(roll_history)

    mean_pitch_dev = float(np.mean(pitch_arr))
    mean_roll_dev = float(np.mean(roll_arr))
    z_stability = float(np.std(z_arr))

    # Composite score: reward distance, penalize instability
    orientation_penalty = mean_pitch_dev + mean_roll_dev  # in degrees
    composite = dist / (1.0 + orientation_penalty / 10.0)  # normalize penalty

    # Entire arena is rough terrain (no flat zone)
    result = {
        'status': 'OK',
        'start_y': start_pos[1],
        'final_y': final_pos[1],
        'distance': dist,
        'travel_x': travel_x,
        'travel_y': travel_y,
        'z_start': start_z,
        'z_final': z_final,
        'z_min': z_min,
        'z_stability': z_stability,
        'mean_pitch_dev': mean_pitch_dev,
        'mean_roll_dev': mean_roll_dev,
        'max_pitch_dev': float(np.max(pitch_arr)),
        'max_roll_dev': float(np.max(roll_arr)),
        'collapsed': collapsed,
        'composite_score': composite,
        'n_sensors': model.nsensor,
        'n_sensor_samples': len(sensor_time),
    }

    # Build structured sensor arrays for .npz
    sensor_data = {
        'time': np.array(sensor_time),
        'base_pose': np.array(base_pose_log),  # (N,7) x,y,z,qw,qx,qy,qz
    }
    if sensor_rows:
        for key in sensor_rows[0].keys():
            sensor_data[key] = np.array([row[key] for row in sensor_rows])

    return result, recorder, sensor_data


# ─────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Rough terrain locomotion benchmark for centipede models",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--base", type=float, default=0.8,
                        help="Base height in inches (default: 0.8)")
    parser.add_argument("--levels", type=int, default=3,
                        help="Number of height levels (default: 3)")
    parser.add_argument("--n-seeds", type=int, default=1,
                        help="Number of terrain seeds / repeats per model (default: 1)")
    parser.add_argument("--spawn-y-range", type=float, default=5.0,
                        help="Randomize spawn y in [-R, R] per trial (default: 5.0 m). "
                             "Set to 0 to always spawn at y=0.")
    parser.add_argument("--seed-start", type=int, default=0,
                        help="Starting seed number (default: 0)")
    parser.add_argument("--segments", type=int, nargs="*",
                        default=[4, 5, 6, 7, 8, 15],
                        help="Segment counts to test (default: 4 5 6 7 8 15)")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Simulation duration in seconds (default: 10)")
    parser.add_argument("--no-video", action="store_true",
                        help="Skip video recording for faster execution")
    parser.add_argument("--fps", type=int, default=30,
                        help="Video frame rate (default: 30)")
    args = parser.parse_args()

    # ── Locate terrain files ──
    level_str = f"base{args.base:.1f}in_{args.levels}levels"
    terrain_subdir = TERRAIN_DIR / level_str
    meta_path = terrain_subdir / "terrain_meta.json"

    if not meta_path.exists():
        print(f"ERROR: Terrain not found at {terrain_subdir}")
        print(f"  Run first:  python scripts/generate_terrain.py "
              f"--base {args.base} --levels {args.levels} --n-seeds {args.n_seeds}")
        sys.exit(1)

    with open(meta_path) as f:
        terrain_meta = json.load(f)

    z_top = terrain_meta['z_top_m']
    arena_w = terrain_meta['arena_width_m']
    arena_l = terrain_meta['arena_length_m']
    heights_m = terrain_meta['heights_m']

    # ── Collect available terrain PNGs on disk ──
    available_pngs = sorted(terrain_subdir.glob("terrain_seed*.png"))
    if not available_pngs:
        print("ERROR: No terrain PNGs found. Run generate_terrain.py first.")
        sys.exit(1)

    def _seed_of(path):
        m = re.search(r'terrain_seed(\d+)\.png$', path.name)
        return int(m.group(1)) if m else -1
    available_map = {_seed_of(p): p for p in available_pngs}

    # Build the trial list: n_seeds rows, each with a terrain PNG (cycles
    # through whatever is on disk) and a deterministic random spawn y.
    terrains = []
    avail_sorted = sorted(available_map.items())
    for i, seed_i in enumerate(range(args.seed_start, args.seed_start + args.n_seeds)):
        if seed_i in available_map:
            png_path = available_map[seed_i]
        else:
            # Cycle through whatever terrain files exist on disk
            cycle_idx = i % len(avail_sorted)
            _, png_path = avail_sorted[cycle_idx]
        # Per-trial RNG seeded by the trial index — reproducible.
        trial_rng = np.random.default_rng(seed_i)
        if args.spawn_y_range > 0:
            spawn_y = float(trial_rng.uniform(-args.spawn_y_range, args.spawn_y_range))
        else:
            spawn_y = 0.0
        terrains.append((seed_i, png_path, spawn_y))

    # ── Output directory ──
    run_dir = OUTPUT_DIR / level_str
    video_dir = run_dir / "videos"
    sensor_dir = run_dir / "sensors"
    run_dir.mkdir(parents=True, exist_ok=True)
    sensor_dir.mkdir(parents=True, exist_ok=True)
    if not args.no_video:
        video_dir.mkdir(parents=True, exist_ok=True)

    # ── Models ──
    all_segments = sorted(args.segments)
    model_paths = {}
    for n in all_segments:
        if n == 4:
            p = MODEL_DIR / "centipede_4module.xml"
        else:
            p = MODEL_DIR / f"centipede_{n}seg.xml"
        if p.exists():
            model_paths[n] = p
        else:
            print(f"  Warning: model for {n} segments not found — {p.name}")

    if not model_paths:
        print("ERROR: No models found.")
        sys.exit(1)

    # ── Print header ──
    print("=" * 80)
    print("ROUGH TERRAIN LOCOMOTION BENCHMARK")
    print("=" * 80)
    print(f"  Terrain:      {level_str}")
    print(f"  Heights (in): {', '.join(f'{h/INCH_TO_M:.1f}' for h in heights_m)}")
    print(f"  Heights (cm): {', '.join(f'{h*100:.1f}' for h in heights_m)}")
    print(f"  Arena:        {arena_l}m × {arena_w}m")
    print(f"  Seeds:        {[s for s, _, _ in terrains]}")
    print(f"  Spawn y range: ±{args.spawn_y_range:.1f} m (random per trial)")
    print(f"  Models:       {list(model_paths.keys())}")
    print(f"  Duration:     {args.duration}s per trial")
    print(f"  Video:        {'off' if args.no_video else 'on'}")
    print(f"  Total trials: {len(model_paths)} models × {len(terrains)} seeds "
          f"= {len(model_paths) * len(terrains)}")
    print(f"  Output:       {run_dir}/")
    print("=" * 80)

    # ── Run all trials ──
    all_results = []
    trial_num = 0
    total_trials = len(model_paths) * len(terrains)

    for n_seg, xml_path in sorted(model_paths.items()):
        for seed_i, terrain_png, spawn_y in terrains:
            trial_num += 1
            model_name = xml_path.stem
            print(f"\n  [{trial_num}/{total_trials}] "
                  f"{n_seg}-seg × seed {seed_i} (y0={spawn_y:+.2f}m) ...",
                  end=" ", flush=True)

            t0 = time.time()
            recorder = None
            sensor_data = None
            try:
                out = run_terrain_trial(
                    xml_path, n_seg, terrain_png, z_top,
                    arena_l, arena_w,
                    duration=args.duration,
                    record_video=(not args.no_video),
                    video_fps=args.fps,
                    spawn_y=spawn_y,
                )
                if isinstance(out, tuple):
                    if len(out) == 3:
                        result, recorder, sensor_data = out
                    else:
                        result, recorder = out
                else:
                    result = out
            except Exception as e:
                result = {'status': 'ERROR', 'error': str(e)}

            elapsed = time.time() - t0

            if result['status'] != 'OK':
                print(f"FAIL: {result['status']} — {result.get('error', '')}")
                all_results.append({
                    'segments': n_seg,
                    'seed': seed_i,
                    'spawn_y': spawn_y,
                    'status': result['status'],
                })
                continue

            tag = "COLLAPSE" if result['collapsed'] else "OK"
            print(f"{tag}  dist={result['distance']:.3f}m  "
                  f"y:[{result.get('start_y',0):.2f}→{result.get('final_y',0):.2f}]  "
                  f"pitch={result['mean_pitch_dev']:.1f}°  "
                  f"roll={result['mean_roll_dev']:.1f}°  "
                  f"score={result['composite_score']:.3f}  [{elapsed:.1f}s]")

            # Save video
            if recorder and recorder.frames:
                vpath = video_dir / f"{model_name}_seed{seed_i}.mp4"
                try:
                    recorder.save(vpath)
                except Exception as e:
                    print(f"    Video error: {e}")
                finally:
                    recorder.close()

            # Save sensor data
            if sensor_data is not None and len(sensor_data.get('time', [])) > 0:
                spath = sensor_dir / f"{model_name}_seed{seed_i}_sensors.npz"
                try:
                    np.savez_compressed(
                        str(spath),
                        n_segments=n_seg,
                        seed=seed_i,
                        spawn_y=spawn_y,
                        terrain_base_inch=args.base,
                        terrain_levels=args.levels,
                        **sensor_data,
                    )
                except Exception as e:
                    print(f"    Sensor save error: {e}")

            all_results.append({
                'segments': n_seg,
                'seed': seed_i,
                'spawn_y': spawn_y,
                'status': result['status'],
                **{k: v for k, v in result.items() if k != 'status'},
            })

    # ── Per-trial CSV ──
    csv_path = run_dir / "trial_results.csv"
    if all_results:
        keys = list(all_results[0].keys())
        # Ensure all results have the same keys
        all_keys = set()
        for r in all_results:
            all_keys.update(r.keys())
        all_keys = sorted(all_keys)

        with open(csv_path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=all_keys, extrasaction='ignore')
            writer.writeheader()
            for r in all_results:
                writer.writerow(r)
        print(f"\n  Trial results: {csv_path}")

    # ── Summary: mean ± std per model ──
    print("\n" + "=" * 80)
    print("SUMMARY (mean ± std across seeds)")
    print("=" * 80)
    print(f"{'Seg':>4} {'Distance (m)':>16} {'Pitch dev (°)':>16} "
          f"{'Roll dev (°)':>16} {'Composite':>16} {'Collapsed':>10}")
    print("-" * 80)

    summary_rows = []
    for n_seg in sorted(model_paths.keys()):
        trials = [r for r in all_results
                  if r['segments'] == n_seg and r.get('status') == 'OK']
        if not trials:
            print(f"{n_seg:>4}   {'NO DATA':>12}")
            continue

        dists = [t['distance'] for t in trials]
        pitches = [t['mean_pitch_dev'] for t in trials]
        rolls = [t['mean_roll_dev'] for t in trials]
        scores = [t['composite_score'] for t in trials]
        n_collapsed = sum(1 for t in trials if t.get('collapsed', False))

        print(f"{n_seg:>4} "
              f"{np.mean(dists):>7.3f} ± {np.std(dists):<6.3f} "
              f"{np.mean(pitches):>7.1f} ± {np.std(pitches):<6.1f} "
              f"{np.mean(rolls):>7.1f} ± {np.std(rolls):<6.1f} "
              f"{np.mean(scores):>7.3f} ± {np.std(scores):<6.3f} "
              f"{n_collapsed}/{len(trials):>3}")

        summary_rows.append({
            'segments': n_seg,
            'n_trials': len(trials),
            'dist_mean': np.mean(dists),
            'dist_std': np.std(dists),
            'pitch_dev_mean': np.mean(pitches),
            'pitch_dev_std': np.std(pitches),
            'roll_dev_mean': np.mean(rolls),
            'roll_dev_std': np.std(rolls),
            'composite_mean': np.mean(scores),
            'composite_std': np.std(scores),
            'n_collapsed': n_collapsed,
        })

    # Save summary CSV
    if summary_rows:
        sum_csv = run_dir / "summary.csv"
        with open(sum_csv, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=summary_rows[0].keys())
            writer.writeheader()
            writer.writerows(summary_rows)
        print(f"\n  Summary CSV: {sum_csv}")

    print("=" * 80)
    print(f"  Output: {run_dir}/")
    if not args.no_video:
        print(f"  Videos: {video_dir}/")
    print("=" * 80)


if __name__ == "__main__":
    main()
