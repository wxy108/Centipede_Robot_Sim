"""Render median-representative Step-40 IID contact-erasure trials.

For each locked phase label, render N=5 and N=10.  Red marks a physical leg
whose collision channel is erased in the current T_g/50 window; green marks an
enabled leg with realized contact; light gray marks an enabled leg without a
current contact.  Leg pitch remains commanded down throughout.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import sys
from pathlib import Path

import numpy as np


os.environ.setdefault("CENTIPEDE_SKIP_OPTIONAL_MUJOCO_PLUGINS", "1")
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import run_high_frequency_iid_contact_erasure_sweep as step40  # noqa: E402
import run_locked_phase_segment_iid_sweep as step37  # noqa: E402
from run_independent_random_leg_phases import _actual_contact_flags, _font  # noqa: E402


WIDTH = 960
HEIGHT = 540
HEADER = 52
FOOTER = 70
FPS = 24


def select_trials(rows: list[dict]) -> list[dict]:
    selected = []
    for wave in step37.REFERENCE_WAVES:
        for n_segments in (5, 10):
            group = [
                row for row in rows
                if int(row["n_segments"]) == n_segments
                and math.isclose(float(row["N_wave_ref10"]), wave)
                and row["status"] == "PASS"
            ]
            if not group:
                raise RuntimeError(f"no passing trials for N={n_segments}, wave={wave:g}")
            median = float(np.median([float(row["forward_displacement_m"]) for row in group]))
            selected.append(min(
                group,
                key=lambda row: (abs(float(row["forward_displacement_m"]) - median), int(row["trial_index"])),
            ))
    return selected


def set_channel_colors(model, geom_ids: np.ndarray, erased: np.ndarray, contact: np.ndarray) -> None:
    for index, geom_id in np.ndenumerate(geom_ids):
        if erased[index]:
            rgba = (0.91, 0.12, 0.10, 1.0)
        elif contact[index]:
            rgba = (0.10, 0.68, 0.26, 1.0)
        else:
            rgba = (0.72, 0.76, 0.80, 1.0)
        model.geom_rgba[int(geom_id)] = rgba


def render_one(root: Path, output: Path, selected: dict) -> dict:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    n_segments = int(selected["n_segments"])
    wave = float(selected["N_wave_ref10"])
    trial_index = int(selected["trial_index"])
    q = float(selected["q_commanded"])
    evaluation_cycles = int(selected["evaluation_cycles"])
    assets = step37.load_assets(root, n_segments, step37.FULL_PD_NAME)
    geom_ids = step40.leg_geom_ids(assets.model, n_segments, mujoco)
    pair_ids = step40.leg_ground_pair_ids(assets.model, geom_ids, mujoco)
    flat_geoms = geom_ids.reshape(-1)
    flat_pairs = pair_ids.reshape(-1)
    original_collision = (
        np.asarray(assets.model.geom_contype[flat_geoms], dtype=int).copy(),
        np.asarray(assets.model.geom_conaffinity[flat_geoms], dtype=int).copy(),
        np.asarray(assets.model.pair_margin[flat_pairs], dtype=float).copy(),
    )
    original_rgba = np.asarray(assets.model.geom_rgba[flat_geoms], dtype=float).copy()
    start_state, warmup_time, state_spec = step40.warmup_state(
        assets, wave, step40.DEFAULT_WARMUP_CYCLES
    )
    edges, _min_window, _max_window = step40.window_edges(assets.dt, evaluation_cycles)
    n_windows = len(edges) - 1
    mask = step40.erasure_mask(n_segments, wave, trial_index, n_windows, q)
    expected_digest = hashlib.sha256(np.packbits(mask.reshape(-1)).tobytes()).hexdigest()
    if expected_digest != selected["mask_sha256"]:
        raise RuntimeError("selected stochastic mask does not reproduce")

    data = mujoco.MjData(assets.model)
    mujoco.mj_setState(assets.model, data, start_state, state_spec)
    step40.restore_leg_collisions(assets.model, geom_ids, pair_ids, original_collision)
    mujoco.mj_forward(assets.model, data)
    start_com = np.asarray(data.subtree_com[assets.root_body_id], dtype=float).copy()
    total_steps = int(edges[-1])
    frame_steps = np.unique(np.rint(
        np.arange(0.0, total_steps * assets.dt + 0.5 / FPS, 1.0 / FPS) / assets.dt
    ).astype(int))
    frame_steps = frame_steps[(frame_steps >= 0) & (frame_steps <= total_steps)]
    if frame_steps[-1] != total_steps:
        frame_steps = np.append(frame_steps, total_steps)
    assets.model.vis.global_.offwidth = max(assets.model.vis.global_.offwidth, WIDTH)
    assets.model.vis.global_.offheight = max(assets.model.vis.global_.offheight, HEIGHT)
    renderer = mujoco.Renderer(assets.model, height=HEIGHT, width=WIDTH)
    camera, option = step37.step33.base._camera_for(
        assets.model, assets.root_body_id, n_segments, mujoco
    )
    demos = output / "demos"
    demos.mkdir(parents=True, exist_ok=True)
    wave_tag = f"{wave:.1f}".replace(".", "p")
    video = demos / f"N{n_segments:02d}_Nwave10_{wave_tag}_trial{trial_index:03d}_iid_contact_erasure.mp4"
    preview = video.with_name(video.stem + "_preview.png")
    writer = imageio.get_writer(
        str(video), fps=FPS, codec="libx264", quality=8,
        pixelformat="yuv420p", macro_block_size=2,
    )
    frame_cursor = 0
    written = 0
    preview_written = False

    def append_frame(step_index: int, current_mask: np.ndarray) -> None:
        nonlocal written, preview_written
        contact = _actual_contact_flags(assets.model, data, assets.body_map)
        set_channel_colors(assets.model, geom_ids, current_mask, contact)
        renderer.update_scene(data, camera=camera, scene_option=option)
        canvas = Image.new("RGB", (WIDTH, HEIGHT + HEADER + FOOTER), "#edf1f2")
        canvas.paste(Image.fromarray(renderer.render().copy()), (0, HEADER))
        draw = ImageDraw.Draw(canvas)
        phase = math.degrees(step37.locked_phase_increment(wave))
        draw.text(
            (16, 13),
            f"IID contact erasure | N={n_segments} | phase={phase:+.0f} deg | q={q:g}",
            font=_font(20, bold=True), fill="#17202a",
        )
        displacement = np.asarray(data.subtree_com[assets.root_body_id], dtype=float) - start_com
        draw.text(
            (16, HEIGHT + HEADER + 14),
            f"t={step_index * assets.dt:5.2f} s   signed forward COM={displacement[1]:+.3f} m   "
            f"erased={int(np.sum(current_mask))}/{2*n_segments}   actual contact={int(np.sum(contact))}/{2*n_segments}",
            font=_font(16), fill="#273746",
        )
        draw.ellipse((WIDTH - 224, 17, WIDTH - 208, 33), fill=(232, 31, 27))
        draw.text((WIDTH - 202, 13), "erased", font=_font(14), fill="#273746")
        draw.ellipse((WIDTH - 127, 17, WIDTH - 111, 33), fill=(25, 173, 66))
        draw.text((WIDTH - 105, 13), "contact", font=_font(14), fill="#273746")
        writer.append_data(np.asarray(canvas))
        written += 1
        if not preview_written and step_index >= total_steps // 2:
            canvas.save(preview)
            preview_written = True

    try:
        append_frame(0, mask[0])
        frame_cursor = 1
        global_step = 0
        previous_mask = None
        for window_index in range(n_windows):
            current_mask = mask[window_index]
            if previous_mask is None or not np.array_equal(current_mask, previous_mask):
                step40.apply_erasure_state(
                    assets.model, geom_ids, pair_ids, current_mask, original_collision
                )
            previous_mask = current_mask
            midpoint = 0.5 * (edges[window_index] + edges[window_index + 1])
            data.ctrl[:] = step40.all_down_control(
                assets, warmup_time + midpoint * assets.dt, wave
            )
            while global_step < int(edges[window_index + 1]):
                mujoco.mj_step(assets.model, data)
                global_step += 1
                while frame_cursor < len(frame_steps) and global_step >= int(frame_steps[frame_cursor]):
                    append_frame(global_step, current_mask)
                    frame_cursor += 1
    finally:
        writer.close()
        renderer.close()
        step40.restore_leg_collisions(assets.model, geom_ids, pair_ids, original_collision)
        assets.model.geom_rgba[flat_geoms] = original_rgba

    final_displacement = float(
        np.asarray(data.subtree_com[assets.root_body_id], dtype=float)[step40.FORWARD_AXIS]
        - start_com[step40.FORWARD_AXIS]
    )
    numeric_displacement = float(selected["forward_displacement_m"])
    with imageio.get_reader(str(video)) as reader:
        readable = int(reader.count_frames())
    gates = {
        "mask_sha256_matches_raw_trial": expected_digest == selected["mask_sha256"],
        "numeric_replay_matches_raw_trial": abs(final_displacement - numeric_displacement) < 1.0e-10,
        "media_frame_count_matches": readable == written,
        "preview_exists": preview.is_file(),
        "no_mujoco_warnings": sum(int(item.number) for item in data.warning) == 0,
    }
    return {
        "status": "PASS" if all(gates.values()) else "FAIL",
        "n_segments": n_segments,
        "N_wave_ref10": wave,
        "locked_spatial_phase_offset_deg": math.degrees(step37.locked_phase_increment(wave)),
        "trial_index": trial_index,
        "q": q,
        "raw_trial_forward_displacement_m": numeric_displacement,
        "replay_forward_displacement_m": final_displacement,
        "absolute_replay_error_m": abs(final_displacement - numeric_displacement),
        "written_frames": written,
        "readable_frames": readable,
        "video": str(video),
        "preview": str(preview),
        "gates": json.dumps(gates, sort_keys=True),
    }


def main() -> None:
    root = step37.repo_root()
    output = root / "exp_data" / "reliability" / step40.OUTPUT_NAME
    rows = step40.read_csv(output / "iid_contact_erasure_trials.csv")
    expected = len(step37.REFERENCE_WAVES) * len(step37.SEGMENTS) * step40.DEFAULT_TRIALS
    if len(rows) != expected:
        raise RuntimeError(f"final trial table is incomplete: {len(rows)} of {expected}")
    selected = select_trials(rows)
    manifest = []
    for row in selected:
        result = render_one(root, output, row)
        manifest.append(result)
        step40.write_csv(output / "demo_manifest.csv", manifest)
        print(
            f"demo N={result['n_segments']}, wave={result['N_wave_ref10']:g}, "
            f"trial={result['trial_index']}: {result['status']}",
            flush=True,
        )
    validation = {
        "passed": len(manifest) == 14 and all(row["status"] == "PASS" for row in manifest),
        "expected_demo_count": 14,
        "demo_count": len(manifest),
        "selection": "closest passing trial to the condition median, N=5 and N=10 per phase",
        "red_semantics": "physical-leg collision channel erased in the current T_g/50 window",
        "green_semantics": "channel enabled and the leg has realized contact",
        "gray_semantics": "channel enabled without realized contact",
    }
    step40.write_json(output / "demo_validation.json", validation)
    if not validation["passed"]:
        raise RuntimeError(f"demo validation failed: {validation}")


if __name__ == "__main__":
    main()
