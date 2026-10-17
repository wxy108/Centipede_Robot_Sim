"""Render representative Step-37 IID-contact-opportunity demos.

Each demo replays the seed whose signed displacement is closest to the median
of the corresponding completed phase-by-segment condition. The controller, stronger
runtime body-yaw PD, warm-up, and 12-cycle evaluation match Step 37 exactly.
"""

from __future__ import annotations

import argparse
import csv
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

import run_locked_phase_segment_iid_sweep as step37  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    OMEGA_BODY,
    _actual_contact_flags,
    _font,
    _set_leg_colors,
)


WARMUP_CYCLES = 4
EVALUATION_CYCLES = 12
FPS = 20
WIDTH = 960
HEIGHT = 540


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, rows: list[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def wave_tag(wave: float) -> str:
    return f"{wave:.1f}".replace(".", "p")


def representative_trial(rows: list[dict], wave: float, n_segments: int) -> dict:
    candidates = [
        row for row in rows
        if row["status"] == "PASS"
        and int(row["n_segments"]) == n_segments
        and math.isclose(float(row["N_wave_ref10"]), wave)
    ]
    if len(candidates) != 128:
        raise RuntimeError(
            f"wave={wave:g}, N={n_segments}: expected 128 passing trials, found {len(candidates)}"
        )
    median = float(np.median([float(row["forward_displacement_m"]) for row in candidates]))
    return min(
        candidates,
        key=lambda row: (abs(float(row["forward_displacement_m"]) - median), int(row["seed"])),
    )


def compose_frame(
    pixels: np.ndarray,
    *,
    n_segments: int,
    wave: float,
    phase_deg: float,
    seed: int,
    time_s: float,
    displacement_m: float,
    commanded_down: np.ndarray,
    actual_contact: np.ndarray,
) -> np.ndarray:
    from PIL import Image, ImageDraw

    image = Image.fromarray(pixels)
    canvas = Image.new("RGB", (WIDTH, HEIGHT + 90), "#eef1f2")
    canvas.paste(image, (0, 50))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (16, 9),
        f"N={n_segments} | N_wave^(10)={wave:g} | locked phase={phase_deg:+.0f} deg | seed={seed}",
        font=_font(20, bold=True),
        fill="#17202a",
    )
    draw.text(
        (16, HEIGHT + 57),
        (
            f"t={time_s:5.2f} s   signed forward COM={displacement_m:+.3f} m   "
            f"commanded-down/contact={int(np.sum(commanded_down))}/{int(np.sum(actual_contact))}"
        ),
        font=_font(16),
        fill="#273746",
    )
    draw.ellipse((WIDTH - 300, 17, WIDTH - 284, 33), fill="#2aa657")
    draw.text((WIDTH - 277, 12), "commanded down", font=_font(15), fill="#17202a")
    draw.ellipse((WIDTH - 126, 17, WIDTH - 110, 33), fill="#e6452b")
    draw.text((WIDTH - 103, 12), "up", font=_font(15), fill="#17202a")
    return np.asarray(canvas)


def render_one(
    root: Path,
    output: Path,
    trials: list[dict],
    wave: float,
    n_segments: int = step37.REFERENCE_SEGMENTS,
) -> dict:
    import imageio.v2 as imageio
    import mujoco
    import mujoco.rollout

    selected = representative_trial(trials, wave, n_segments)
    seed = int(selected["seed"])
    assets = step37.load_assets(root, n_segments, step37.FULL_PD_NAME)
    controls, evaluation_start, _warmup_time = step37.build_controls(
        assets,
        seeds=[seed],
        reference_wave=wave,
        warmup_cycles=WARMUP_CYCLES,
        evaluation_cycles=EVALUATION_CYCLES,
    )
    controls = controls[0]
    data = mujoco.MjData(assets.model)
    controller = step37.LockedPhaseIIDController(
        assets.model,
        assets.indices,
        seed=seed,
        reference_wave=wave,
        n_segments=n_segments,
    )
    assets.model.vis.global_.offwidth = max(assets.model.vis.global_.offwidth, WIDTH)
    assets.model.vis.global_.offheight = max(assets.model.vis.global_.offheight, HEIGHT)
    renderer = mujoco.Renderer(assets.model, height=HEIGHT, width=WIDTH)
    camera, option = step37.step33.base._camera_for(
        assets.model, assets.root_body_id, n_segments, mujoco
    )

    phase_deg = math.degrees(step37.locked_phase_increment(wave))
    demo_dir = output / "demos"
    if n_segments != step37.REFERENCE_SEGMENTS:
        demo_dir = demo_dir / f"N{n_segments}"
    demo_dir.mkdir(parents=True, exist_ok=True)
    stem = f"N{n_segments}_Nwave10_{wave_tag(wave)}_iid_contact_opportunity_seed{seed:03d}"
    video = demo_dir / f"{stem}.mp4"
    preview = demo_dir / f"{stem}_preview.png"
    writer = imageio.get_writer(
        str(video), fps=FPS, codec="libx264", quality=8,
        pixelformat="yuv420p", macro_block_size=2,
    )

    state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    nstate = mujoco.mj_stateSize(assets.model, state_spec)
    initial = np.empty(nstate, dtype=float)
    mujoco.mj_getState(assets.model, data, initial, state_spec)
    rollout_data = [mujoco.MjData(assets.model)]
    with mujoco.rollout.Rollout(nthread=1) as engine:
        states, _ = engine.rollout(
            assets.model,
            rollout_data,
            initial[None, :],
            controls[None, :, :],
            control_spec=mujoco.mjtState.mjSTATE_CTRL.value,
            chunk_size=1,
        )
    warning_max = max(int(item.number) for item in rollout_data[0].warning)
    mujoco.mj_setState(assets.model, data, states[0, evaluation_start - 1], state_spec)
    mujoco.mj_forward(assets.model, data)
    start_com = np.asarray(data.subtree_com[assets.root_body_id], dtype=float).copy()
    evaluation_steps = states.shape[1] - evaluation_start
    frame_steps = np.unique(
        np.minimum(
            evaluation_steps - 1,
            np.round(np.arange(0.0, evaluation_steps * assets.dt, 1.0 / FPS) / assets.dt).astype(int),
        )
    )
    if frame_steps[-1] != evaluation_steps - 1:
        frame_steps = np.append(frame_steps, evaluation_steps - 1)

    last_displacement = float("nan")
    minimum_up_z = 1.0
    frame_count = 0
    preview_written = False
    try:
        for local_step in frame_steps:
            state_index = evaluation_start + int(local_step)
            mujoco.mj_setState(assets.model, data, states[0, state_index], state_spec)
            mujoco.mj_forward(assets.model, data)
            minimum_up_z = min(
                minimum_up_z,
                float(data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2]),
            )
            evaluation_time = (int(local_step) + 1) * assets.dt
            commanded_down = controller.stance_flags(int(local_step) * assets.dt)
            actual_contact = _actual_contact_flags(assets.model, data, assets.body_map)
            _set_leg_colors(assets.model, assets.body_map, commanded_down)
            renderer.update_scene(data, camera=camera, scene_option=option)
            last_displacement = float(data.subtree_com[assets.root_body_id, 1] - start_com[1])
            frame = compose_frame(
                renderer.render().copy(),
                n_segments=n_segments,
                wave=wave,
                phase_deg=phase_deg,
                seed=seed,
                time_s=evaluation_time,
                displacement_m=last_displacement,
                commanded_down=commanded_down,
                actual_contact=actual_contact,
            )
            writer.append_data(frame)
            frame_count += 1
            if not preview_written and evaluation_time >= 0.5 * EVALUATION_CYCLES * 2.0 * math.pi / OMEGA_BODY:
                from PIL import Image

                Image.fromarray(frame).save(preview)
                preview_written = True
    finally:
        writer.close()
        renderer.close()

    with imageio.get_reader(str(video)) as reader:
        readable_frames = int(reader.count_frames())
        metadata = reader.get_meta_data()
    trial_displacement = float(selected["forward_displacement_m"])
    return {
        "N_wave_ref10": wave,
        "locked_phase_offset_deg": phase_deg,
        "n_segments": n_segments,
        "seed": seed,
        "selection": "nearest passing trial to within-condition median signed displacement",
        "trial_displacement_m": trial_displacement,
        "rendered_displacement_m": last_displacement,
        "absolute_replay_error_m": abs(last_displacement - trial_displacement),
        "duration_s": EVALUATION_CYCLES * 2.0 * math.pi / OMEGA_BODY,
        "fps": FPS,
        "written_frames": frame_count,
        "readable_frames": readable_frames,
        "codec": str(metadata.get("codec", "unknown")),
        "render_mujoco_version": mujoco.__version__,
        "model_sha256": assets.model_sha256,
        "all_states_finite": bool(np.all(np.isfinite(states))),
        "minimum_up_z": minimum_up_z,
        "maximum_mujoco_warning_count": warning_max,
        "video": str(video),
        "preview": str(preview),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--segments", type=int, nargs="+", default=[step37.REFERENCE_SEGMENTS])
    args = parser.parse_args()
    if any(value not in step37.SEGMENTS for value in args.segments):
        raise ValueError(f"segments must be selected from {step37.SEGMENTS}")
    root = step37.repo_root()
    output = root / "exp_data" / "reliability" / step37.OUTPUT_NAME
    trials = read_csv(output / "iid_trials.csv")
    rows = []
    for n_segments in args.segments:
        for wave in step37.REFERENCE_WAVES:
            row = render_one(root, output, trials, wave, n_segments=n_segments)
            rows.append(row)
            print(
                f"rendered N={n_segments}, wave={wave:g}, "
                f"D={row['rendered_displacement_m']:+.3f} m, "
                f"replay difference={row['absolute_replay_error_m']:.3e} m",
                flush=True,
            )
    stem = "demo" if args.segments == [step37.REFERENCE_SEGMENTS] else "small_N_demo"
    write_csv(output / f"{stem}_manifest.csv", rows)
    (output / f"{stem}_manifest.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    validation = {
        "passed": bool(
            len(rows) == len(step37.REFERENCE_WAVES) * len(args.segments)
            and all(row["readable_frames"] == row["written_frames"] for row in rows)
            and all(row["maximum_mujoco_warning_count"] == 0 for row in rows)
            and all(row["all_states_finite"] and row["minimum_up_z"] > 0.0 for row in rows)
        ),
        "segments": args.segments,
        "expected_video_count": len(step37.REFERENCE_WAVES) * len(args.segments),
        "video_count": len(rows),
        "all_media_readable": all(row["readable_frames"] == row["written_frames"] for row in rows),
        "all_displacement_signs_consistent_with_raw_trials": all(
            np.sign(row["rendered_displacement_m"]) == np.sign(row["trial_displacement_m"])
            for row in rows
        ),
        "cross_runtime_displacement_difference_is_diagnostic_not_gate": True,
        "cross_runtime_sign_consistency_is_diagnostic_not_gate": True,
        "reason_exact_replay_is_not_a_gate": (
            "The original sweep did not record its MuJoCo version; the available "
            "approved rendering runtime is recorded per video."
        ),
        "maximum_cross_runtime_displacement_difference_m": max(
            row["absolute_replay_error_m"] for row in rows
        ),
        "maximum_mujoco_warning_count": max(row["maximum_mujoco_warning_count"] for row in rows),
    }
    (output / f"{stem}_validation.json").write_text(
        json.dumps(validation, indent=2), encoding="utf-8"
    )
    if not validation["passed"]:
        raise RuntimeError(f"demo validation failed: {validation}")


if __name__ == "__main__":
    main()
