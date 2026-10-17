"""Render representative IID trials from the center-rooted physical-robot sweep."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np


HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import generate_center_rooted_physical_robot_models as generator  # noqa: E402
import run_center_rooted_dynamic_null_iid as center  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    _actual_contact_flags,
    _font,
    _set_leg_colors,
)


def _repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def _model_path(output_dir: Path, n_segments: int) -> Path:
    return (
        output_dir
        / "models"
        / generator.VARIANT
        / f"centipede_{n_segments}seg_{generator.VARIANT}.xml"
    )


def _choose_median_seed(output_dir: Path, n_segments: int) -> int:
    records = []
    for path in (output_dir / "trial_records" / "center_rooted").glob(
        f"N{n_segments}_seed*.json"
    ):
        row = json.loads(path.read_text(encoding="utf-8"))
        if row.get("status") == "PASS":
            records.append(row)
    if not records:
        raise RuntimeError(f"No completed N={n_segments} physical-robot trials")
    values = np.asarray([float(row["forward_displacement_m"]) for row in records])
    median = float(np.median(values))
    chosen = min(
        records,
        key=lambda row: (
            abs(float(row["forward_displacement_m"]) - median),
            int(row["seed"]),
        ),
    )
    return int(chosen["seed"])


def _render_one(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    seed: int,
    warmup_cycles: int,
    duration_s: float,
    fps: int,
) -> dict:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    model_path = _model_path(output_dir, n_segments)
    prepared = center._prepare_center_simulation(
        repo_root,
        model_path,
        n_segments=n_segments,
        seed=seed,
        warmup_cycles=warmup_cycles,
        complement=False,
        stability_body_name="Chasis1",
    )
    width, height = 960, 600
    prepared.model.vis.global_.offwidth = max(prepared.model.vis.global_.offwidth, width)
    prepared.model.vis.global_.offheight = max(prepared.model.vis.global_.offheight, height)
    renderer = mujoco.Renderer(prepared.model, height=height, width=width)
    camera, option = base._camera_for(
        prepared.model, prepared.root_body_id, n_segments, mujoco
    )
    start = np.asarray(prepared.data.subtree_com[prepared.root_body_id]).copy()

    demo_dir = output_dir / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    stem = f"N{n_segments}_center_rooted_physical_iid_seed{seed:03d}"
    video_path = demo_dir / f"{stem}.mp4"
    preview_path = demo_dir / f"{stem}_preview.png"
    writer = imageio.get_writer(
        str(video_path), fps=fps, codec="libx264", quality=8,
        pixelformat="yuv420p", macro_block_size=2,
    )
    steps = int(math.ceil(duration_s / prepared.dt))
    next_frame_s = 0.0
    frame_count = 0
    try:
        for step in range(steps):
            evaluation_time_s = step * prepared.dt
            body_time_s = prepared.warmup_time_s + evaluation_time_s
            prepared.data.ctrl[:] = prepared.controller.control(
                body_time_s, evaluation_time_s
            )
            mujoco.mj_step(prepared.model, prepared.data)
            time_end_s = (step + 1) * prepared.dt
            if time_end_s + 0.5 * prepared.dt < next_frame_s:
                continue
            stance = prepared.controller.stance_flags(evaluation_time_s)
            contact = _actual_contact_flags(
                prepared.model, prepared.data, prepared.body_map
            )
            _set_leg_colors(prepared.model, prepared.body_map, stance)
            renderer.update_scene(prepared.data, camera=camera, scene_option=option)
            image = Image.fromarray(renderer.render().copy())
            canvas = Image.new("RGB", (width, height + 82), "#eef1f2")
            canvas.paste(image, (0, 44))
            draw = ImageDraw.Draw(canvas)
            draw.text(
                (16, 9),
                f"Center-rooted physical robot | N={n_segments} | IID pitch commands",
                font=_font(21, bold=True), fill="#17202a",
            )
            displacement = float(
                prepared.data.subtree_com[prepared.root_body_id][1] - start[1]
            )
            draw.text(
                (16, height + 51),
                (
                    f"t={time_end_s:4.2f} s   signed COM displacement={displacement:+.3f} m   "
                    f"commanded-down/actual-contact={int(np.sum(stance))}/{int(np.sum(contact))}"
                ),
                font=_font(16), fill="#273746",
            )
            draw.ellipse((width - 210, 14, width - 194, 30), fill="#2aa657")
            draw.text((width - 187, 9), "down", font=_font(15), fill="#17202a")
            draw.ellipse((width - 105, 14, width - 89, 30), fill="#e6452b")
            draw.text((width - 82, 9), "up", font=_font(15), fill="#17202a")
            writer.append_data(np.asarray(canvas))
            frame_count += 1
            if not preview_path.exists() and time_end_s >= 0.5 * duration_s:
                canvas.save(preview_path)
            next_frame_s += 1.0 / fps
    finally:
        writer.close()
        renderer.close()

    return {
        "n_segments": n_segments,
        "seed": seed,
        "selection": "nearest completed trial to within-N median displacement",
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "video": str(video_path),
        "preview": str(preview_path),
    }


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    rows = []
    for n_segments in sorted(set(args.segments)):
        seed = _choose_median_seed(output_dir, n_segments)
        rows.append(
            _render_one(
                repo_root, output_dir, n_segments=n_segments, seed=seed,
                warmup_cycles=args.warmup_cycles, duration_s=args.duration_s,
                fps=args.fps,
            )
        )
    with (output_dir / "demo_manifest.csv").open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / "demo_manifest.json").write_text(
        json.dumps(rows, indent=2), encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = _repo_root_from_script()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir", type=Path,
        default=repo_root / "exp_data" / "reliability" / "step26b_center_rooted_physical_robot_uniform512",
    )
    parser.add_argument("--segments", type=int, nargs="+", default=[4, 7, 10])
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--duration-s", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=30)
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
