"""Render matched canonical/left-right-pitch-flip comparisons for Step 30."""

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

import run_center_rooted_dynamic_null_wave_flip as experiment  # noqa: E402
import run_fixed_window_random_and_pitch_ablation as base  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    OMEGA_BODY,
    _actual_contact_flags,
    _font,
    _set_leg_colors,
)


CONTACT_RGBA = np.asarray([1.0, 0.78, 0.05, 1.0])


def _color_state(prepared, stance: np.ndarray, contact: np.ndarray) -> None:
    _set_leg_colors(prepared.model, prepared.body_map, stance)
    for segment_index, side_index in np.argwhere(contact):
        geom_id = int(prepared.foot_geom_ids[segment_index, side_index])
        prepared.model.geom_rgba[geom_id] = CONTACT_RGBA


def _panel(
    pixels: np.ndarray,
    *,
    label: str,
    color: str,
    time_s: float,
    displacement_m: float,
    stance: np.ndarray,
    contact: np.ndarray,
):
    from PIL import Image, ImageDraw

    image = Image.fromarray(pixels)
    width, height = image.size
    header = 42
    footer = 42
    panel = Image.new("RGB", (width, height + header + footer), "#f2f4f5")
    panel.paste(image, (0, header))
    draw = ImageDraw.Draw(panel)
    draw.rectangle((0, 0, width, 5), fill=color)
    draw.text((15, 10), label, font=_font(20, bold=True), fill="#17202a")
    draw.text(
        (15, height + 51),
        (
            f"t={time_s:4.2f} s    D={1000*displacement_m:+7.1f} mm    "
            f"cmd-down/contact={int(np.sum(stance))}/{int(np.sum(contact))}"
        ),
        font=_font(15),
        fill="#273746",
    )
    return panel


def _render_comparison(
    repo_root: Path,
    output_dir: Path,
    *,
    n_segments: int,
    n_wave: float,
    phase_offset_deg: float,
    warmup_cycles: int,
    duration_cycles: float,
    fps: int,
) -> dict:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    prepared_list = [
        experiment.prepare_simulation(
            repo_root,
            n_segments=n_segments,
            n_wave=n_wave,
            phase_offset_deg=phase_offset_deg,
            mode=mode,
            warmup_cycles=warmup_cycles,
        )
        for mode in experiment.MODES
    ]
    panel_width, render_height = 640, 400
    renderers = []
    cameras = []
    options = []
    starts = []
    for prepared in prepared_list:
        prepared.model.vis.global_.offwidth = max(
            prepared.model.vis.global_.offwidth, panel_width
        )
        prepared.model.vis.global_.offheight = max(
            prepared.model.vis.global_.offheight, render_height
        )
        renderers.append(
            mujoco.Renderer(
                prepared.model, height=render_height, width=panel_width
            )
        )
        camera, option = base._camera_for(
            prepared.model, prepared.root_body_id, n_segments, mujoco
        )
        cameras.append(camera)
        options.append(option)
        starts.append(
            np.asarray(prepared.data.subtree_com[prepared.root_body_id]).copy()
        )

    gait_period_s = 2.0 * math.pi / OMEGA_BODY
    duration_s = float(duration_cycles) * gait_period_s
    dt = prepared_list[0].dt
    steps = int(math.ceil(duration_s / dt))
    demo_dir = output_dir / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    stem = f"canonical_vs_pitch_flip_Nwave{experiment.wave_token(n_wave)}"
    video_path = demo_dir / f"{stem}.mp4"
    preview_path = demo_dir / f"{stem}_preview.png"
    writer = imageio.get_writer(
        str(video_path),
        fps=fps,
        codec="libx264",
        quality=8,
        pixelformat="yuv420p",
        macro_block_size=2,
    )
    next_frame_s = 0.0
    frame_count = 0
    preview_written = False
    final_displacements = [0.0, 0.0]
    try:
        for step in range(steps):
            evaluation_time_s = step * dt
            for prepared in prepared_list:
                body_time_s = prepared.warmup_time_s + evaluation_time_s
                prepared.data.ctrl[:] = prepared.controller.control(body_time_s)
                mujoco.mj_step(prepared.model, prepared.data)

            time_end_s = (step + 1) * dt
            if time_end_s + 0.5 * dt < next_frame_s:
                continue

            panels = []
            for panel_index, (
                mode,
                prepared,
                renderer,
                camera,
                option,
                start,
            ) in enumerate(
                zip(
                    experiment.MODES,
                    prepared_list,
                    renderers,
                    cameras,
                    options,
                    starts,
                )
            ):
                body_time_s = prepared.warmup_time_s + evaluation_time_s
                stance = prepared.controller.stance_flags(body_time_s)
                contact = _actual_contact_flags(
                    prepared.model, prepared.data, prepared.body_map
                )
                _color_state(prepared, stance, contact)
                renderer.update_scene(
                    prepared.data, camera=camera, scene_option=option
                )
                displacement = float(
                    prepared.data.subtree_com[prepared.root_body_id][1] - start[1]
                )
                final_displacements[panel_index] = displacement
                panels.append(
                    _panel(
                        renderer.render().copy(),
                        label=experiment.MODE_LABELS[mode],
                        color=experiment.MODE_COLORS[mode],
                        time_s=time_end_s,
                        displacement_m=displacement,
                        stance=stance,
                        contact=contact,
                    )
                )

            top = 50
            canvas = Image.new(
                "RGB",
                (2 * panel_width, panels[0].height + top),
                "#e7ebed",
            )
            canvas.paste(panels[0], (0, top))
            canvas.paste(panels[1], (panel_width, top))
            draw = ImageDraw.Draw(canvas)
            draw.text(
                (16, 10),
                (
                    f"N={n_segments} | N_wave={n_wave:g} | "
                    f"body-yaw half-amplitude={experiment.BODY_WAVE_HALF_AMPLITUDE_RAD:.1f} rad"
                ),
                font=_font(22, bold=True),
                fill="#17202a",
            )
            draw.ellipse((790, 15, 805, 30), fill="#2aa657")
            draw.text((812, 10), "cmd down", font=_font(14), fill="#17202a")
            draw.ellipse((912, 15, 927, 30), fill="#e6452b")
            draw.text((934, 10), "cmd up", font=_font(14), fill="#17202a")
            draw.ellipse((1014, 15, 1029, 30), fill="#ffbe0b")
            draw.text((1036, 10), "contact", font=_font(14), fill="#17202a")
            writer.append_data(np.asarray(canvas))
            frame_count += 1
            if not preview_written and time_end_s >= 0.5 * duration_s:
                canvas.save(preview_path)
                preview_written = True
            next_frame_s += 1.0 / fps
    finally:
        writer.close()
        for renderer in renderers:
            renderer.close()

    return {
        "n_segments": n_segments,
        "N_wave": n_wave,
        "phase_offset_deg": phase_offset_deg,
        "warmup_cycles": warmup_cycles,
        "duration_cycles": duration_cycles,
        "duration_s": duration_s,
        "fps": fps,
        "frame_count": frame_count,
        "canonical_demo_displacement_m": final_displacements[0],
        "flipped_demo_displacement_m": final_displacements[1],
        "video": str(video_path),
        "preview": str(preview_path),
    }


def run(args: argparse.Namespace) -> None:
    repo_root = args.repo_root.resolve()
    output_dir = args.output_dir.resolve()
    demo_dir = output_dir / "demos"
    prior_rows = {}
    prior_manifest = demo_dir / "demo_manifest.json"
    if prior_manifest.is_file():
        for row in json.loads(prior_manifest.read_text(encoding="utf-8")):
            prior_rows[float(row["N_wave"])] = row
    for n_wave in sorted(set(args.n_wave)):
        row = _render_comparison(
            repo_root,
            output_dir,
            n_segments=args.n_segments,
            n_wave=n_wave,
            phase_offset_deg=args.phase_offset_deg,
            warmup_cycles=args.warmup_cycles,
            duration_cycles=args.duration_cycles,
            fps=args.fps,
        )
        prior_rows[float(n_wave)] = row
        print(
            f"N_wave={n_wave}: canonical={1000*row['canonical_demo_displacement_m']:+.1f} mm, "
            f"flipped={1000*row['flipped_demo_displacement_m']:+.1f} mm",
            flush=True,
        )

    rows = [prior_rows[n_wave] for n_wave in sorted(prior_rows)]
    with (demo_dir / "demo_manifest.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (demo_dir / "demo_manifest.json").write_text(
        json.dumps(rows, indent=2), encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    repo_root = experiment._repo_root()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=repo_root)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=experiment._output_dir(repo_root),
    )
    parser.add_argument("--n-segments", type=int, default=10)
    parser.add_argument(
        "--n-wave",
        type=float,
        nargs="+",
        default=[0.5, 1.0, 1.5, 2.0, 3.0, 4.0],
    )
    parser.add_argument("--phase-offset-deg", type=float, default=0.0)
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--duration-cycles", type=float, default=4.0)
    parser.add_argument("--fps", type=int, default=30)
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
