"""Step-37 deterministic all-pitch-down reference over locked phase offsets.

N=10, body-yaw half-amplitude 19 deg, the same moderate body-yaw PD, and fixed
leg yaw.  After the neutral settling ramp, every leg-pitch target stays at the
down angle throughout bridge, warm-up, and evaluation.  Ground contact is not
forced: its realized fraction is measured separately.
"""

from __future__ import annotations

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
import render_locked_phase_segment_iid_demos as media  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    LEG_DOWN_DEG, OMEGA_BODY, SETTLE_TIME_S,
    _actual_contact_flags, _font, _root_roll_pitch_deg, _set_leg_colors,
)


def run_one(root: Path, output: Path, wave: float) -> tuple[dict, list[dict]]:
    import imageio.v2 as imageio
    import mujoco
    import mujoco.rollout
    from PIL import Image, ImageDraw

    n_segments = 10
    assets = step37.load_assets(root, n_segments, step37.FULL_PD_NAME)
    controls, evaluation_start, _ = step37.build_controls(
        assets, seeds=[0], reference_wave=wave,
        warmup_cycles=media.WARMUP_CYCLES,
        evaluation_cycles=media.EVALUATION_CYCLES,
    )
    shoulder_r, leg_r, shoulder_l, leg_l, link_y, link_p = assets.indices
    settle_steps = int(math.ceil(SETTLE_TIME_S / assets.dt))
    pitch_ids = np.r_[leg_r, leg_l]
    controls[0, settle_steps:, pitch_ids] = math.radians(LEG_DOWN_DEG)
    all_down_command = bool(np.allclose(
        controls[0, settle_steps:, pitch_ids], math.radians(LEG_DOWN_DEG)
    ))
    data = mujoco.MjData(assets.model)
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    initial = np.empty(mujoco.mj_stateSize(assets.model, spec), dtype=float)
    mujoco.mj_getState(assets.model, data, initial, spec)
    workers = [mujoco.MjData(assets.model)]
    with mujoco.rollout.Rollout(nthread=1) as engine:
        states, _ = engine.rollout(
            assets.model, workers, initial[None, :], controls,
            control_spec=mujoco.mjtState.mjSTATE_CTRL.value, chunk_size=1,
        )
    warnings = sum(int(item.number) for item in workers[0].warning)
    mujoco.mj_setState(assets.model, data, states[0, evaluation_start-1], spec)
    mujoco.mj_forward(assets.model, data)
    start_com = np.asarray(data.subtree_com[assets.root_body_id]).copy()
    evaluation_steps = states.shape[1] - evaluation_start
    frame_steps = np.unique(np.minimum(
        evaluation_steps - 1,
        np.round(np.arange(0.0, evaluation_steps*assets.dt, 1.0/media.FPS)/assets.dt).astype(int),
    ))
    if frame_steps[-1] != evaluation_steps-1:
        frame_steps = np.append(frame_steps, evaluation_steps-1)
    body_qpos = assets.model.jnt_qposadr[assets.model.actuator_trnid[link_y, 0]]
    down_flags = np.ones((n_segments, 2), dtype=bool)
    _set_leg_colors(assets.model, assets.body_map, down_flags)
    assets.model.vis.global_.offwidth = max(assets.model.vis.global_.offwidth, media.WIDTH)
    assets.model.vis.global_.offheight = max(assets.model.vis.global_.offheight, media.HEIGHT)
    renderer = mujoco.Renderer(assets.model, height=media.HEIGHT, width=media.WIDTH)
    camera, option = step37.step33.base._camera_for(assets.model, assets.root_body_id, n_segments, mujoco)
    demo_dir = output / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    tag = media.wave_tag(wave)
    video = demo_dir / f"N10_Nwave10_{tag}_all_pitch_down.mp4"
    preview = demo_dir / f"N10_Nwave10_{tag}_all_pitch_down_preview.png"
    writer = imageio.get_writer(
        str(video), fps=media.FPS, codec="libx264", quality=8,
        pixelformat="yuv420p", macro_block_size=2,
    )
    phase_deg = math.degrees(step37.locked_phase_increment(wave))
    series = []
    body_error = []
    min_up_z = 1.0
    max_roll = max_pitch = 0.0
    preview_written = False
    try:
        for local_step in frame_steps:
            state_index = evaluation_start + int(local_step)
            mujoco.mj_setState(assets.model, data, states[0, state_index], spec)
            mujoco.mj_forward(assets.model, data)
            time_s = (int(local_step)+1)*assets.dt
            contact = _actual_contact_flags(assets.model, data, assets.body_map)
            displacement = np.asarray(data.subtree_com[assets.root_body_id])-start_com
            body_error.extend((data.qpos[body_qpos]-controls[0, state_index, link_y]).tolist())
            min_up_z = min(min_up_z, float(data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2]))
            pose = np.zeros(7)
            pose[3:7] = data.xquat[assets.stability_body_id]
            roll, pitch = _root_roll_pitch_deg(pose)
            max_roll = max(max_roll, abs(roll))
            max_pitch = max(max_pitch, abs(pitch))
            series.append({
                "N_wave_ref10": wave, "locked_phase_offset_deg": phase_deg,
                "time_s": time_s, "forward_displacement_m": float(displacement[1]),
                "lateral_displacement_m": float(displacement[0]),
                "actual_contact_count": int(np.sum(contact)),
                "actual_contact_fraction": float(np.mean(contact)),
            })
            renderer.update_scene(data, camera=camera, scene_option=option)
            canvas = Image.new("RGB", (media.WIDTH, media.HEIGHT+90), "#eef1f2")
            canvas.paste(Image.fromarray(renderer.render().copy()), (0, 50))
            draw = ImageDraw.Draw(canvas)
            draw.text(
                (16, 9),
                f"All pitch down | N=10 | N_wave^(10)={wave:g} | phase={phase_deg:+.0f} deg",
                font=_font(20, bold=True), fill="#17202a",
            )
            draw.text(
                (16, media.HEIGHT+57),
                f"t={time_s:5.2f} s   signed forward COM={displacement[1]:+.3f} m   actual contact={int(np.sum(contact))}/20 legs",
                font=_font(16), fill="#273746",
            )
            writer.append_data(np.asarray(canvas))
            if not preview_written and time_s >= 0.5*media.EVALUATION_CYCLES*2.0*math.pi/OMEGA_BODY:
                canvas.save(preview)
                preview_written = True
    finally:
        writer.close()
        renderer.close()
    with imageio.get_reader(str(video)) as reader:
        readable_frames = int(reader.count_frames())
    gates = {
        "all_pitch_commands_down_after_settle": all_down_command,
        "body_pitch_locked": bool(np.allclose(controls[0, :, link_p], 0.0)),
        "all_states_finite": bool(np.all(np.isfinite(states))),
        "no_mujoco_warnings": warnings == 0,
        "never_inverted_at_sampled_states": min_up_z > 0.0,
        "media_readable": readable_frames == len(frame_steps),
    }
    row = {
        "N_wave_ref10": wave, "locked_phase_offset_deg": phase_deg,
        "n_segments": n_segments, "condition": "all_pitch_down_reference",
        "status": "PASS" if all(gates.values()) else "FAIL",
        "body_yaw_half_amplitude_deg": step37.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_pd": step37.FULL_PD_NAME, "pitch_down_deg": LEG_DOWN_DEG,
        "warmup_cycles": media.WARMUP_CYCLES,
        "evaluation_cycles": media.EVALUATION_CYCLES,
        "evaluation_time_s": evaluation_steps*assets.dt,
        "forward_displacement_m": series[-1]["forward_displacement_m"],
        "lateral_displacement_m": series[-1]["lateral_displacement_m"],
        "mean_actual_contact_fraction": float(np.mean([item["actual_contact_fraction"] for item in series])),
        "rms_body_yaw_tracking_error_deg": math.degrees(float(np.sqrt(np.mean(np.square(body_error))))),
        "max_abs_roll_deg": max_roll, "max_abs_pitch_deg": max_pitch,
        "minimum_up_z": min_up_z, "mujoco_warning_total": warnings,
        "mujoco_version": mujoco.__version__, "model_sha256": assets.model_sha256,
        "written_frames": len(frame_steps), "readable_frames": readable_frames,
        "video": str(video), "preview": str(preview),
        "gates": json.dumps(gates, sort_keys=True),
    }
    return row, series


def make_figures(output: Path, rows: list[dict], iid_summary: list[dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    waves = [row["N_wave_ref10"] for row in rows]
    phases = [row["locked_phase_offset_deg"] for row in rows]
    displacement = [row["forward_displacement_m"] for row in rows]
    contact = [row["mean_actual_contact_fraction"] for row in rows]
    for filename, values, ylabel in (
        ("all_pitch_down_displacement_vs_phase_offset.png", displacement, "Signed forward displacement (m)"),
        ("all_pitch_down_actual_contact_vs_phase_offset.png", contact, "Actual-contact fraction"),
    ):
        fig, ax = plt.subplots(figsize=(6.5, 4.3))
        ax.plot(np.arange(len(waves)), values, "o-", color="#d97732")
        ax.axhline(0, color="#777777", linewidth=0.8)
        ax.set(xticks=np.arange(len(waves)), xticklabels=[f"{value:.0f}" for value in phases],
               xlabel="Locked phase offset (deg)", ylabel=ylabel)
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(figures / filename, dpi=260)
        plt.close(fig)
    iid = sorted(
        [row for row in iid_summary if int(row["n_segments"]) == 10],
        key=lambda row: float(row["N_wave_ref10"]),
    )
    mean = np.asarray([float(row["mean_forward_displacement_m"]) for row in iid])
    low = np.asarray([float(row["bootstrap_mean_ci95_low_m"]) for row in iid])
    high = np.asarray([float(row["bootstrap_mean_ci95_high_m"]) for row in iid])
    fig, ax = plt.subplots(figsize=(6.7, 4.5))
    ax.errorbar(waves, mean, yerr=np.vstack((mean-low, high-mean)), fmt="o-", capsize=3,
                label="IID: mean + 95% CI", color="#177e89")
    ax.plot(waves, displacement, "s-", label="All pitch down: deterministic", color="#d97732")
    ax.axhline(0, color="#777777", linewidth=0.8)
    ax.set(xlabel=r"Reference label, $N_{wave}^{(10)}$", ylabel="Signed forward displacement (m)", xticks=waves)
    ax.grid(alpha=0.2)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(figures / "N10_IID_vs_all_pitch_down_displacement.png", dpi=260)
    plt.close(fig)


def main() -> None:
    root = step37.repo_root()
    parent = root / "exp_data" / "reliability" / step37.OUTPUT_NAME
    output = parent / "all_pitch_down_reference"
    output.mkdir(parents=True, exist_ok=True)
    rows, series = [], []
    for wave in step37.REFERENCE_WAVES:
        row, trajectory = run_one(root, output, wave)
        rows.append(row)
        series.extend(trajectory)
        step37.write_csv(output / "all_pitch_down_reference.csv", rows)
        print(f"all-down wave={wave:g}, D={row['forward_displacement_m']:+.4f} m, contact={row['mean_actual_contact_fraction']:.3f}, {row['status']}", flush=True)
    step37.write_csv(output / "all_pitch_down_time_series.csv", series)
    make_figures(output, rows, step37.read_csv(parent / "iid_summary.csv"))
    validation = {
        "passed": len(rows) == len(step37.REFERENCE_WAVES) and all(row["status"] == "PASS" for row in rows),
        "expected_conditions": len(step37.REFERENCE_WAVES), "condition_count": len(rows),
        "all_down_is_command_not_forced_contact": True,
        "mujoco_version": rows[0]["mujoco_version"],
        "maximum_warning_count": max(row["mujoco_warning_total"] for row in rows),
        "comparison_runtime_caveat": "The earlier IID sweep did not record its MuJoCo version; the overlay is descriptive, not a matched-runtime causal effect estimate.",
    }
    step37.write_json(output / "validation.json", validation)
    step37.write_json(output / "run_configuration.json", {
        "n_segments": 10, "reference_wave_labels": list(step37.REFERENCE_WAVES),
        "locked_phase_formula": "Delta_phi=-2*pi*N_wave_ref10/9",
        "body_yaw_half_amplitude_deg": step37.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_PD": step37.PD_CANDIDATES[step37.FULL_PD_NAME],
        "leg_yaw_fixed": True, "leg_pitch_after_settle": "all legs always commanded down",
        "pitch_down_deg": LEG_DOWN_DEG, "warmup_cycles": media.WARMUP_CYCLES,
        "evaluation_cycles": media.EVALUATION_CYCLES,
        "mujoco_version": rows[0]["mujoco_version"],
    })
    (output / "README.md").write_text(
        "# All-pitch-down reference\n\n"
        "N=10; seven locked phase offsets defined on the ten-segment reference. "
        "Body-yaw half-amplitude is 19 deg, the stronger moderate PD is unchanged, "
        "and leg yaw is fixed. Every leg remains commanded down after settling. "
        "This is deterministic, not IID, and ground contact is not artificially forced.\n\n"
        "Each reference has four warm-up cycles and twelve evaluated cycles. "
        "The CSV records signed COM displacement and realized contact fraction; "
        "the videos show the full evaluation.\n\n"
        "The IID overlay is descriptive only: the previous IID sweep omitted its "
        "MuJoCo version, while these references explicitly use 3.3.7.\n",
        encoding="utf-8",
    )
    if not validation["passed"]:
        raise RuntimeError(f"all-down reference failed: {validation}")


if __name__ == "__main__":
    main()
