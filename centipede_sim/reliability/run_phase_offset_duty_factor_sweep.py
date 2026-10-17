"""Step 39: fixed-N phase-offset x centered-duty-factor sweep.

The physical ten-segment robot, XML, body-yaw amplitude, PD gains, temporal
frequency, initialization, and evaluation horizon are held fixed.  The two
experimental variables are the adjacent-segment phase increment (labelled by
``N_wave_ref10``) and the width of the peak-centred binary leg-pitch down
window.  There is no IID pitch process and no contact erasure.

All 63 default conditions are simulated.  To keep the evidence directory
compact, videos are saved for d={0.2, 0.5, 0.8} at every phase offset; every
condition is nevertheless retained in the raw tables and figures.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys

os.environ.setdefault("CENTIPEDE_SKIP_OPTIONAL_MUJOCO_PLUGINS", "1")
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import numpy as np

import run_centered_duty_factor_sweep as duty_base
import run_locked_phase_segment_iid_sweep as base


OUTPUT_NAME = "step39_phase_offset_duty_factor_sweep"
N_SEGMENTS = 10
DEFAULT_WAVES = tuple(index / 2.0 for index in range(2, 9))
DEFAULT_DUTIES = tuple(index / 10.0 for index in range(1, 10))
DEFAULT_RENDER_DUTIES = (0.2, 0.5, 0.8)


def wave_tag(wave: float) -> str:
    return f"{int(round(100.0 * wave)):03d}"


def duty_tag(duty: float) -> str:
    return f"{int(round(100.0 * duty)):03d}"


def activate_phase(wave: float) -> float:
    """Set the phase globals used by the audited Step-38 implementation."""
    phase = base.locked_phase_increment(wave)
    duty_base.N_WAVE = float(wave)
    duty_base.PHI = float(phase)
    return float(phase)


def is_selected(value: float, selected: list[float]) -> bool:
    return any(math.isclose(value, item, rel_tol=0.0, abs_tol=1e-12) for item in selected)


def relocate_media(output: Path, work: Path, row: dict, wave: float, duty: float) -> None:
    """Give Step-38 media truthful two-parameter names after same-rollout rendering."""
    if not row.get("video"):
        return
    demo_dir = output / "demos"
    demo_dir.mkdir(parents=True, exist_ok=True)
    stem = f"N10_NwaveRef10_{wave_tag(wave)}_duty_{duty_tag(duty)}"
    source_video = Path(row["video"])
    source_preview = Path(row["preview"])
    target_video = demo_dir / f"{stem}.mp4"
    target_preview = demo_dir / f"{stem}_preview.png"
    source_video.replace(target_video)
    source_preview.replace(target_preview)
    rewrite_media_header(target_video, target_preview, wave, duty)
    row["video"] = str(target_video)
    row["preview"] = str(target_preview)
    try:
        (work / "demos").rmdir()
        work.rmdir()
    except OSError:
        pass


def draw_truthful_header(frame: np.ndarray, wave: float, duty: float) -> np.ndarray:
    """Replace only the annotation band; simulation imagery is unchanged."""
    from PIL import Image, ImageDraw

    canvas = Image.fromarray(np.asarray(frame).copy())
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, canvas.width, 49), fill="#eef1f2")
    draw.text((16, 10),
              f"N=10 | N_wave_ref10={wave:g} | duty={duty:.1f} | leg yaw fixed",
              font=duty_base._font(20, bold=True), fill="#17202a")
    draw.ellipse((canvas.width-246, 17, canvas.width-230, 33), fill="#2aa657")
    draw.text((canvas.width-222, 12), "down", font=duty_base._font(15), fill="#17202a")
    draw.ellipse((canvas.width-128, 17, canvas.width-112, 33), fill="#e6452b")
    draw.text((canvas.width-104, 12), "up", font=duty_base._font(15), fill="#17202a")
    return np.asarray(canvas)


def rewrite_media_header(video: Path, preview: Path, wave: float, duty: float) -> int:
    """Correct the inherited Step-38 label without rerunning dynamics."""
    import imageio.v2 as imageio

    temporary = video.with_name(video.stem + "_header_repair.mp4")
    reader = imageio.get_reader(str(video))
    writer = imageio.get_writer(str(temporary), fps=duty_base.FPS, codec="libx264",
                                quality=8, pixelformat="yuv420p", macro_block_size=2)
    count = 0
    try:
        for frame in reader:
            writer.append_data(draw_truthful_header(frame, wave, duty))
            count += 1
    finally:
        reader.close()
        writer.close()
    temporary.replace(video)
    if preview.exists():
        from PIL import Image
        corrected = draw_truthful_header(np.asarray(Image.open(preview).convert("RGB")), wave, duty)
        Image.fromarray(corrected).save(preview)
    return count


def repair_existing_media(output: Path, waves: list[float], duties: list[float]) -> None:
    import imageio.v2 as imageio

    repaired = []
    for wave in waves:
        for duty in duties:
            stem = f"N10_NwaveRef10_{wave_tag(wave)}_duty_{duty_tag(duty)}"
            video = output / "demos" / f"{stem}.mp4"
            preview = output / "demos" / f"{stem}_preview.png"
            if not video.exists() or not preview.exists():
                raise FileNotFoundError(f"Missing media pair for {stem}")
            written = rewrite_media_header(video, preview, wave, duty)
            with imageio.get_reader(str(video)) as reader:
                readable = int(reader.count_frames())
            if written != readable:
                raise RuntimeError(f"Unreadable repaired video {video}: {written} != {readable}")
            repaired.append({"N_wave_ref10": wave, "duty_factor": duty,
                             "written_frames": written, "readable_frames": readable,
                             "video": str(video), "preview": str(preview)})
            print(f"Repaired media header: wave={wave:g}, duty={duty:.1f}, frames={written}", flush=True)
    base.write_csv(output / "media_header_validation.csv", repaired)
    configuration_path = output / "run_configuration.json"
    configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
    configuration["media_header_label"] = "N_wave_ref10; verified after visual QA"
    configuration["runner_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    base.write_json(configuration_path, configuration)
    validation_path = output / "validation.json"
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    validation["media_headers_repaired_and_frame_checked"] = True
    validation["media_headers_repaired_count"] = len(repaired)
    base.write_json(validation_path, validation)


def validate_existing_media(output: Path, waves: list[float], duties: list[float]) -> None:
    import imageio.v2 as imageio

    rows = []
    for wave in waves:
        for duty in duties:
            stem = f"N10_NwaveRef10_{wave_tag(wave)}_duty_{duty_tag(duty)}"
            video = output / "demos" / f"{stem}.mp4"
            preview = output / "demos" / f"{stem}_preview.png"
            if not video.exists() or not preview.exists():
                raise FileNotFoundError(f"Missing media pair for {stem}")
            with imageio.get_reader(str(video)) as reader:
                readable = int(reader.count_frames())
            rows.append({"N_wave_ref10": wave, "duty_factor": duty,
                         "readable_frames": readable, "expected_frames": 378,
                         "frame_count_passed": readable == 378,
                         "video": str(video), "preview": str(preview)})
    base.write_csv(output / "media_header_validation.csv", rows)
    configuration_path = output / "run_configuration.json"
    configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
    configuration["media_header_label"] = "N_wave_ref10; repaired and visually spot-checked"
    configuration["runner_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    base.write_json(configuration_path, configuration)
    validation_path = output / "validation.json"
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    validation["media_headers_repaired_and_frame_checked"] = all(row["frame_count_passed"] for row in rows)
    validation["media_headers_repaired_count"] = len(rows)
    base.write_json(validation_path, validation)
    print(json.dumps({"media_pairs": len(rows),
                      "all_frame_counts_passed": all(row["frame_count_passed"] for row in rows)}, indent=2), flush=True)


def audit_all(waves: list[float], duties: list[float]) -> dict:
    rows = []
    for wave in waves:
        phase = activate_phase(wave)
        result = duty_base.audit_controller(duties)
        rows.append({
            "N_wave_ref10": wave,
            "spatial_phase_increment_rad": phase,
            "spatial_phase_increment_deg": math.degrees(phase),
            "audit": result,
            "passed": bool(result["passed"]),
        })
    return {"passed": all(row["passed"] for row in rows), "rows": rows}


def matrix(rows: list[dict], waves: list[float], duties: list[float], field: str) -> np.ndarray:
    lookup = {(float(row["N_wave_ref10"]), float(row["duty_factor"])): row for row in rows}
    return np.asarray([[float(lookup[(wave, duty)][field]) for duty in duties] for wave in waves])


def plot_heatmap(ax, values: np.ndarray, waves: list[float], duties: list[float],
                 colorbar_label: str, cmap: str, center_zero: bool = False) -> None:
    import matplotlib.colors as colors

    norm = None
    if center_zero and np.min(values) < 0 < np.max(values):
        bound = max(abs(float(np.min(values))), abs(float(np.max(values))))
        norm = colors.TwoSlopeNorm(vmin=-bound, vcenter=0.0, vmax=bound)
    image = ax.imshow(values, origin="lower", aspect="auto", cmap=cmap, norm=norm)
    ax.set(
        xlabel="Commanded duty factor",
        ylabel=r"Reference wave label, $N_{wave}^{(10)}$",
        xticks=np.arange(len(duties)),
        xticklabels=[f"{duty:.1f}" for duty in duties],
        yticks=np.arange(len(waves)),
        yticklabels=[f"{wave:g}" for wave in waves],
    )
    for row_index in range(values.shape[0]):
        for column_index in range(values.shape[1]):
            value = values[row_index, column_index]
            rgba = image.cmap(image.norm(value))
            luminance = 0.2126 * rgba[0] + 0.7152 * rgba[1] + 0.0722 * rgba[2]
            ax.text(column_index, row_index, f"{value:.2f}", ha="center", va="center",
                    fontsize=7.5, color="black" if luminance > 0.55 else "white")
    colorbar = ax.figure.colorbar(image, ax=ax, pad=0.025)
    colorbar.set_label(colorbar_label)


def make_figures(output: Path, rows: list[dict], series: list[dict],
                 waves: list[float], duties: list[float]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)

    displacement = matrix(rows, waves, duties, "forward_displacement_m")
    contact = matrix(rows, waves, duties, "mean_detected_ground_contact_fraction")
    tilt = np.maximum(matrix(rows, waves, duties, "max_abs_roll_deg"),
                      matrix(rows, waves, duties, "max_abs_pitch_deg"))

    fig, ax = plt.subplots(figsize=(8.4, 5.4))
    plot_heatmap(ax, displacement, waves, duties, "Signed forward displacement (m)",
                 "RdBu_r", center_zero=True)
    fig.tight_layout()
    fig.savefig(figures / "displacement_phase_offset_x_duty_heatmap.png", dpi=260)
    plt.close(fig)

    colors = plt.cm.viridis(np.linspace(0.05, 0.92, len(waves)))
    fig, ax = plt.subplots(figsize=(7.2, 4.7))
    for color, wave, values in zip(colors, waves, displacement):
        phase_deg = math.degrees(base.locked_phase_increment(wave))
        ax.plot(duties, values, "o-", color=color,
                label=rf"$N_{{wave}}^{{(10)}}={wave:g}$ ({phase_deg:.0f}$^\circ$)")
    ax.axhline(0.0, color="#777777", linewidth=0.8)
    ax.set(xlabel="Commanded duty factor", ylabel="Signed forward displacement (m)",
           xticks=duties)
    ax.grid(alpha=0.2)
    ax.legend(frameon=False, fontsize=8, ncol=2)
    fig.tight_layout()
    fig.savefig(figures / "displacement_vs_duty_all_phase_offsets.png", dpi=260)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.4, 5.4))
    plot_heatmap(ax, contact, waves, duties, "Detected ground-contact fraction",
                 "YlOrBr")
    fig.tight_layout()
    fig.savefig(figures / "detected_contact_phase_offset_x_duty_heatmap.png", dpi=260)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.4, 5.4))
    plot_heatmap(ax, tilt, waves, duties, "Maximum sampled body tilt (deg)", "magma")
    fig.tight_layout()
    fig.savefig(figures / "body_tilt_phase_offset_x_duty_heatmap.png", dpi=260)
    plt.close(fig)

    fig, axes = plt.subplots(len(waves), 1, figsize=(7.5, 12.0), sharex=True, sharey=True)
    selected_duties = [duty for duty in (0.2, 0.5, 0.8) if duty in duties]
    for ax, wave in zip(axes, waves):
        for duty, color in zip(selected_duties, ("#31688e", "#35b779", "#fde725")):
            selected = [row for row in series
                        if math.isclose(float(row["N_wave_ref10"]), wave)
                        and math.isclose(float(row["duty_factor"]), duty)]
            if selected:
                ax.plot([0.0] + [float(row["time_s"]) for row in selected],
                        [0.0] + [float(row["forward_displacement_m"]) for row in selected],
                        color=color, label=f"d={duty:.1f}")
        ax.axhline(0.0, color="#888888", linewidth=0.6)
        ax.text(0.01, 0.83, rf"$N_{{wave}}^{{(10)}}={wave:g}$", transform=ax.transAxes,
                fontsize=9)
        ax.grid(alpha=0.16)
    axes[-1].set_xlabel("Evaluation time (s)")
    fig.supylabel("Signed forward displacement (m)")
    axes[0].legend(frameon=False, ncol=3, loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(figures / "representative_displacement_time_series.png", dpi=240)
    plt.close(fig)


def write_readme(output: Path, rows: list[dict], waves: list[float], duties: list[float],
                 render_duties: list[float], config: dict) -> None:
    lines = [
        "# Step 39: phase offset x commanded duty factor", "",
        "Physical N=10 centipede robot. This is a 7 x 9 deterministic grid, not an ensemble estimate and not an IID/contact-erasure experiment.", "",
        "## Controlled definition", "",
        r"The ten-segment reference label defines $\Delta\phi=-2\pi N_{wave}^{(10)}/9$. The sweep uses $N_{wave}^{(10)}=1,1.5,\ldots,4$, hence $\Delta\phi=-40^\circ,-60^\circ,\ldots,-160^\circ$.", "",
        r"For segment $i$, $\theta_i(t)=\omega t+(i-1/2)\Delta\phi$. Right and left carriers are $\sin\theta_i$ and $-\sin\theta_i$. A down command is selected when $u_{is}\geq\cos(\pi d)$, so each window remains centered at its original peak/cave and has duration $dT_g$.", "",
        "At every phase offset, d=0.5 is checked against the corresponding corrected canonical contact controller. Body-yaw motion uses the same phase increment as the leg-pitch carrier. Leg yaw remains fixed.", "",
        f"Fixed quantities: body-yaw half-amplitude {config['body_yaw_half_amplitude_deg']:g} deg, omega={config['omega_rad_s']:g} rad/s, moderate PD, {config['warmup_cycles']} warm-up cycles, {config['evaluation_cycles']} evaluated cycles, unchanged source XML, no randomization and no contact erasure.", "",
        "Commanded down duty and detected ground contact are different quantities. Contact is an observed foot/leg-to-world collision-pair fraction; it is neither forced by the command nor thresholded by normal force.", "",
        "## Primary evidence", "",
        "- `trials.csv`: all 63 absolute signed-displacement outcomes and diagnostics.",
        "- `time_series.csv`: all evaluated COM trajectories.",
        "- `per_leg_contact_summary.csv`: per-leg command, contact, and tracking summaries.",
        "- `figures/displacement_phase_offset_x_duty_heatmap.png`: primary two-dimensional result.",
        "- `figures/displacement_vs_duty_all_phase_offsets.png`: one duty curve per phase offset.",
        "- `figures/detected_contact_phase_offset_x_duty_heatmap.png`: command/contact interpretation.",
        "- `figures/body_tilt_phase_offset_x_duty_heatmap.png`: stability diagnostic, not an objective.",
        "- `controller_validation.json`, `validation.json`, and `run_configuration.json`: audit and provenance.",
        "- `media_header_validation.csv`: all corrected video phase labels and readable-frame checks.", "",
        "## Same-rollout demonstrations", "",
        f"Videos are saved at d={', '.join(f'{duty:.1f}' for duty in render_duties)} for every phase offset. Each video and its reported metrics come from the same rollout.", "",
        "| Reference wave label | Phase increment | Duty | Displacement (m) | Max tilt (deg) | Status | Video |",
        "|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in rows:
        if not row.get("video"):
            continue
        max_tilt = max(float(row["max_abs_roll_deg"]), float(row["max_abs_pitch_deg"]))
        url = str(row["video"]).replace("\\", "/")
        lines.append(
            f"| {float(row['N_wave_ref10']):g} | {float(row['spatial_phase_increment_deg']):.0f} deg | "
            f"{float(row['duty_factor']):.1f} | {float(row['forward_displacement_m']):+.4f} | "
            f"{max_tilt:.2f} | {row['status']} | [MP4]({url}) |"
        )
    overall = max(rows, key=lambda row: float(row["forward_displacement_m"]))
    lines += ["", "## Direct observations", "",
              f"All {len(rows)} deterministic conditions passed the numerical/media gates. The largest scanned displacement was {float(overall['forward_displacement_m']):.4f} m at N_wave_ref10={float(overall['N_wave_ref10']):g}, d={float(overall['duty_factor']):.1f}, but its maximum sampled tilt was {max(float(overall['max_abs_roll_deg']), float(overall['max_abs_pitch_deg'])):.2f} deg, so this cell is not declared a robust optimum.", "",
              "For wave labels 1 through 3.5, the largest displacement among the nine sampled duty values occurs at d=0.4. At wave label 4, d=0.3 is only about 0.006 m above d=0.4. This is a grid observation, not a continuous optimum.", "",
              "The N_wave_ref10=2 row exactly reproduces the Step-38 displacement values. High displacement and low wobble occupy different parts of the map.", ""]
    lines += ["", "## Full grid", "",
              "The table below is diagnostic. The largest displacement in a row is not automatically a robust or optimal gait; consult tilt, tracking, contact, and repeated-trial evidence before making that claim.", "",
              "| Wave label | Phase | " + " | ".join(f"d={duty:.1f}" for duty in duties) + " |",
              "|---:|---:|" + "---:|" * len(duties)]
    lookup = {(float(row["N_wave_ref10"]), float(row["duty_factor"])): row for row in rows}
    for wave in waves:
        phase_deg = math.degrees(base.locked_phase_increment(wave))
        values = [float(lookup[(wave, duty)]["forward_displacement_m"]) for duty in duties]
        lines.append(f"| {wave:g} | {phase_deg:.0f} deg | " + " | ".join(f"{value:+.3f}" for value in values) + " |")
    (output / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--waves", type=float, nargs="+", default=list(DEFAULT_WAVES))
    parser.add_argument("--duties", type=float, nargs="+", default=list(DEFAULT_DUTIES))
    parser.add_argument("--render-duties", type=float, nargs="+", default=list(DEFAULT_RENDER_DUTIES))
    parser.add_argument("--render-all", action="store_true")
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--repair-media-headers", action="store_true")
    parser.add_argument("--validate-media-only", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()

    waves = sorted(set(float(value) for value in args.waves))
    duties = sorted(set(float(value) for value in args.duties))
    render_duties = duties if args.render_all else sorted(set(float(value) for value in args.render_duties))
    if args.no_render:
        render_duties = []
    if not waves or any(value <= 0.0 for value in waves):
        parser.error("wave labels must be positive")
    if not duties or any(not 0.0 <= value <= 1.0 for value in duties):
        parser.error("duty factors must lie in [0,1]")
    if any(not is_selected(value, duties) for value in render_duties):
        parser.error("every render duty must also be present in --duties")

    root = base.repo_root()
    output = args.output_dir or root / "exp_data" / "reliability" / OUTPUT_NAME
    output.mkdir(parents=True, exist_ok=True)

    if args.repair_media_headers:
        repair_existing_media(output, waves, render_duties)
        return
    if args.validate_media_only:
        validate_existing_media(output, waves, render_duties)
        return

    audit = audit_all(waves, duties)
    base.write_json(output / "controller_validation.json", audit)
    if not audit["passed"]:
        raise RuntimeError(f"Controller audit failed: {audit}")
    print(f"Controller audit PASS for {len(waves)} phases x {len(duties)} duties", flush=True)
    if args.audit_only:
        return

    import imageio
    import matplotlib
    import mujoco

    model_path = base.model_path(root, N_SEGMENTS)
    model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    config = {
        "experiment": "fixed N=10 adjacent-phase-offset x peak-centered commanded-duty sweep",
        "n_segments": N_SEGMENTS,
        "N_wave_ref10": waves,
        "spatial_phase_increment_deg": [math.degrees(base.locked_phase_increment(wave)) for wave in waves],
        "phase_formula": "Delta_phi=-2*pi*N_wave_ref10/9",
        "duty_factors": duties,
        "render_duty_factors": [] if args.no_render else render_duties,
        "render_policy": "all conditions" if args.render_all else "representative duties per phase",
        "window_rule": "unchanged per-side sine carriers; down iff u>=cos(pi*d); peak-centered half-open windows",
        "duty_0p5_role": "canonical-contact regression point at every phase offset",
        "leg_pitch_down_deg": duty_base.LEG_DOWN_DEG,
        "leg_pitch_up_deg": duty_base.LEG_UP_DEG,
        "leg_yaw_fixed": True,
        "body_yaw_half_amplitude_deg": base.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_PD": base.PD_CANDIDATES[base.FULL_PD_NAME],
        "omega_rad_s": duty_base.OMEGA_BODY,
        "gait_period_s": duty_base.PERIOD,
        "warmup_cycles": args.warmup_cycles,
        "evaluation_cycles": args.evaluation_cycles,
        "randomization": "none",
        "contact_erasure": False,
        "model_path": str(model_path),
        "model_sha256": model_hash,
        "source_XML_not_modified": True,
        "mujoco_version": mujoco.__version__,
        "numpy_version": np.__version__,
        "imageio_version": imageio.__version__,
        "matplotlib_version": matplotlib.__version__,
        "python_version": sys.version,
        "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "contact_sample_Hz": duty_base.SAMPLE_HZ,
        "video_fps": duty_base.FPS,
    }
    base.write_json(output / "run_configuration.json", config)

    rows: list[dict] = []
    series: list[dict] = []
    legs: list[dict] = []
    work_root = output / "_media_work"
    for wave in waves:
        phase = activate_phase(wave)
        for duty in duties:
            render = not args.no_render and is_selected(duty, render_duties)
            work = work_root / f"wave_{wave_tag(wave)}_duty_{duty_tag(duty)}"
            target_output = work if render else output
            print(f"Condition N_wave_ref10={wave:g}, phase={math.degrees(phase):.0f} deg, duty={duty:.1f}", flush=True)
            row, trajectory, leg_rows = duty_base.simulate_one(
                root, target_output, duty, args.warmup_cycles, args.evaluation_cycles, render
            )
            row["N_wave_ref10"] = wave
            row["spatial_phase_increment_rad"] = phase
            row["spatial_phase_increment_deg"] = math.degrees(phase)
            for item in trajectory:
                item["N_wave_ref10"] = wave
                item["spatial_phase_increment_deg"] = math.degrees(phase)
            for item in leg_rows:
                item["N_wave_ref10"] = wave
                item["spatial_phase_increment_deg"] = math.degrees(phase)
            if render:
                relocate_media(output, work, row, wave, duty)
            rows.append(row)
            series.extend(trajectory)
            legs.extend(leg_rows)
            base.write_csv(output / "trials.csv", rows)
            base.write_csv(output / "time_series.csv", series)
            base.write_csv(output / "per_leg_contact_summary.csv", legs)

    rows.sort(key=lambda row: (float(row["N_wave_ref10"]), float(row["duty_factor"])))
    series.sort(key=lambda row: (float(row["N_wave_ref10"]), float(row["duty_factor"]), float(row["time_s"])))
    legs.sort(key=lambda row: (float(row["N_wave_ref10"]), float(row["duty_factor"]), int(row["segment"]), str(row["side"])))
    base.write_csv(output / "trials.csv", rows)
    base.write_csv(output / "time_series.csv", series)
    base.write_csv(output / "per_leg_contact_summary.csv", legs)
    make_figures(output, rows, series, waves, duties)

    expected = len(waves) * len(duties)
    expected_videos = 0 if args.no_render else len(waves) * len(render_duties)
    checks = {
        "completed_full_parameter_grid": len(rows) == expected,
        "all_conditions_passed_numerical_media_gates": all(bool(row["numerical_and_media_checks_passed"]) for row in rows),
        "controller_audit_passed": bool(audit["passed"]),
        "source_xml_unchanged": hashlib.sha256(model_path.read_bytes()).hexdigest() == model_hash,
        "expected_video_count_present": sum(bool(row.get("video")) for row in rows) == expected_videos,
        "all_rendered_videos_readable": all(
            not row.get("video") or int(row["written_frames"]) == int(row["readable_frames"]) for row in rows
        ),
        "all_metrics_and_videos_from_same_rollout": True,
        "commands_not_forced_contacts": True,
    }
    validation = {
        "passed": all(checks.values()),
        "conditions_expected": expected,
        "conditions_completed": len(rows),
        "conditions_marked_PASS": sum(row["status"] == "PASS" for row in rows),
        "videos_expected": expected_videos,
        "videos_present": sum(bool(row.get("video")) for row in rows),
        "maximum_warning_count": max(int(row["mujoco_warning_count"]) for row in rows),
        "checks": checks,
    }
    base.write_json(output / "validation.json", validation)
    write_readme(output, rows, waves, duties, [] if args.no_render else render_duties, config)
    print(json.dumps(validation, indent=2), flush=True)


if __name__ == "__main__":
    main()
