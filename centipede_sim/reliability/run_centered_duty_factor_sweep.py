"""Step 38: peak-centered duty-factor sweep, physical N=10, N_wave=2.

Preserve the corrected cave-collocated sinusoidal phase carrier. Change only
the width of the binary down-command window around each side's original
stance center. No IID, contact erasure, time warping, leg-yaw motion, or XML
edits. At d=0.5 the pitch commands recover the corrected canonical pattern.
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

import cave_grouped_wave_controller as cave
import run_locked_phase_segment_iid_sweep as base
from run_independent_random_leg_phases import (
    BRIDGE_TIME_S, LEG_DOWN_DEG, LEG_OFFSET_DEG, LEG_UP_DEG,
    OMEGA_BODY, SETTLE_TIME_S, _actual_contact_flags, _font,
    _root_roll_pitch_deg, _set_leg_colors,
)


OUTPUT_NAME = "step38_centered_duty_factor_sweep"
N_SEGMENTS = 10
N_WAVE = 2.0
PHI = base.locked_phase_increment(N_WAVE)
PERIOD = 2.0 * math.pi / OMEGA_BODY
DEFAULT_DUTIES = tuple(index / 10.0 for index in range(1, 10))
WIDTH, HEIGHT, FPS = 960, 540, 20
SAMPLE_HZ = 50.0


def duty_tag(duty: float) -> str:
    return f"{int(round(100*duty)):03d}"


def down_flags_from_phase(theta: np.ndarray, duty: float) -> np.ndarray:
    """Return [..., segment, side(R,L)] binary commands.

    Center R on theta=pi/2, L on theta=3*pi/2. The common sinusoid is
    unchanged. Half-open windows include their opening and exclude closing;
    the inherited 1e-12 phase tolerance resolves exact crossing ties.
    """
    if not 0.0 <= duty <= 1.0:
        raise ValueError("duty factor must be in [0,1]")
    theta = np.asarray(theta, dtype=float)
    if duty == 0.0:
        return np.zeros(theta.shape + (2,), dtype=bool)
    if duty == 1.0:
        return np.ones(theta.shape + (2,), dtype=bool)
    if duty == 0.5:
        right = np.floor((theta + cave.CAVE_BOUNDARY_TOL) / math.pi).astype(np.int64) % 2 == 0
        return np.stack((right, ~right), axis=-1)
    side_phase = theta[..., None] + np.array([0.0, math.pi])
    opening = math.pi / 2.0 - math.pi * duty
    elapsed = np.remainder(side_phase - opening + cave.CAVE_BOUNDARY_TOL, 2.0*math.pi)
    return elapsed < 2.0 * math.pi * duty


def stance_flags(body_times: np.ndarray, duty: float) -> np.ndarray:
    theta = OMEGA_BODY * np.asarray(body_times)[..., None] + (np.arange(N_SEGMENTS)-0.5)*PHI
    return down_flags_from_phase(theta, duty)


def dynamic_controls(body_times: np.ndarray, duty: float, indices: tuple, n_ctrl: int) -> np.ndarray:
    body_times = np.atleast_1d(body_times).astype(float)
    sr, pr, sl, pl, by, bp = indices
    controls = np.zeros((body_times.size, n_ctrl), dtype=float)
    neutral = math.radians(LEG_OFFSET_DEG)
    controls[:, sr] = neutral
    controls[:, sl] = -neutral
    controls[:, by] = math.radians(base.BODY_YAW_HALF_AMPLITUDE_DEG) * np.sin(
        OMEGA_BODY*body_times[:, None] + np.arange(N_SEGMENTS-1)*PHI
    )
    controls[:, bp] = 0.0
    flags = stance_flags(body_times, duty)
    down, up = math.radians(LEG_DOWN_DEG), math.radians(LEG_UP_DEG)
    controls[:, pr] = np.where(flags[:, :, 0], down, up)
    controls[:, pl] = np.where(flags[:, :, 1], down, up)
    return controls


def audit_controller(duties: list[float]) -> dict:
    indices = base.step33._build_actuator_indices(N_SEGMENTS)
    n_ctrl = 4*N_SEGMENTS + 2*(N_SEGMENTS-1)
    sr, pr, sl, pl, by, bp = indices
    original = cave.CaveGroupedContactPatternController(
        N_SEGMENTS, n_ctrl, indices, None, n_wave=N_WAVE,
        global_phase_deg=0.0, flipped=False,
    )
    sample_count = 20_000
    times = (np.arange(sample_count)+0.371)/sample_count*PERIOD
    phase = OMEGA_BODY*times[:, None] + (np.arange(N_SEGMENTS)-0.5)*PHI
    rows = []
    for duty in duties:
        flags = stance_flags(times, duty)
        fractions = flags.mean(axis=0)
        periodic = np.array_equal(flags, stance_flags(times+PERIOD, duty))
        side_shift = np.array_equal(flags[:, :, 1], stance_flags(times+PERIOD/2.0, duty)[:, :, 0])
        side_sine = np.sin(phase[..., None] + np.array([0.0, math.pi]))
        threshold = math.cos(math.pi*duty)
        away_from_boundary = np.abs(side_sine-threshold) > 1e-9
        threshold_agrees = bool(np.all(flags[away_from_boundary] == (side_sine >= threshold)[away_from_boundary]))
        both_up = float(np.mean(~flags[:, :, 0] & ~flags[:, :, 1]))
        both_down = float(np.mean(flags[:, :, 0] & flags[:, :, 1]))
        # Reflection around a peak preserves the window except exactly at its boundaries.
        probe = np.linspace(-math.pi, math.pi, 6001) + 0.000017
        mirrored_a = down_flags_from_phase((math.pi/2+probe)[:, None], duty)[:, 0, 0]
        mirrored_b = down_flags_from_phase((math.pi/2-probe)[:, None], duty)[:, 0, 0]
        commands = dynamic_controls(times[:100], duty, indices, n_ctrl)
        pitch = commands[:, np.r_[pr, pl]]
        checks = {
            "periodic": periodic,
            "left_center_shifted_by_half_period": side_shift,
            "sine_carrier_threshold_equivalence": threshold_agrees,
            "windows_centered_without_phase_shift": bool(np.array_equal(mirrored_a, mirrored_b)),
            "each_leg_has_requested_duty": bool(np.max(np.abs(fractions-duty)) <= 2/sample_count),
            "bilateral_gap_fraction_correct": abs(both_up-max(0.0, 1-2*duty)) <= 2/sample_count,
            "bilateral_overlap_fraction_correct": abs(both_down-max(0.0, 2*duty-1)) <= 2/sample_count,
            "binary_pitch_targets": bool(np.all(np.isclose(pitch, math.radians(LEG_DOWN_DEG)) | np.isclose(pitch, math.radians(LEG_UP_DEG)))),
            "leg_yaw_fixed": bool(np.allclose(commands[:, sr], math.radians(LEG_OFFSET_DEG)) and np.allclose(commands[:, sl], -math.radians(LEG_OFFSET_DEG))),
            "body_pitch_fixed": bool(np.allclose(commands[:, bp], 0.0)),
        }
        rows.append({"duty_factor": duty, "min_leg_duty": float(fractions.min()),
                     "max_leg_duty": float(fractions.max()), "both_up_fraction": both_up,
                     "both_down_fraction": both_down, **checks, "passed": all(checks.values())})
    # The d=0.5 baseline must match the actual corrected controller, including boundaries.
    reference_times = list(np.linspace(0, PERIOD, 721, endpoint=False))
    for segment in range(N_SEGMENTS):
        offset = (segment-0.5)*PHI
        for half_wave in range(-10, 11):
            crossing = (half_wave*math.pi-offset)/OMEGA_BODY
            if 0 <= crossing <= PERIOD:
                reference_times.extend([crossing-1e-9, crossing, crossing+1e-9])
    matches_original = all(
        np.array_equal(stance_flags(np.array([time]), 0.5)[0], original.stance_flags(time))
        for time in reference_times
    )
    expected_body = dynamic_controls(times[:100], 0.5, indices, n_ctrl)[:, by]
    unchanged_body = all(np.array_equal(dynamic_controls(times[:100], duty, indices, n_ctrl)[:, by], expected_body) for duty in duties)
    return {"passed": bool(matches_original and unchanged_body and all(row["passed"] for row in rows)),
            "duty_0p5_matches_corrected_original_pitch": bool(matches_original),
            "original_comparison_samples": len(reference_times),
            "body_yaw_identical_across_duties": bool(unchanged_body), "rows": rows}


def build_controls(assets, duty: float, warmup_cycles: int, evaluation_cycles: int):
    settle_steps = math.ceil(SETTLE_TIME_S/assets.dt)
    bridge_steps = math.ceil(BRIDGE_TIME_S/assets.dt)
    warmup_steps = math.ceil(warmup_cycles*PERIOD/assets.dt)
    eval_steps = math.ceil(evaluation_cycles*PERIOD/assets.dt)
    start = settle_steps + bridge_steps + warmup_steps
    initial = base.initial_target(assets.model, assets.indices)
    settle = (np.arange(settle_steps)+1)[:, None]/settle_steps*initial[None, :]
    target_zero = dynamic_controls(np.array([0.0]), duty, assets.indices, assets.model.nu)[0]
    bridge_alpha = 0.5-0.5*np.cos(math.pi*(np.arange(bridge_steps)+1)/bridge_steps)
    bridge = initial[None, :] + bridge_alpha[:, None]*(target_zero-initial)[None, :]
    body_times = np.arange(warmup_steps+eval_steps)*assets.dt
    dynamic = dynamic_controls(body_times, duty, assets.indices, assets.model.nu)
    controls = np.concatenate((settle, bridge, dynamic))[None, :, :]
    return controls, start, warmup_steps*assets.dt


def simulate_one(root: Path, output: Path, duty: float, warmup_cycles: int,
                 evaluation_cycles: int, render: bool) -> tuple[dict, list[dict], list[dict]]:
    import mujoco
    import mujoco.rollout

    assets = base.load_assets(root, N_SEGMENTS, base.FULL_PD_NAME)
    model = assets.model
    controls, eval_start, warm_time = build_controls(assets, duty, warmup_cycles, evaluation_cycles)
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    initial_data = mujoco.MjData(model)
    initial = np.empty(mujoco.mj_stateSize(model, spec), dtype=float)
    mujoco.mj_getState(model, initial_data, initial, spec)
    worker = mujoco.MjData(model)
    print(f"Simulating duty={duty:.1f}", flush=True)
    with mujoco.rollout.Rollout(nthread=1) as engine:
        states, _ = engine.rollout(model, [worker], initial[None, :], controls,
                                  control_spec=mujoco.mjtState.mjSTATE_CTRL.value, chunk_size=1)
    warning_count = sum(int(warning.number) for warning in worker.warning)
    if not np.all(np.isfinite(states)):
        raise RuntimeError(f"Non-finite state at duty={duty}; refusing to plot misleading output")
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, states[0, eval_start-1], spec)
    mujoco.mj_forward(model, data)
    start_com = np.array(data.subtree_com[assets.root_body_id])
    steps = states.shape[1]-eval_start
    sample_steps = np.unique(np.r_[np.arange(0, steps, max(1, round(1/(SAMPLE_HZ*assets.dt)))), steps-1])
    sr, pr, sl, pl, by, bp = assets.indices
    pitch_ids = np.stack((pr, pl), axis=-1)
    pitch_qpos = model.jnt_qposadr[model.actuator_trnid[pitch_ids, 0]]
    body_qpos = model.jnt_qposadr[model.actuator_trnid[by, 0]]
    evaluation_controls = controls[0, eval_start:]
    command_eval = np.stack((evaluation_controls[:, pr], evaluation_controls[:, pl]), axis=-1)
    command_flags = np.isclose(command_eval, math.radians(LEG_DOWN_DEG))
    command_fractions = command_flags.mean(axis=0)
    contacts, sampled_flags, pitch_errors, body_errors, series = [], [], [], [], []
    min_up_z = 1.0
    max_roll = max_pitch = 0.0
    for local in sample_steps:
        index = eval_start + int(local)
        mujoco.mj_setState(model, data, states[0, index], spec)
        mujoco.mj_forward(model, data)
        displacement = np.asarray(data.subtree_com[assets.root_body_id])-start_com
        flags = command_flags[int(local)]
        contact = _actual_contact_flags(model, data, assets.body_map)
        contacts.append(contact)
        sampled_flags.append(flags)
        pitch_errors.append(data.qpos[pitch_qpos]-controls[0, index, pitch_ids])
        body_errors.append(data.qpos[body_qpos]-controls[0, index, by])
        up_z = float(data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2])
        min_up_z = min(min_up_z, up_z)
        pose = np.zeros(7)
        pose[3:7] = data.xquat[assets.stability_body_id]
        roll, pitch = _root_roll_pitch_deg(pose)
        max_roll, max_pitch = max(max_roll, abs(roll)), max(max_pitch, abs(pitch))
        series.append({"duty_factor": duty, "time_s": (int(local)+1)*assets.dt,
                       "forward_displacement_m": float(displacement[1]),
                       "lateral_displacement_m": float(displacement[0]),
                       "commanded_down_count": int(flags.sum()),
                       "ground_contact_count": int(contact.sum()),
                       "roll_deg": roll, "pitch_deg": pitch, "up_z": up_z})
    contacts = np.asarray(contacts)
    sampled_flags = np.asarray(sampled_flags)
    pitch_errors = np.asarray(pitch_errors)
    leg_rows = []
    for segment in range(N_SEGMENTS):
        for side_index, side in enumerate(("R", "L")):
            leg_rows.append({"duty_factor": duty, "segment": segment+1, "side": side,
                             "commanded_down_fraction": float(command_fractions[segment, side_index]),
                             "detected_ground_contact_fraction": float(contacts[:, segment, side_index].mean()),
                             "pitch_tracking_RMS_deg": math.degrees(float(np.sqrt(np.mean(pitch_errors[:, segment, side_index]**2))))})
    video, preview = None, None
    frame_count = readable_frames = 0
    if render:
        import imageio.v2 as imageio
        from PIL import Image, ImageDraw

        demo_dir = output / "demos"
        demo_dir.mkdir(parents=True, exist_ok=True)
        video = demo_dir / f"N10_Nwave2_duty_{duty_tag(duty)}.mp4"
        preview = demo_dir / f"N10_Nwave2_duty_{duty_tag(duty)}_preview.png"
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, WIDTH)
        model.vis.global_.offheight = max(model.vis.global_.offheight, HEIGHT)
        renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
        camera, option = base.step33.base._camera_for(model, assets.root_body_id, N_SEGMENTS, mujoco)
        frame_steps = np.unique(np.minimum(steps-1, np.round(np.arange(0, steps*assets.dt, 1/FPS)/assets.dt).astype(int)))
        frame_steps = np.unique(np.r_[frame_steps, steps-1])
        writer = imageio.get_writer(str(video), fps=FPS, codec="libx264", quality=8,
                                   pixelformat="yuv420p", macro_block_size=2)
        try:
            for frame_index, local in enumerate(frame_steps):
                index = eval_start+int(local)
                mujoco.mj_setState(model, data, states[0, index], spec)
                mujoco.mj_forward(model, data)
                flags = command_flags[int(local)]
                contact = _actual_contact_flags(model, data, assets.body_map)
                _set_leg_colors(model, assets.body_map, flags)
                renderer.update_scene(data, camera=camera, scene_option=option)
                canvas = Image.new("RGB", (WIDTH, HEIGHT+90), "#eef1f2")
                canvas.paste(Image.fromarray(renderer.render().copy()), (0, 50))
                draw = ImageDraw.Draw(canvas)
                draw.text((16, 10), f"N=10 | N_wave=2 | duty={duty:.1f} | leg yaw fixed", font=_font(20, bold=True), fill="#17202a")
                draw.ellipse((WIDTH-246, 17, WIDTH-230, 33), fill="#2aa657")
                draw.text((WIDTH-222, 12), "down", font=_font(15), fill="#17202a")
                draw.ellipse((WIDTH-128, 17, WIDTH-112, 33), fill="#e6452b")
                draw.text((WIDTH-104, 12), "up", font=_font(15), fill="#17202a")
                displacement = np.asarray(data.subtree_com[assets.root_body_id])-start_com
                draw.text((16, HEIGHT+57),
                          f"t={(int(local)+1)*assets.dt:5.2f} s   forward COM={displacement[1]:+.3f} m   down={int(flags.sum())}/20   contact={int(contact.sum())}/20",
                          font=_font(16), fill="#273746")
                writer.append_data(np.asarray(canvas))
                if frame_index == len(frame_steps)//2:
                    canvas.save(preview)
            frame_count = len(frame_steps)
        finally:
            writer.close()
            renderer.close()
        with imageio.get_reader(str(video)) as reader:
            readable_frames = int(reader.count_frames())
    model_unchanged = hashlib.sha256(assets.model_file.read_bytes()).hexdigest() == assets.model_sha256
    gates = {
        "all_states_finite": bool(np.all(np.isfinite(states))),
        "no_mujoco_warnings": warning_count == 0,
        "never_inverted_at_sampled_states": min_up_z > 0.0,
        "command_duty_matches_request": bool(np.max(np.abs(command_fractions-duty)) < 0.002),
        "source_xml_unchanged": model_unchanged,
        "media_readable": not render or readable_frames == frame_count,
    }
    row = {
        "duty_factor": duty, "n_segments": N_SEGMENTS, "N_wave": N_WAVE,
        "spatial_phase_increment_deg": math.degrees(PHI),
        "stance_duration_s": duty*PERIOD, "swing_duration_s": (1-duty)*PERIOD,
        "body_yaw_half_amplitude_deg": base.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_PD": base.FULL_PD_NAME, "warmup_cycles": warmup_cycles,
        "evaluation_cycles": evaluation_cycles, "evaluation_time_s": steps*assets.dt,
        "forward_displacement_m": series[-1]["forward_displacement_m"],
        "lateral_displacement_m": series[-1]["lateral_displacement_m"],
        "mean_commanded_down_fraction": float(command_flags.mean()),
        "min_leg_commanded_down_fraction": float(command_fractions.min()),
        "max_leg_commanded_down_fraction": float(command_fractions.max()),
        "mean_detected_ground_contact_fraction": float(contacts.mean()),
        "contact_given_down": float(contacts[sampled_flags].mean()) if np.any(sampled_flags) else None,
        "contact_given_up": float(contacts[~sampled_flags].mean()) if np.any(~sampled_flags) else None,
        "body_yaw_tracking_RMS_deg": math.degrees(float(np.sqrt(np.mean(np.square(body_errors))))),
        "leg_pitch_tracking_RMS_deg": math.degrees(float(np.sqrt(np.mean(pitch_errors**2)))),
        "max_abs_roll_deg": max_roll, "max_abs_pitch_deg": max_pitch,
        "minimum_up_z": min_up_z, "mujoco_warning_count": warning_count,
        "numerical_and_media_checks_passed": all(gates.values()),
        "status": "PASS" if all(gates.values()) else "REVIEW",
        "mujoco_version": mujoco.__version__, "model_sha256": assets.model_sha256,
        "video": str(video) if video else "", "preview": str(preview) if preview else "",
        "written_frames": frame_count, "readable_frames": readable_frames,
        "gates": json.dumps(gates, sort_keys=True),
    }
    print(f"d={duty:.1f}: D={row['forward_displacement_m']:+.4f} m, contact={contacts.mean():.3f}, tilt=({max_roll:.2f},{max_pitch:.2f}) deg, {row['status']}", flush=True)
    return row, series, leg_rows


def make_figures(output: Path, duties: list[float], rows: list[dict], series: list[dict]) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Patch

    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    phase_fraction = (np.arange(2000)+0.5)/2000
    theta = 2*math.pi*phase_fraction
    fig, axes = plt.subplots(3, 1, figsize=(7.2, 7.5), sharex=True,
                             gridspec_kw={"height_ratios": [1.0, 1.6, 1.6]})
    axes[0].plot(phase_fraction, np.sin(theta), color="#273746", label=r"$\sin\theta_i$")
    axes[0].axhline(0, color="#888888", linewidth=0.7)
    axes[0].set(ylabel="Carrier", ylim=(-1.1, 1.1))
    handles, labels = axes[0].get_legend_handles_labels()
    handles += [Patch(facecolor="#2aa657"), Patch(facecolor="#eef1f2", edgecolor="#aaaaaa")]
    labels += ["Down command", "Up command"]
    axes[0].legend(handles, labels, frameon=False, loc="upper right", fontsize=9)
    cmap = ListedColormap(["#eef1f2", "#2aa657"])
    flags = np.asarray([down_flags_from_phase(theta[:, None], duty)[:, 0, :] for duty in duties])
    for side, ax in enumerate(axes[1:]):
        ax.imshow(flags[:, :, side], origin="lower", aspect="auto", cmap=cmap,
                  interpolation="nearest", extent=(0, 1, -0.5, len(duties)-0.5), vmin=0, vmax=1)
        ax.set(yticks=np.arange(len(duties)), yticklabels=[f"{d:.1f}" for d in duties],
               ylabel=f"{'Right' if side == 0 else 'Left'}: duty factor")
        ax.axvline(0.25 if side == 0 else 0.75, color="#273746", linestyle="--", linewidth=1)
    axes[-1].set(xlabel=r"Local carrier phase, $\theta_i/(2\pi)$", xticks=[0, .25, .5, .75, 1])
    fig.tight_layout()
    fig.savefig(figures / "centered_stance_windows.png", dpi=240)
    plt.close(fig)
    if not rows:
        return
    x = [r["duty_factor"] for r in rows]
    fig, ax = plt.subplots(figsize=(6.7, 4.3))
    ax.plot(x, [r["forward_displacement_m"] for r in rows], "o-", color="#177e89")
    ax.axhline(0, color="#777777", linewidth=0.8)
    ax.set(xlabel="Commanded duty factor", ylabel="Signed forward displacement (m)", xticks=x)
    ax.grid(alpha=0.2)
    fig.tight_layout()
    fig.savefig(figures / "displacement_vs_duty_factor.png", dpi=250)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(6.7, 4.3))
    ax.plot(x, [r["mean_commanded_down_fraction"] for r in rows], "o--", color="#2aa657", label="Commanded down")
    ax.plot(x, [r["mean_detected_ground_contact_fraction"] for r in rows], "s-", color="#d97732", label="Detected ground contact")
    ax.set(xlabel="Commanded duty factor", ylabel="Fraction of leg-time samples", xticks=x, ylim=(0, 1))
    ax.grid(alpha=0.2)
    ax.legend(frameon=False)
    fig.tight_layout()
    fig.savefig(figures / "commanded_vs_detected_contact.png", dpi=250)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(7.3, 4.8))
    for color, duty in zip(plt.cm.viridis(np.linspace(.08, .88, len(rows))), x):
        selected = [r for r in series if r["duty_factor"] == duty]
        ax.plot([0]+[r["time_s"] for r in selected], [0]+[r["forward_displacement_m"] for r in selected],
                color=color, label=f"d={duty:.1f}")
    ax.axhline(0, color="#777777", linewidth=0.8)
    ax.set(xlabel="Evaluation time (s)", ylabel="Signed forward displacement (m)")
    ax.grid(alpha=0.2)
    ax.legend(frameon=False, ncol=3, fontsize=9)
    fig.tight_layout()
    fig.savefig(figures / "displacement_time_series.png", dpi=250)
    plt.close(fig)


def write_readme(output: Path, rows: list[dict], config: dict) -> None:
    lines = [
        "# Step 38: peak-centered duty factor, physical centipede robot", "",
        "N=10; reference wave number 2; spatial increment -80 deg. No IID, no contact erasure, no leg-yaw oscillation. The source XML is unchanged.", "",
        "## Phase and duty contract", "",
        r"For segment index $i=0,\ldots,9$, $\theta_i(t)=\omega t+(i-1/2)\Delta\phi$, with $\Delta\phi=-4\pi/9$ and $\omega=4\,\mathrm{rad/s}$. Body-yaw joint $j$ has phase $\omega t+j\Delta\phi$.", "",
        r"The unchanged stance carriers are $u_{iR}=\sin\theta_i$ and $u_{iL}=-\sin\theta_i$. Their peaks are separated by half a period. A stance/down command is selected when $u_{is}\geq\cos(\pi d)$, using deterministic half-open opening/closing ties in code. Each window has phase width $2\pi d$ and time duration $dT_g$, where $T_g=2\pi/\omega$.", "",
        "The numerical pitch targets remain +5 deg (down) and -40 deg (up). The sine is a phase carrier, not a replacement continuous motor command. No time warping or movement of the stance-window centers is introduced. At d=0.5 the corrected canonical pitch commands are recovered, including zero-crossing ties.", "",
        "A blanket leg-pitch/body-yaw offset of -pi is not the current convention: at the collocated segment center, right-side down follows positive body curvature; left-side down follows negative curvature. Choosing lift rather than down as the positive carrier would reverse that sign. Segment-center and inter-segment-joint phases differ by the required half spatial increment.", "",
        "For d<0.5, each pair has a both-up gap of fraction 1-2d per cycle. For d>0.5, both-down overlap occupies fraction 2d-1. Both sides still have the same duty and retain their original, half-cycle-separated centers.", "",
        "## Controlled simulation", "",
        f"Body-yaw half-amplitude {base.BODY_YAW_HALF_AMPLITUDE_DEG:g} deg, Step-37 moderate PD, fixed leg yaw (R=-5 deg, L=+5 deg), fixed body pitch. Common settling ramp {SETTLE_TIME_S:g} s, cosine bridge {BRIDGE_TIME_S:g} s into each condition's initial targets, {config['warmup_cycles']} warm-up cycles, {config['evaluation_cycles']} evaluated cycles. Each condition is one deterministic run, not an ensemble mean.", "",
        "Command duty is measured at every simulation step. Detected ground contact is sampled at 50 Hz and uses the existing foot/leg-to-world contact-pair definition; it is not an enforced support fraction or a force-threshold criterion. MuJoCo contact geometry, actuation lag, and slip can make it differ from commanded duty. Stability checks are numerical/sample-based gates, not a proof of physical accuracy.", "",
        f"MuJoCo version: {config['mujoco_version']}. Videos and metrics use the very same newly simulated trajectory; they are not cross-runtime reruns of an older trial.", "",
        "## Results and videos", "",
        "| Duty | Down time per cycle (s) | Displacement (m) | Detected contact fraction | Status | Video |",
        "|---:|---:|---:|---:|---|---|",
    ]
    for row in rows:
        url = row["video"].replace("\\", "/")
        lines.append(f"| {row['duty_factor']:.1f} | {row['stance_duration_s']:.3f} | {row['forward_displacement_m']:+.4f} | {row['mean_detected_ground_contact_fraction']:.3f} | {row['status']} | {'[MP4]('+url+')' if url else 'not rendered'} |")
    lines += ["", "## Files", "",
              "- `demos/`: full-evaluation MP4s and previews.",
              "- `figures/centered_stance_windows.png`: carrier and centered R/L command windows.",
              "- `figures/displacement_vs_duty_factor.png`: signed, unnormalized forward COM displacement.",
              "- `figures/commanded_vs_detected_contact.png`: command/contact distinction.",
              "- `figures/displacement_time_series.png`: trajectories.",
              "- `trials.csv`, `time_series.csv`, `per_leg_contact_summary.csv`: compact numerical evidence.",
              "- `controller_validation.json`, `validation.json`, `run_configuration.json`: definition, provenance, and checks.", ""]
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duties", type=float, nargs="+", default=list(DEFAULT_DUTIES))
    parser.add_argument("--warmup-cycles", type=int, default=4)
    parser.add_argument("--evaluation-cycles", type=int, default=12)
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--no-render", action="store_true")
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    duties = sorted(set(args.duties))
    if not duties or any(not 0 <= d <= 1 for d in duties):
        parser.error("provide duty factors between 0 and 1")
    root = base.repo_root()
    output = args.output_dir or root / "exp_data" / "reliability" / OUTPUT_NAME
    output.mkdir(parents=True, exist_ok=True)
    audit = audit_controller(duties)
    base.write_json(output / "controller_validation.json", audit)
    if not audit["passed"]:
        raise RuntimeError(f"Controller audit failed: {audit}")
    print("Centered windows, exact duty, fixed carrier, and d=0.5 canonical recovery: PASS", flush=True)
    if args.audit_only:
        make_figures(output, duties, [], [])
        return
    import mujoco
    import imageio
    import matplotlib
    model_path = base.model_path(root, N_SEGMENTS)
    model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    configuration = {
        "n_segments": N_SEGMENTS, "N_wave_ref10": N_WAVE,
        "spatial_phase_increment_rad": PHI, "spatial_phase_increment_deg": math.degrees(PHI),
        "duty_factors": duties, "sine_carrier_unchanged": True,
        "window_center_rule": "R at local theta=pi/2; L at local theta=3pi/2",
        "window_threshold_rule": "sin(theta_side)>=cos(pi*d); deterministic half-open boundaries",
        "leg_pitch_down_deg": LEG_DOWN_DEG, "leg_pitch_up_deg": LEG_UP_DEG,
        "leg_yaw_fixed": True, "body_yaw_half_amplitude_deg": base.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_PD": base.PD_CANDIDATES[base.FULL_PD_NAME],
        "omega_rad_s": OMEGA_BODY, "gait_period_s": PERIOD,
        "warmup_cycles": args.warmup_cycles, "evaluation_cycles": args.evaluation_cycles,
        "randomization": "none", "contact_erasure": False,
        "model_path": str(model_path), "model_sha256": model_hash,
        "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
        "imageio_version": imageio.__version__, "matplotlib_version": matplotlib.__version__,
        "python_version": sys.version, "runner_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "source_XML_not_modified": True, "contact_sample_Hz": SAMPLE_HZ, "video_fps": FPS,
    }
    base.write_json(output / "run_configuration.json", configuration)
    rows, series, legs = [], [], []
    for duty in duties:
        row, trajectory, leg_rows = simulate_one(root, output, duty, args.warmup_cycles,
                                                args.evaluation_cycles, not args.no_render)
        rows.append(row)
        series.extend(trajectory)
        legs.extend(leg_rows)
        base.write_csv(output / "trials.csv", rows)
        base.write_csv(output / "time_series.csv", series)
        base.write_csv(output / "per_leg_contact_summary.csv", legs)
    make_figures(output, duties, rows, series)
    validation = {
        "completed_all_requested_conditions": len(rows) == len(duties),
        "all_checks_passed": all(row["numerical_and_media_checks_passed"] for row in rows),
        "conditions_completed": len(rows), "conditions_passing": sum(row["numerical_and_media_checks_passed"] for row in rows),
        "controller_audit_passed": audit["passed"],
        "source_xml_unchanged": hashlib.sha256(model_path.read_bytes()).hexdigest() == model_hash,
        "video_count": sum(bool(row["video"]) for row in rows),
        "maximum_warning_count": max(row["mujoco_warning_count"] for row in rows),
        "commands_not_forced_contacts": True, "all_metrics_and_video_from_same_rollout": True,
    }
    base.write_json(output / "validation.json", validation)
    write_readme(output, rows, configuration)
    print(json.dumps(validation, indent=2), flush=True)


if __name__ == "__main__":
    main()
