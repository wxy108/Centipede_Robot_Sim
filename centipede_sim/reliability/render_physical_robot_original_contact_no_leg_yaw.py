"""Step 34: deterministic original contact, matched to Step 33 except pitch law.

Reuse Step 32's corrected cave parity, Step 33's physical assets, amplitude,
neutral leg-yaw targets, initialization stages and video style. No IID draws,
contact switches, model edits or solver retuning are used.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import cave_grouped_wave_controller as cave
import run_physical_robot_pitch_body_yaw_iid_wave_sweep as iid


OUTPUT_NAME = "step34_physical_robot_original_contact_no_leg_yaw_demos"


class OriginalContactNoYaw(cave.CaveGroupedContactPatternController):
    def __init__(self, n_ctrl: int, indices: tuple, n_wave: float):
        super().__init__(iid.N_SEGMENTS, n_ctrl, indices, None,
                         n_wave=n_wave, global_phase_deg=0.0, flipped=False)

    def control(self, body_time_s: float, _decision_time_s: float = 0.0) -> np.ndarray:
        # Keep the exact, corrected Step-32 pitch assignment.
        ctrl = super().control(body_time_s)
        shoulder_r, _leg_r, shoulder_l, _leg_l, link_y, link_p = self.indices
        neutral = math.radians(iid.LEG_OFFSET_DEG)
        ctrl[shoulder_r] = neutral
        ctrl[shoulder_l] = -neutral
        ctrl[link_y] = math.radians(iid.YAW_HALF_AMPLITUDE_DEG) * np.sin(
            iid.OMEGA_BODY * body_time_s + np.arange(iid.N_SEGMENTS - 1) * self.phi_body
        )
        ctrl[link_p] = 0.0
        return ctrl


def audit_controller(waves: list[float]) -> dict:
    indices = iid._build_actuator_indices(iid.N_SEGMENTS)
    sr, pr, sl, pl, by, bp = indices
    nu = 4 * iid.N_SEGMENTS + 2 * (iid.N_SEGMENTS - 1)
    period = 2 * math.pi / iid.OMEGA_BODY
    neutral = math.radians(iid.LEG_OFFSET_DEG)
    results = []
    for wave in waves:
        controller = OriginalContactNoYaw(nu, indices, wave)
        reference = cave.CaveGroupedContactPatternController(
            iid.N_SEGMENTS, nu, indices, None, n_wave=wave,
            global_phase_deg=0, flipped=False,
        )
        times = list(np.linspace(0, period, 721, endpoint=False))
        # Explicitly inspect before/on/after every segment's zero crossing.
        for segment in range(iid.N_SEGMENTS):
            offset = (segment - 0.5) * controller.phi_body
            for half_wave in range(-10, 11):
                t = (half_wave * math.pi - offset) / iid.OMEGA_BODY
                if 0 <= t <= period:
                    times.extend([t - 1e-9, t, t + 1e-9])
        checks = dict(same_cave_same_state=True, left_right_complementary=True,
                      pitch_matches_corrected_step32=True, yaw_targets_constant=True,
                      body_matches_step33=True, exact_periodicity=True)
        for t in times:
            ctrl = controller.control(t)
            flags = controller.stance_flags(t)
            caves = np.array([controller.cave_index(t, i) for i in range(iid.N_SEGMENTS)])
            for group in np.unique(caves):
                group_flags = flags[caves == group]
                checks["same_cave_same_state"] &= bool(np.all(group_flags == group_flags[0]))
            checks["left_right_complementary"] &= bool(np.all(flags[:, 0] ^ flags[:, 1]))
            checks["pitch_matches_corrected_step32"] &= bool(np.array_equal(
                ctrl[np.r_[pr, pl]], reference.control(t)[np.r_[pr, pl]]
            ))
            checks["yaw_targets_constant"] &= bool(
                np.allclose(ctrl[sr], neutral) and np.allclose(ctrl[sl], -neutral)
                and np.allclose(ctrl[bp], 0)
            )
            expected_body = iid._body_only_compute_ctrl(
                t, nu, iid.N_SEGMENTS, indices, phi_spine=controller.phi_body
            )
            checks["body_matches_step33"] &= bool(np.allclose(ctrl[by], expected_body[by]))
            checks["exact_periodicity"] &= bool(np.allclose(ctrl, controller.control(t + period)))
        results.append({"N_wave": wave, "time_samples": len(times), **checks,
                        "passed": all(checks.values())})
    return {"passed": all(row["passed"] for row in results), "rows": results}


def prepare(assets, controller, mujoco):
    model = assets.model
    data = mujoco.MjData(model)
    initial = iid._initial_target(model, assets.indices)
    settle_steps = math.ceil(iid.SETTLE_TIME_S / assets.dt)
    bridge_steps = math.ceil(iid.BRIDGE_TIME_S / assets.dt)
    warmup_steps = math.ceil(4 * 2 * math.pi / iid.OMEGA_BODY / assets.dt)
    for step in range(settle_steps):
        data.ctrl[:] = (step + 1) / settle_steps * initial
        mujoco.mj_step(model, data)
    target = controller.control(0.0)
    for step in range(bridge_steps):
        alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
        data.ctrl[:] = (1 - alpha) * initial + alpha * target
        mujoco.mj_step(model, data)
    for step in range(warmup_steps):
        data.ctrl[:] = controller.control(step * assets.dt)
        mujoco.mj_step(model, data)
    return data, warmup_steps * assets.dt


def render_one(assets, output: Path, wave: float, duration_s: float, fps: int) -> dict:
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw

    model = assets.model
    controller = OriginalContactNoYaw(model.nu, assets.indices, wave)
    data, warmup_time = prepare(assets, controller, mujoco)
    observation = mujoco.MjData(model)
    state_spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    state = np.empty(mujoco.mj_stateSize(model, state_spec))

    def observe():
        # Refresh rendering/metric fields on a separate copy, never changing
        # the integration data's constraint warm start.
        mujoco.mj_getState(model, data, state, state_spec)
        mujoco.mj_setState(model, observation, state, state_spec)
        observation.ctrl[:] = data.ctrl
        mujoco.mj_forward(model, observation)

    observe()
    start_com = observation.subtree_com[assets.root_body_id].copy()
    start_time = float(data.time)
    width, height = 960, 600
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    renderer = mujoco.Renderer(model, height=height, width=width)
    camera, option = iid.base._camera_for(model, assets.root_body_id, iid.N_SEGMENTS, mujoco)
    tag = iid._wave_tag(wave)
    video = output / "demos" / f"Nwave{tag}_original_contact.mp4"
    preview = video.with_name(video.stem + "_preview.png")
    writer = imageio.get_writer(str(video), fps=fps, codec="libx264", quality=8,
                                pixelformat="yuv420p", macro_block_size=2)
    sr, pr, sl, pl, by, _bp = assets.indices
    yaw_act, pitch_act = np.r_[sr, sl], np.r_[pr, pl]
    yaw_q = model.jnt_qposadr[model.actuator_trnid[yaw_act, 0]]
    pitch_q = model.jnt_qposadr[model.actuator_trnid[pitch_act, 0]]
    body_q = model.jnt_qposadr[model.actuator_trnid[by, 0]]
    steps = math.ceil(duration_s / assets.dt)
    frames = 0
    next_frame = 1 / fps
    preview_saved = False
    finite = True
    min_up_z = 1.0
    max_yaw_error = 0.0
    pitch_squared_error = body_squared_error = 0.0
    contact_samples = []
    try:
        for step in range(steps):
            body_t = warmup_time + step * assets.dt
            data.ctrl[:] = controller.control(body_t)
            mujoco.mj_step(model, data)
            finite &= bool(np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel)))
            max_yaw_error = max(max_yaw_error, float(np.max(np.abs(data.qpos[yaw_q] - data.ctrl[yaw_act]))))
            pitch_squared_error += float(np.mean((data.qpos[pitch_q] - data.ctrl[pitch_act]) ** 2))
            body_squared_error += float(np.mean((data.qpos[body_q] - data.ctrl[by]) ** 2))
            elapsed = (step + 1) * assets.dt
            if elapsed + 0.5 * assets.dt < next_frame:
                continue
            observe()
            flags = controller.stance_flags(body_t)
            assert int(flags.sum()) == iid.N_SEGMENTS
            contacts = iid._actual_contact_flags(model, observation, assets.body_map)
            contact_samples.append(float(np.mean(contacts)))
            min_up_z = min(min_up_z, float(observation.xmat[assets.stability_body_id].reshape(3, 3)[2, 2]))
            iid._set_leg_colors(model, assets.body_map, flags)
            renderer.update_scene(observation, camera=camera, scene_option=option)
            canvas = Image.new("RGB", (width, height + 82), "#eef1f2")
            canvas.paste(Image.fromarray(renderer.render().copy()), (0, 44))
            draw = ImageDraw.Draw(canvas)
            draw.text((16, 9), f"N_wave={wave:g} | Original contact | Leg yaw fixed",
                      font=iid._font(20, bold=True), fill="#17202a")
            displacement = float(observation.subtree_com[assets.root_body_id, 1] - start_com[1])
            draw.text((16, height + 51),
                      f"t={elapsed:4.2f} s   signed forward displacement={displacement:+.3f} m   down/contact={int(flags.sum())}/{int(contacts.sum())}",
                      font=iid._font(16), fill="#273746")
            draw.ellipse((width - 210, 14, width - 194, 30), fill="#2aa657")
            draw.text((width - 187, 9), "down", font=iid._font(15), fill="#17202a")
            draw.ellipse((width - 105, 14, width - 89, 30), fill="#e6452b")
            draw.text((width - 82, 9), "up", font=iid._font(15), fill="#17202a")
            writer.append_data(np.asarray(canvas))
            frames += 1
            if not preview_saved and elapsed >= duration_s / 2:
                canvas.save(preview)
                preview_saved = True
            next_frame += 1 / fps
    finally:
        writer.close()
        renderer.close()
    observe()
    warning_count = sum(int(item.number) for item in data.warning)
    elapsed_ok = math.isclose(float(data.time) - start_time, steps * assets.dt, abs_tol=1e-8)
    with imageio.get_reader(str(video)) as reader:
        decoded_frames = reader.count_frames()
        first, last = reader.get_data(0), reader.get_data(decoded_frames - 1)
        video_ok = bool(decoded_frames == frames == round(duration_s * fps)
                        and first.shape == last.shape == (682, 960, 3))
    gates = {"all_states_finite": finite, "no_mujoco_warnings": warning_count == 0,
             "never_inverted_at_video_samples": min_up_z > 0,
             "simulation_time_advanced_as_expected": elapsed_ok, "video_decodes": video_ok}
    row = {"N_wave": wave, "n_segments": iid.N_SEGMENTS, "mode": "original_cave_contact_no_leg_yaw",
           "randomization": "none", "duration_s": duration_s, "fps": fps, "frames": frames,
           "signed_forward_displacement_m": float(observation.subtree_com[assets.root_body_id, 1] - start_com[1]),
           "minimum_body_up_z_at_video_samples": min_up_z, "mujoco_warning_total": warning_count,
           "sampled_actual_contact_fraction": float(np.mean(contact_samples)),
           "maximum_leg_yaw_target_error_deg": math.degrees(max_yaw_error),
           "rms_leg_pitch_target_error_deg": math.degrees(math.sqrt(pitch_squared_error / steps)),
           "rms_body_yaw_target_error_deg": math.degrees(math.sqrt(body_squared_error / steps)),
           "passed": all(gates.values()), "gates": json.dumps(gates),
           "video": str(video), "preview": str(preview)}
    print(f"N_wave={wave:g}: {frames} frames, D={row['signed_forward_displacement_m']:+.4f} m, passed={row['passed']}", flush=True)
    return row


def run(args):
    root = iid._repo_root()
    output = root / "exp_data" / "reliability" / OUTPUT_NAME
    (output / "demos").mkdir(parents=True, exist_ok=True)
    waves = sorted(set(args.n_wave))
    audit = audit_controller(waves)
    (output / "controller_validation.json").write_text(json.dumps(audit, indent=2), encoding="utf-8")
    assert audit["passed"], audit
    print("Cave assignment, pitch reference, constant yaw and periodicity checks passed", flush=True)
    model_path = iid._model_path(root)
    model_hash = hashlib.sha256(model_path.read_bytes()).hexdigest()
    previous = json.loads((root / "exp_data/reliability" / iid.OUTPUT_NAME / "rollout_control_validation.json").read_text(encoding="utf-8"))
    assert model_hash == previous["model_sha256"], "Physical model differs from Step 33"
    assets = iid._load_assets(root)
    config = {"model": str(model_path), "model_sha256": model_hash, "matches_step33_model": True,
              "n_segments": iid.N_SEGMENTS, "N_wave": waves,
              "body_yaw_half_amplitude_deg": iid.YAW_HALF_AMPLITUDE_DEG,
              "body_omega_rad_s": iid.OMEGA_BODY, "leg_yaw_targets_deg": {"R": -5, "L": 5},
              "leg_pitch_targets_deg": {"down": iid.LEG_DOWN_DEG, "up": iid.LEG_UP_DEG},
              "Delta_phi": "-2*pi*N_wave/(N_segments-1)",
              "segment_phase": "omega*t+(i-0.5)*Delta_phi; i=0,...,N-1",
              "pitch_rule": "even floor((segment_phase+1e-12)/pi): R down, L up; odd: exchange",
              "randomization": "none", "contact_erasure": False,
              "settle_s": iid.SETTLE_TIME_S, "bridge_s": iid.BRIDGE_TIME_S,
              "warmup_cycles": 4, "demo_duration_s": args.duration, "fps": args.fps,
              "dt_s": assets.dt, "mujoco_version": __import__("mujoco").__version__}
    (output / "run_configuration.json").write_text(json.dumps(config, indent=2), encoding="utf-8")
    rows = []
    for wave in waves:
        rows.append(render_one(assets, output, wave, args.duration, args.fps))
        iid._write_csv(output / "demo_manifest.csv", rows)
        (output / "demo_manifest.json").write_text(json.dumps(rows, indent=2), encoding="utf-8")
    validation = {"passed": all(row["passed"] for row in rows), "demo_count": len(rows),
                  "expected_demo_count": len(waves), "controller_checks_passed": audit["passed"],
                  "model_unchanged_from_step33": model_hash == hashlib.sha256(model_path.read_bytes()).hexdigest(),
                  "no_randomization": True, "leg_yaw_constant_targets": True}
    (output / "validation.json").write_text(json.dumps(validation, indent=2), encoding="utf-8")
    readme = """# Step 34: original contact pattern, no leg-yaw oscillation

These eight deterministic videos complement Step 33's IID videos. The same
center-rooted, measured-STL, ten-segment physical robot and the same 15-degree
body-yaw half-amplitude are retained. Leg yaw has fixed neutral servo targets
(R=-5 deg, L=+5 deg); pitch is +5 deg down / -40 deg up. No model, gain,
armature, solver, collision or friction parameters are changed.

Only the pitch law changes: restore the corrected Step-32 cave grouping.
The body-joint phase is omega*t+j*Delta_phi; the segment-center phase is
omega*t+(i-1/2)*Delta_phi. Each half-wave/cave receives one identical side-down
choice; the other side is complementary. The pattern is periodic, not random,
and is not artificially turning ground contacts off. Step 32's 0.4-rad body
amplitude and oscillating leg yaw are NOT reintroduced.

Initialization uses the same stages as Step 33: straight all-down settling,
cosine bridge, then four warm-up cycles, now with the deterministic controller.
Each video shows 6 seconds after warm-up at 30 fps. Green = commanded down;
red = commanded up. Commands follow the target body-wave phase; actuator lag
means actual contact and observed instantaneous body curvature need not agree
perfectly with those target states. Measured tracking errors are retained.

`demo_manifest.csv` records each six-second deterministic displacement and
tracking/stability diagnostics. These are not ensemble means and must not be
compared directly with the 18.85-second IID ensemble statistics. `demos/`
contains MP4s and previews; `controller_validation.json` checks grouping and
periodicity, including zero-crossing boundaries; `validation.json` checks
numerical stability and video decoding. Existing Step-32/33 files are untouched.
"""
    (output / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps(validation, indent=2), flush=True)
    assert validation["passed"] and validation["model_unchanged_from_step33"]


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--n-wave", nargs="+", type=float, default=list(iid.DEFAULT_N_WAVE))
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--fps", type=int, default=30)
    run(parser.parse_args())
