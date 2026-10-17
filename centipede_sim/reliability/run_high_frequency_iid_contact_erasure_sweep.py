"""Step 40: high-frequency IID physical-leg contact-erasure sweep.

The body-yaw phase increment is defined on the ten-segment reference robot,

    Delta_phi = -2*pi*N_wave_ref10/9,

and is held fixed while N changes from 5 to 10.  Leg yaw is fixed and every
leg-pitch actuator remains commanded to the down angle.  The stochastic input
acts only on the collision channel: once per T_g/50 window, every physical leg
independently draws E~Bernoulli(q); E=1 sets that leg geom's collision bits to
zero for the complete window.  Consecutive erased windows remain continuously
disabled.  Source XML files are never edited.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import statistics
import sys
import time
from pathlib import Path

import numpy as np


os.environ.setdefault("CENTIPEDE_SKIP_OPTIONAL_MUJOCO_PLUGINS", "1")
HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

import run_locked_phase_segment_iid_sweep as step37  # noqa: E402
from run_independent_random_leg_phases import (  # noqa: E402
    LEG_DOWN_DEG,
    OMEGA_BODY,
    SETTLE_TIME_S,
    _actual_contact_flags,
    _root_roll_pitch_deg,
)


OUTPUT_NAME = "step40_high_frequency_iid_contact_erasure_sweep"
SCHEMA_VERSION = 1
WINDOWS_PER_CYCLE = 50
DEFAULT_TRIALS = 512
DEFAULT_Q = 0.5
DEFAULT_WARMUP_CYCLES = 4
DEFAULT_EVALUATION_CYCLES = 12
MASTER_SEED = 40_2026_0904
RNG_TAG = 40_050
FORWARD_AXIS = 1
LATERAL_AXIS = 0


def write_csv(path: Path, rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("", encoding="utf-8")
        return
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


def read_csv(path: Path) -> list[dict]:
    if not path.is_file() or path.stat().st_size == 0:
        return []
    with path.open("r", encoding="utf-8", newline="") as stream:
        return list(csv.DictReader(stream))


def write_json(path: Path, payload) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def condition_tag(reference_wave: float, n_segments: int) -> str:
    wave = f"{reference_wave:03.1f}".replace(".", "p")
    return f"Nwave10_{wave}_Nseg{n_segments:02d}"


def trial_seed_sequence(n_segments: int, reference_wave: float, trial_index: int) -> np.random.SeedSequence:
    """A non-nested, condition-specific random key for exactly one trial."""
    return np.random.SeedSequence(
        [MASTER_SEED, int(n_segments), step37.wave_token(reference_wave), int(trial_index), RNG_TAG]
    )


def erasure_mask(
    n_segments: int,
    reference_wave: float,
    trial_index: int,
    n_windows: int,
    q: float,
) -> np.ndarray:
    seed_index = int(trial_index) if int(trial_index) >= 0 else 2_147_483_647
    rng = np.random.default_rng(trial_seed_sequence(n_segments, reference_wave, seed_index))
    return rng.random((n_windows, n_segments, 2)) < float(q)


def binary_correlations(mask: np.ndarray) -> tuple[float, float]:
    """Return mean same-window cross-leg and lag-one temporal correlations."""
    values = np.asarray(mask, dtype=float).reshape(mask.shape[0], -1)
    centered = values - np.mean(values, axis=0, keepdims=True)
    scale = np.sqrt(np.sum(centered * centered, axis=0))
    valid = scale > 0
    if int(np.sum(valid)) >= 2:
        normalized = centered[:, valid] / scale[valid]
        corr = normalized.T @ normalized
        upper = corr[np.triu_indices(corr.shape[0], 1)]
        cross_leg = float(np.mean(upper)) if upper.size else 0.0
    else:
        cross_leg = 0.0
    x0 = values[:-1].reshape(-1)
    x1 = values[1:].reshape(-1)
    if np.std(x0) > 0 and np.std(x1) > 0:
        temporal = float(np.corrcoef(x0, x1)[0, 1])
    else:
        temporal = 0.0
    return cross_leg, temporal


def contact_body_slots(model, body_map: dict) -> np.ndarray:
    """Map a MuJoCo body id to the flattened (segment, side) leg slot."""
    slots = np.full(model.nbody, -1, dtype=int)
    for body_id, (segment_index, side_index) in body_map.items():
        slots[int(body_id)] = 2 * int(segment_index) + int(side_index)
    return slots


def actual_contact_flags_fast(model, data, body_slots: np.ndarray, n_segments: int) -> np.ndarray:
    """Vectorized equivalent of the audited world-leg contact detector."""
    flat = np.zeros(2 * int(n_segments), dtype=bool)
    if data.ncon == 0:
        return flat.reshape(n_segments, 2)
    geom_pairs = np.asarray(data.contact.geom[: data.ncon], dtype=int)
    body_pairs = np.asarray(model.geom_bodyid[geom_pairs], dtype=int)
    other_body = np.where(
        body_pairs[:, 0] == 0,
        body_pairs[:, 1],
        np.where(body_pairs[:, 1] == 0, body_pairs[:, 0], -1),
    )
    valid_body = other_body >= 0
    if np.any(valid_body):
        leg_slots = body_slots[other_body[valid_body]]
        leg_slots = leg_slots[leg_slots >= 0]
        flat[leg_slots] = True
    return flat.reshape(n_segments, 2)


def leg_geom_ids(model, n_segments: int, mujoco) -> np.ndarray:
    ids = np.empty((n_segments, 2), dtype=int)
    for segment in range(1, n_segments + 1):
        for side_index, side in enumerate(("R", "L")):
            name = f"leg_geom_{segment}_{side}"
            geom_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name))
            if geom_id < 0:
                raise KeyError(f"missing physical-leg geom: {name}")
            ids[segment - 1, side_index] = geom_id
    if len(set(ids.reshape(-1).tolist())) != 2 * n_segments:
        raise RuntimeError("leg geom mapping is not one-to-one")
    return ids


def leg_ground_pair_ids(model, geom_ids: np.ndarray, mujoco) -> np.ndarray:
    """Map each leg geom to its unique explicit floor-contact pair."""
    floor_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "floor"))
    if floor_id < 0:
        raise KeyError("floor geom is missing")
    pair_ids = np.empty_like(geom_ids)
    for index, leg_id in np.ndenumerate(geom_ids):
        matches = np.flatnonzero(
            ((model.pair_geom1 == floor_id) & (model.pair_geom2 == int(leg_id)))
            | ((model.pair_geom2 == floor_id) & (model.pair_geom1 == int(leg_id)))
        )
        if matches.size != 1:
            raise RuntimeError(
                f"leg geom {int(leg_id)} has {matches.size} explicit floor pairs; expected one"
            )
        pair_ids[index] = int(matches[0])
    if len(set(pair_ids.reshape(-1).tolist())) != pair_ids.size:
        raise RuntimeError("leg explicit-pair mapping is not one-to-one")
    return pair_ids


def all_down_control(assets, body_time_s: float, reference_wave: float) -> np.ndarray:
    ctrl = step37.initial_target(assets.model, assets.indices)
    _shoulder_r, leg_r, _shoulder_l, leg_l, link_y, link_p = assets.indices
    ctrl[leg_r] = math.radians(LEG_DOWN_DEG)
    ctrl[leg_l] = math.radians(LEG_DOWN_DEG)
    ctrl[link_y] = math.radians(step37.BODY_YAW_HALF_AMPLITUDE_DEG) * np.sin(
        OMEGA_BODY * float(body_time_s)
        + np.arange(assets.n_segments - 1, dtype=float)
        * step37.locked_phase_increment(reference_wave)
    )
    ctrl[link_p] = 0.0
    return ctrl


def warmup_state(assets, reference_wave: float, warmup_cycles: int):
    """Return the clean, all-down state immediately before evaluation."""
    import mujoco
    import mujoco.rollout

    controls, evaluation_start, warmup_time = step37.build_controls(
        assets,
        seeds=[0],
        reference_wave=reference_wave,
        warmup_cycles=warmup_cycles,
        evaluation_cycles=1,
    )
    _shoulder_r, leg_r, _shoulder_l, leg_l, _link_y, _link_p = assets.indices
    settle_steps = int(math.ceil(SETTLE_TIME_S / assets.dt))
    pitch_ids = np.r_[leg_r, leg_l]
    controls[0, settle_steps:evaluation_start, pitch_ids] = math.radians(LEG_DOWN_DEG)
    controls = np.ascontiguousarray(controls[:, :evaluation_start, :])
    spec = mujoco.mjtState.mjSTATE_FULLPHYSICS.value
    initial_data = mujoco.MjData(assets.model)
    initial = np.empty(mujoco.mj_stateSize(assets.model, spec), dtype=float)
    mujoco.mj_getState(assets.model, initial_data, initial, spec)
    worker = [mujoco.MjData(assets.model)]
    with mujoco.rollout.Rollout(nthread=1) as engine:
        states, _ = engine.rollout(
            assets.model,
            worker,
            initial[None, :],
            controls,
            control_spec=mujoco.mjtState.mjSTATE_CTRL.value,
            chunk_size=1,
        )
    warnings = sum(int(item.number) for item in worker[0].warning)
    if warnings:
        raise RuntimeError(f"warm-up generated {warnings} MuJoCo warnings")
    return np.asarray(states[0, -1], dtype=float).copy(), float(warmup_time), spec


def window_edges(dt: float, evaluation_cycles: int) -> tuple[np.ndarray, float, float]:
    period = 2.0 * math.pi / OMEGA_BODY
    evaluation_steps = int(math.ceil(evaluation_cycles * period / dt))
    n_windows = int(evaluation_cycles * WINDOWS_PER_CYCLE)
    edges = np.rint(np.linspace(0, evaluation_steps, n_windows + 1)).astype(int)
    if edges[0] != 0 or edges[-1] != evaluation_steps or np.any(np.diff(edges) <= 0):
        raise RuntimeError("invalid channel-window discretization")
    durations = np.diff(edges) * dt
    return edges, float(np.min(durations)), float(np.max(durations))


def apply_erasure_state(
    model,
    geom_ids: np.ndarray,
    pair_ids: np.ndarray,
    mask: np.ndarray,
    original: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> None:
    flat_ids = geom_ids.reshape(-1)
    flat_mask = np.asarray(mask, dtype=bool).reshape(-1)
    flat_pairs = pair_ids.reshape(-1)
    original_contype, original_conaffinity, original_pair_margin = original
    model.geom_contype[flat_ids] = np.where(flat_mask, 0, original_contype)
    model.geom_conaffinity[flat_ids] = np.where(flat_mask, 0, original_conaffinity)
    # These XMLs contain explicit floor-leg pairs, which bypass the ordinary
    # geom bitmask.  A negative one-metre activation margin makes an erased
    # pair geometrically inactive while preserving all pair identities and
    # solver parameters.  The q=1 semantic pilot verifies that no erased leg
    # appears in data.contact.
    model.pair_margin[flat_pairs] = np.where(flat_mask, -1.0, original_pair_margin)


def restore_leg_collisions(
    model,
    geom_ids: np.ndarray,
    pair_ids: np.ndarray,
    original: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> None:
    flat_ids = geom_ids.reshape(-1)
    flat_pairs = pair_ids.reshape(-1)
    model.geom_contype[flat_ids] = original[0]
    model.geom_conaffinity[flat_ids] = original[1]
    model.pair_margin[flat_pairs] = original[2]


def run_trial(
    assets,
    start_state: np.ndarray,
    state_spec: int,
    warmup_time_s: float,
    reference_wave: float,
    trial_index: int,
    q: float,
    evaluation_cycles: int,
    geom_ids: np.ndarray,
    pair_ids: np.ndarray,
    original_collision: tuple[np.ndarray, np.ndarray, np.ndarray],
) -> dict:
    import mujoco

    edges, min_window_s, max_window_s = window_edges(assets.dt, evaluation_cycles)
    n_windows = len(edges) - 1
    mask = erasure_mask(assets.n_segments, reference_wave, trial_index, n_windows, q)
    cross_corr, lag_corr = binary_correlations(mask)
    mask_digest = hashlib.sha256(np.packbits(mask.reshape(-1)).tobytes()).hexdigest()
    data = mujoco.MjData(assets.model)
    mujoco.mj_setState(assets.model, data, start_state, state_spec)
    restore_leg_collisions(assets.model, geom_ids, pair_ids, original_collision)
    mujoco.mj_forward(assets.model, data)
    start_com = np.asarray(data.subtree_com[assets.root_body_id], dtype=float).copy()
    start_z = float(data.xpos[assets.stability_body_id, 2])
    actual_contact_sum = 0
    available_contact_sum = 0
    erased_contact_violations = 0
    min_up_z = float(data.xmat[assets.stability_body_id].reshape(3, 3)[2, 2])
    min_root_z = start_z
    max_roll = 0.0
    max_pitch = 0.0
    body_error_sq = 0.0
    body_error_count = 0
    body_slots = contact_body_slots(assets.model, assets.body_map)
    link_y = assets.indices[4]
    body_qpos = assets.model.jnt_qposadr[assets.model.actuator_trnid[link_y, 0]]
    all_pitch_down = True
    previous_mask = None
    try:
        for window_index in range(n_windows):
            current_mask = mask[window_index]
            if previous_mask is None or not np.array_equal(current_mask, previous_mask):
                apply_erasure_state(
                    assets.model, geom_ids, pair_ids, current_mask, original_collision
                )
            previous_mask = current_mask
            midpoint_step = 0.5 * (edges[window_index] + edges[window_index + 1])
            body_time = warmup_time_s + midpoint_step * assets.dt
            control = all_down_control(assets, body_time, reference_wave)
            data.ctrl[:] = control
            leg_r, leg_l = assets.indices[1], assets.indices[3]
            all_pitch_down = all_pitch_down and bool(
                np.allclose(control[np.r_[leg_r, leg_l]], math.radians(LEG_DOWN_DEG))
            )
            mujoco.mj_step(
                assets.model,
                data,
                nstep=int(edges[window_index + 1] - edges[window_index]),
            )
            contacts = actual_contact_flags_fast(
                assets.model, data, body_slots, assets.n_segments
            )
            actual_contact_sum += int(np.sum(contacts))
            available_contact_sum += int(np.sum(contacts & ~current_mask))
            erased_contact_violations += int(np.sum(contacts & current_mask))
            root_matrix = data.xmat[assets.stability_body_id].reshape(3, 3)
            min_up_z = min(min_up_z, float(root_matrix[2, 2]))
            min_root_z = min(min_root_z, float(data.xpos[assets.stability_body_id, 2]))
            pose = np.zeros(7, dtype=float)
            pose[3:7] = data.xquat[assets.stability_body_id]
            roll, pitch = _root_roll_pitch_deg(pose)
            max_roll = max(max_roll, abs(float(roll)))
            max_pitch = max(max_pitch, abs(float(pitch)))
            target = control[link_y]
            error = data.qpos[body_qpos] - target
            body_error_sq += float(np.sum(error * error))
            body_error_count += int(error.size)
    finally:
        restore_leg_collisions(assets.model, geom_ids, pair_ids, original_collision)
    final_com = np.asarray(data.subtree_com[assets.root_body_id], dtype=float).copy()
    displacement = final_com - start_com
    warning_total = int(sum(int(item.number) for item in data.warning))
    state = np.empty(mujoco.mj_stateSize(assets.model, state_spec), dtype=float)
    mujoco.mj_getState(assets.model, data, state, state_spec)
    gates = {
        "all_pitch_commands_down": all_pitch_down,
        "all_final_state_finite": bool(np.all(np.isfinite(state))),
        "no_mujoco_warnings": warning_total == 0,
        "no_erased_leg_contact_at_window_boundaries": erased_contact_violations == 0,
        "collision_bits_restored": bool(
            np.array_equal(assets.model.geom_contype[geom_ids.reshape(-1)], original_collision[0])
            and np.array_equal(assets.model.geom_conaffinity[geom_ids.reshape(-1)], original_collision[1])
            and np.array_equal(assets.model.pair_margin[pair_ids.reshape(-1)], original_collision[2])
        ),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "status": "PASS" if all(gates.values()) else "FAIL",
        "n_segments": assets.n_segments,
        "n_legs": 2 * assets.n_segments,
        "N_wave_ref10": float(reference_wave),
        "locked_spatial_phase_offset_rad": step37.locked_phase_increment(reference_wave),
        "locked_spatial_phase_offset_deg": math.degrees(step37.locked_phase_increment(reference_wave)),
        "effective_wave_count": step37.effective_wave_count(reference_wave, assets.n_segments),
        "trial_index": int(trial_index),
        "seed_entropy": json.dumps(trial_seed_sequence(
            assets.n_segments,
            reference_wave,
            int(trial_index) if int(trial_index) >= 0 else 2_147_483_647,
        ).entropy),
        "mask_sha256": mask_digest,
        "q_commanded": float(q),
        "q_empirical": float(np.mean(mask)),
        "same_window_mean_cross_leg_correlation": cross_corr,
        "lag1_temporal_correlation": lag_corr,
        "windows_per_cycle": WINDOWS_PER_CYCLE,
        "channel_window_target_s": (2.0 * math.pi / OMEGA_BODY) / WINDOWS_PER_CYCLE,
        "channel_window_min_s": min_window_s,
        "channel_window_max_s": max_window_s,
        "n_channel_windows": n_windows,
        "channel_decisions": int(mask.size),
        "evaluation_cycles": int(evaluation_cycles),
        "evaluation_time_s": float(edges[-1] * assets.dt),
        "body_yaw_half_amplitude_deg": step37.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_pd_name": assets.pd_manifest["name"],
        "leg_yaw_fixed": True,
        "leg_pitch_command_deg": LEG_DOWN_DEG,
        "forward_displacement_m": float(displacement[FORWARD_AXIS]),
        "lateral_displacement_m": float(displacement[LATERAL_AXIS]),
        "vertical_displacement_m": float(displacement[2]),
        "mean_actual_contact_fraction_at_window_boundaries": float(
            actual_contact_sum / (n_windows * 2 * assets.n_segments)
        ),
        "mean_available_leg_contact_fraction_at_window_boundaries": float(
            available_contact_sum / (n_windows * 2 * assets.n_segments)
        ),
        "erased_contact_violation_count": erased_contact_violations,
        "minimum_root_height_m": min_root_z,
        "minimum_up_z": min_up_z,
        "physical_inversion_observed_at_window_boundaries": min_up_z <= 0.0,
        "max_abs_roll_deg": max_roll,
        "max_abs_pitch_deg": max_pitch,
        "rms_body_yaw_tracking_error_deg": math.degrees(
            math.sqrt(body_error_sq / max(body_error_count, 1))
        ),
        "mujoco_warning_total": warning_total,
        "model_sha256": assets.model_sha256,
        "gates": json.dumps(gates, sort_keys=True),
    }


def run_condition(
    root: Path,
    output: Path,
    n_segments: int,
    reference_wave: float,
    trials: int,
    q: float,
    warmup_cycles: int,
    evaluation_cycles: int,
    force: bool,
) -> list[dict]:
    import mujoco

    tag = condition_tag(reference_wave, n_segments)
    part = output / "parts" / f"N{n_segments:02d}"
    trial_file = part / f"trials_{tag}.csv"
    rows = [] if force else read_csv(trial_file)
    # Early Step-40 pilots treated physical inversion as a numerical gate.  It
    # is a valid locomotion failure outcome, so normalize any resumable rows to
    # the corrected separation of validation and performance diagnostics.
    for row in rows:
        gates = json.loads(row["gates"])
        gates.pop("never_inverted_at_window_boundaries", None)
        row["gates"] = json.dumps(gates, sort_keys=True)
        row["status"] = "PASS" if all(bool(value) for value in gates.values()) else "FAIL"
        row.setdefault(
            "physical_inversion_observed_at_window_boundaries",
            str(float(row["minimum_up_z"]) <= 0.0),
        )
    if rows:
        write_csv(trial_file, rows)
    completed = {int(row["trial_index"]) for row in rows}
    assets = step37.load_assets(root, n_segments, step37.FULL_PD_NAME)
    geom_ids = leg_geom_ids(assets.model, n_segments, mujoco)
    pair_ids = leg_ground_pair_ids(assets.model, geom_ids, mujoco)
    flat_ids = geom_ids.reshape(-1)
    flat_pairs = pair_ids.reshape(-1)
    original_collision = (
        np.asarray(assets.model.geom_contype[flat_ids], dtype=int).copy(),
        np.asarray(assets.model.geom_conaffinity[flat_ids], dtype=int).copy(),
        np.asarray(assets.model.pair_margin[flat_pairs], dtype=float).copy(),
    )
    if np.any(original_collision[0] == 0) or np.any(original_collision[1] == 0):
        raise RuntimeError(f"{tag}: a source leg collision channel is already disabled")
    start_state, warmup_time, state_spec = warmup_state(assets, reference_wave, warmup_cycles)
    model_hash_before = hashlib.sha256(assets.model_file.read_bytes()).hexdigest()
    started = time.perf_counter()
    for trial_index in range(trials):
        if trial_index in completed:
            continue
        row = run_trial(
            assets,
            start_state,
            state_spec,
            warmup_time,
            reference_wave,
            trial_index,
            q,
            evaluation_cycles,
            geom_ids,
            pair_ids,
            original_collision,
        )
        rows.append(row)
        if len(rows) % 8 == 0 or len(rows) == trials:
            rows.sort(key=lambda item: int(item["trial_index"]))
            write_csv(trial_file, rows)
        if (trial_index + 1) % 32 == 0 or trial_index + 1 == trials:
            elapsed = time.perf_counter() - started
            print(
                f"{tag}: {len(rows)}/{trials} trials; "
                f"last D={row['forward_displacement_m']:+.4f} m; "
                f"elapsed={elapsed:.1f}s",
                flush=True,
            )
    model_hash_after = hashlib.sha256(assets.model_file.read_bytes()).hexdigest()
    if model_hash_before != model_hash_after or model_hash_after != assets.model_sha256:
        raise RuntimeError(f"{tag}: source XML changed")
    if len(rows) != trials or len({int(row["trial_index"]) for row in rows}) != trials:
        raise RuntimeError(f"{tag}: incomplete or duplicate trial rows")
    if len({row["mask_sha256"] for row in rows}) != trials:
        raise RuntimeError(f"{tag}: duplicate stochastic masks")
    condition_validation = {
        "condition": tag,
        "passed": all(row["status"] == "PASS" for row in rows),
        "trial_count": len(rows),
        "unique_trial_indices": len({int(row["trial_index"]) for row in rows}),
        "unique_masks": len({row["mask_sha256"] for row in rows}),
        "source_xml_unchanged": model_hash_before == model_hash_after,
        "model_sha256": model_hash_after,
    }
    write_json(part / f"validation_{tag}.json", condition_validation)
    if not condition_validation["passed"]:
        raise RuntimeError(f"{tag}: trial gate failure")
    return rows


def clean_reference(
    root: Path,
    n_segments: int,
    reference_wave: float,
    warmup_cycles: int,
    evaluation_cycles: int,
) -> dict:
    import mujoco

    assets = step37.load_assets(root, n_segments, step37.FULL_PD_NAME)
    geom_ids = leg_geom_ids(assets.model, n_segments, mujoco)
    pair_ids = leg_ground_pair_ids(assets.model, geom_ids, mujoco)
    flat_ids = geom_ids.reshape(-1)
    flat_pairs = pair_ids.reshape(-1)
    original = (
        np.asarray(assets.model.geom_contype[flat_ids], dtype=int).copy(),
        np.asarray(assets.model.geom_conaffinity[flat_ids], dtype=int).copy(),
        np.asarray(assets.model.pair_margin[flat_pairs], dtype=float).copy(),
    )
    start_state, warmup_time, state_spec = warmup_state(assets, reference_wave, warmup_cycles)
    row = run_trial(
        assets,
        start_state,
        state_spec,
        warmup_time,
        reference_wave,
        trial_index=-1,
        q=0.0,
        evaluation_cycles=evaluation_cycles,
        geom_ids=geom_ids,
        pair_ids=pair_ids,
        original_collision=original,
    )
    row["condition"] = "matched_clean_all_pitch_down_reference"
    return row


def percentile(values: list[float], q: float) -> float:
    return float(np.quantile(np.asarray(values, dtype=float), q))


def summarize(rows: list[dict], clean_rows: list[dict]) -> list[dict]:
    clean_map = {
        (int(row["n_segments"]), float(row["N_wave_ref10"])): float(row["forward_displacement_m"])
        for row in clean_rows
    }
    groups: dict[tuple[int, float], list[dict]] = {}
    for row in rows:
        groups.setdefault((int(row["n_segments"]), float(row["N_wave_ref10"])), []).append(row)
    summaries = []
    for (n_segments, wave), group in sorted(groups.items(), key=lambda item: (item[0][1], item[0][0])):
        displacement = [float(row["forward_displacement_m"]) for row in group]
        q_values = [float(row["q_empirical"]) for row in group]
        contact = [float(row["mean_actual_contact_fraction_at_window_boundaries"]) for row in group]
        cross = [float(row["same_window_mean_cross_leg_correlation"]) for row in group]
        lag = [float(row["lag1_temporal_correlation"]) for row in group]
        mean_d = statistics.fmean(displacement)
        clean = clean_map[(n_segments, wave)]
        summaries.append({
            "n_segments": n_segments,
            "n_legs": 2 * n_segments,
            "N_wave_ref10": wave,
            "locked_spatial_phase_offset_deg": math.degrees(step37.locked_phase_increment(wave)),
            "effective_wave_count": step37.effective_wave_count(wave, n_segments),
            "trial_count": len(group),
            "pass_count": sum(row["status"] == "PASS" for row in group),
            "mean_forward_displacement_m": mean_d,
            "median_forward_displacement_m": statistics.median(displacement),
            "std_forward_displacement_m": statistics.stdev(displacement),
            "q25_forward_displacement_m": percentile(displacement, 0.25),
            "q75_forward_displacement_m": percentile(displacement, 0.75),
            "iqr_forward_displacement_m": percentile(displacement, 0.75) - percentile(displacement, 0.25),
            "min_forward_displacement_m": min(displacement),
            "max_forward_displacement_m": max(displacement),
            "clean_reference_forward_displacement_m": clean,
            "mean_displacement_change_from_clean_m": mean_d - clean,
            "mean_empirical_q": statistics.fmean(q_values),
            "std_empirical_q_across_trials": statistics.stdev(q_values),
            "mean_actual_contact_fraction_at_window_boundaries": statistics.fmean(contact),
            "mean_same_window_cross_leg_correlation": statistics.fmean(cross),
            "mean_lag1_temporal_correlation": statistics.fmean(lag),
        })
    return summaries


def plot_results(output: Path, rows: list[dict], summaries: list[dict], segments: list[int], waves: list[float]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures = output / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    colors = plt.cm.viridis(np.linspace(0.08, 0.92, len(waves)))

    def series(wave: float, field: str) -> list[float]:
        return [
            float(next(row for row in summaries if int(row["n_segments"]) == n and math.isclose(float(row["N_wave_ref10"]), wave))[field])
            for n in segments
        ]

    for filename, field, ylabel in (
        ("mean_forward_displacement_vs_segment_number.png", "mean_forward_displacement_m", "Mean signed forward displacement (m)"),
        ("std_forward_displacement_vs_segment_number.png", "std_forward_displacement_m", "SD of signed forward displacement (m)"),
        ("mean_change_from_clean_vs_segment_number.png", "mean_displacement_change_from_clean_m", "Mean displacement change from clean (m)"),
        ("actual_contact_fraction_vs_segment_number.png", "mean_actual_contact_fraction_at_window_boundaries", "Actual-contact fraction"),
    ):
        fig, ax = plt.subplots(figsize=(7.2, 4.7))
        for color, wave in zip(colors, waves):
            ax.plot(segments, series(wave, field), "o-", color=color, label=rf"$N_{{wave}}^{{(10)}}={wave:g}$")
        if "displacement" in field:
            ax.axhline(0.0, color="#777777", linewidth=0.8)
        ax.set(xlabel="Segment number, N", ylabel=ylabel, xticks=segments)
        ax.grid(alpha=0.2)
        ax.legend(frameon=False, fontsize=8, ncol=2)
        fig.tight_layout()
        fig.savefig(figures / filename, dpi=260)
        plt.close(fig)

    phase_dir = figures / "by_phase"
    phase_dir.mkdir(parents=True, exist_ok=True)
    for color, wave in zip(colors, waves):
        mean = np.asarray(series(wave, "mean_forward_displacement_m"))
        q25 = np.asarray(series(wave, "q25_forward_displacement_m"))
        q75 = np.asarray(series(wave, "q75_forward_displacement_m"))
        clean = np.asarray(series(wave, "clean_reference_forward_displacement_m"))
        fig, ax = plt.subplots(figsize=(6.2, 4.2))
        ax.fill_between(segments, q25, q75, color=color, alpha=0.18, linewidth=0)
        ax.plot(segments, mean, "o-", color=color, label="IID mean (band: IQR)")
        ax.plot(segments, clean, "s--", color="#4d4d4d", label="Matched clean reference")
        ax.axhline(0.0, color="#888888", linewidth=0.8)
        ax.set(xlabel="Segment number, N", ylabel="Signed forward displacement (m)", xticks=segments)
        ax.grid(alpha=0.2)
        ax.legend(frameon=False, fontsize=8)
        fig.tight_layout()
        tag = f"{wave:.1f}".replace(".", "p")
        fig.savefig(phase_dir / f"performance_Nwave10_{tag}.png", dpi=260)
        plt.close(fig)

    matrix = np.asarray([[series(wave, "mean_forward_displacement_m")[index] for index in range(len(segments))] for wave in waves])
    fig, ax = plt.subplots(figsize=(7.0, 4.6))
    image = ax.imshow(matrix, aspect="auto", cmap="RdBu_r", origin="lower")
    ax.set(xticks=np.arange(len(segments)), xticklabels=segments,
           yticks=np.arange(len(waves)), yticklabels=[f"{wave:g}" for wave in waves],
           xlabel="Segment number, N", ylabel=r"Reference phase label, $N_{wave}^{(10)}$")
    bar = fig.colorbar(image, ax=ax)
    bar.set_label("Mean signed forward displacement (m)")
    fig.tight_layout()
    fig.savefig(figures / "mean_forward_displacement_heatmap.png", dpi=260)
    plt.close(fig)

    for wave in waves:
        wave_values = {
            n_segments: np.asarray([
                float(row["forward_displacement_m"])
                for row in rows
                if int(row["n_segments"]) == n_segments
                and math.isclose(float(row["N_wave_ref10"]), wave)
            ], dtype=float)
            for n_segments in segments
        }
        pooled = np.concatenate(list(wave_values.values()))
        x_min = float(np.min(pooled))
        x_max = float(np.max(pooled))
        x_padding = max(0.02 * (x_max - x_min), 1e-6)
        common_edges = np.linspace(x_min - x_padding, x_max + x_padding, 25)
        common_y_max = max(
            int(np.histogram(values, bins=common_edges)[0].max())
            for values in wave_values.values()
        )

        fig, axes = plt.subplots(2, 3, figsize=(9.0, 5.6), sharex=True, sharey=True)
        for axis, n_segments in zip(axes.reshape(-1), segments):
            values = wave_values[n_segments]
            axis.hist(values, bins=common_edges, color="#237f8d", alpha=0.86, edgecolor="white", linewidth=0.35)
            axis.axvline(statistics.fmean(values), color="#d95f02", linewidth=1.2)
            axis.set_xlim(common_edges[0], common_edges[-1])
            axis.set_ylim(0.0, 1.08 * common_y_max)
            axis.set_title(f"N={n_segments}", fontsize=10)
            axis.grid(axis="y", alpha=0.15)
        fig.supxlabel("Signed forward displacement (m)")
        fig.supylabel("Trial count")
        fig.suptitle(rf"$N_{{wave}}^{{(10)}}={wave:g}$, IID contact erasure ($q={DEFAULT_Q:g}$)", fontsize=12)
        fig.tight_layout()
        tag = f"{wave:.1f}".replace(".", "p")
        fig.savefig(figures / f"histograms_Nwave10_{tag}.png", dpi=260)
        plt.close(fig)

    q_values = [float(row["q_empirical"]) for row in rows]
    cross = [float(row["same_window_mean_cross_leg_correlation"]) for row in rows]
    lag = [float(row["lag1_temporal_correlation"]) for row in rows]
    fig, axes = plt.subplots(1, 3, figsize=(10.0, 3.5))
    for axis, values, target, label in (
        (axes[0], q_values, DEFAULT_Q, "Empirical erasure probability"),
        (axes[1], cross, 0.0, "Cross-leg correlation"),
        (axes[2], lag, 0.0, "Lag-1 correlation"),
    ):
        axis.hist(values, bins=30, color="#5b8c5a", alpha=0.85)
        axis.axvline(target, color="#b23a48", linewidth=1.2)
        axis.set_xlabel(label)
        axis.set_ylabel("Trial count")
        axis.grid(axis="y", alpha=0.15)
    fig.tight_layout()
    fig.savefig(figures / "iid_channel_diagnostics.png", dpi=260)
    plt.close(fig)


def finalize(root: Path, output: Path, segments: list[int], waves: list[float], trials: int, q: float) -> None:
    rows: list[dict] = []
    clean_rows: list[dict] = []
    for n_segments in segments:
        part = output / "parts" / f"N{n_segments:02d}"
        for wave in waves:
            path = part / f"trials_{condition_tag(wave, n_segments)}.csv"
            condition_rows = read_csv(path)
            if len(condition_rows) != trials:
                raise RuntimeError(f"{path}: expected {trials} rows, found {len(condition_rows)}")
            rows.extend(condition_rows)
        for wave in waves:
            clean_path = part / f"clean_reference_{condition_tag(wave, n_segments)}.csv"
            part_clean = read_csv(clean_path)
            if len(part_clean) != 1:
                raise RuntimeError(f"{clean_path}: expected one row, found {len(part_clean)}")
            clean_rows.extend(part_clean)
    expected = len(segments) * len(waves) * trials
    identities = {
        (int(row["n_segments"]), float(row["N_wave_ref10"]), int(row["trial_index"]))
        for row in rows
    }
    masks = {row["mask_sha256"] for row in rows}
    if len(rows) != expected or len(identities) != expected or len(masks) != expected:
        raise RuntimeError("global trial identity or stochastic-mask uniqueness failed")
    rows.sort(key=lambda row: (float(row["N_wave_ref10"]), int(row["n_segments"]), int(row["trial_index"])))
    clean_rows.sort(key=lambda row: (float(row["N_wave_ref10"]), int(row["n_segments"])))
    write_csv(output / "iid_contact_erasure_trials.csv", rows)
    write_csv(output / "matched_clean_references.csv", clean_rows)
    summaries = summarize(rows, clean_rows)
    write_csv(output / "iid_contact_erasure_summary.csv", summaries)
    plot_results(output, rows, summaries, segments, waves)
    validation = {
        "passed": bool(
            len(rows) == expected
            and len(identities) == expected
            and len(masks) == expected
            and all(row["status"] == "PASS" for row in rows)
            and all(row["status"] == "PASS" for row in clean_rows)
        ),
        "expected_trials": expected,
        "trial_count": len(rows),
        "unique_condition_trial_keys": len(identities),
        "unique_erasure_masks": len(masks),
        "condition_count": len(segments) * len(waves),
        "trials_per_condition": trials,
        "all_trial_gates_pass": all(row["status"] == "PASS" for row in rows),
        "all_clean_reference_gates_pass": all(row["status"] == "PASS" for row in clean_rows),
        "maximum_erased_contact_violation_count": max(int(row["erased_contact_violation_count"]) for row in rows),
        "mean_empirical_q": statistics.fmean(float(row["q_empirical"]) for row in rows),
        "mean_cross_leg_correlation": statistics.fmean(float(row["same_window_mean_cross_leg_correlation"]) for row in rows),
        "mean_lag1_temporal_correlation": statistics.fmean(float(row["lag1_temporal_correlation"]) for row in rows),
    }
    write_json(output / "validation.json", validation)
    write_json(output / "run_configuration.json", {
        "model": "center-rooted physical measured-STL robot",
        "segments": segments,
        "reference_wave_labels": waves,
        "locked_phase_formula": "Delta_phi=-2*pi*N_wave_ref10/9",
        "trials_per_condition": trials,
        "q": q,
        "master_seed": MASTER_SEED,
        "rng_key": "[master_seed, N, wave_token, trial_index, rng_tag]",
        "windows_per_cycle": WINDOWS_PER_CYCLE,
        "warmup_cycles": DEFAULT_WARMUP_CYCLES,
        "evaluation_cycles": DEFAULT_EVALUATION_CYCLES,
        "leg_pitch": f"always commanded down at {LEG_DOWN_DEG:g} deg",
        "leg_yaw": "fixed",
        "erasure_object": "each physical leg collision geom independently",
        "body_yaw_half_amplitude_deg": step37.BODY_YAW_HALF_AMPLITUDE_DEG,
        "body_yaw_pd": step37.FULL_PD_NAME,
        "control_discretization": "body-yaw target zero-order held within each T_g/50 channel window",
        "primary_performance": "direct signed COM forward displacement in meters",
    })
    readme = f"""# Step 40: high-frequency IID contact erasure

This experiment uses the center-rooted physical measured-STL robot.  It spans
N={segments[0]}..{segments[-1]} and the seven phase increments labelled
`N_wave_ref10={','.join(f'{wave:g}' for wave in waves)}` on the ten-segment
reference.  Each of the {len(segments) * len(waves)} conditions has {trials}
independent trials ({expected} total).

Every leg remains commanded pitch-down ({LEG_DOWN_DEG:g} deg), and leg yaw is
fixed.  At each of 50 channel windows per gait cycle, each physical leg draws
an independent Bernoulli({q:g}) erasure.  An erased leg's collision channel is
disabled for the complete window; the leg is not lifted.  Evaluation lasts
{DEFAULT_EVALUATION_CYCLES} cycles, or {DEFAULT_EVALUATION_CYCLES * WINDOWS_PER_CYCLE}
channel windows.  Body-yaw commands are zero-order held at that same 50-sample
per-cycle resolution.  Source XML files remain unchanged.

The primary performance measure is direct signed COM forward displacement in
meters.  `iid_contact_erasure_trials.csv` contains every raw trial;
`iid_contact_erasure_summary.csv` contains condition-level mean, median, SD and
IQR; `matched_clean_references.csv` contains q=0 comparisons produced with the
same numerical discretization.  `validation.json` records completion,
independence and contact-erasure gates.
"""
    (output / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps(validation, indent=2), flush=True)
    if not validation["passed"]:
        raise RuntimeError("global validation failed")


def run_segments(args) -> None:
    root = args.repo_root.resolve()
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for n_segments in args.segments:
        for wave in args.reference_waves:
            run_condition(
                root, output, n_segments, wave, args.trials, args.q,
                args.warmup_cycles, args.evaluation_cycles, args.force,
            )
            clean_path = (
                output / "parts" / f"N{n_segments:02d}"
                / f"clean_reference_{condition_tag(wave, n_segments)}.csv"
            )
            if args.force or len(read_csv(clean_path)) != 1:
                write_csv(clean_path, [clean_reference(
                    root, n_segments, wave, args.warmup_cycles, args.evaluation_cycles,
                )])


def parser() -> argparse.ArgumentParser:
    root = step37.repo_root()
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--mode", choices=("run", "finalize"), required=True)
    result.add_argument("--repo-root", type=Path, default=root)
    result.add_argument("--output-dir", type=Path, default=root / "exp_data" / "reliability" / OUTPUT_NAME)
    result.add_argument("--segments", type=int, nargs="+", default=list(step37.SEGMENTS))
    result.add_argument("--reference-waves", type=float, nargs="+", default=list(step37.REFERENCE_WAVES))
    result.add_argument("--trials", type=int, default=DEFAULT_TRIALS)
    result.add_argument("--q", type=float, default=DEFAULT_Q)
    result.add_argument("--warmup-cycles", type=int, default=DEFAULT_WARMUP_CYCLES)
    result.add_argument("--evaluation-cycles", type=int, default=DEFAULT_EVALUATION_CYCLES)
    result.add_argument("--force", action="store_true")
    return result


def main() -> None:
    args = parser().parse_args()
    if not (0.0 <= args.q <= 1.0):
        raise ValueError("q must be in [0,1]")
    if args.trials <= 0 or args.evaluation_cycles <= 0 or args.warmup_cycles < 0:
        raise ValueError("trial/cycle counts are invalid")
    if args.mode == "run":
        run_segments(args)
    else:
        finalize(
            args.repo_root.resolve(), args.output_dir.resolve(),
            sorted(args.segments), sorted(args.reference_waves), args.trials, args.q,
        )


if __name__ == "__main__":
    main()
