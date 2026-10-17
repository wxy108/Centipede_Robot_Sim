"""Independent command-array and model-order audit for the Step-33 sweep.

This does not change the simulation, its model, or any trial checkpoint.
Run before accepting the vectorized rollout results.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np

import run_physical_robot_pitch_body_yaw_iid_wave_sweep as sweep


def run() -> dict:
    root = sweep._repo_root()
    model_path = sweep._model_path(root)
    xml = ET.parse(model_path).getroot()
    actuators = list(xml.find("actuator"))
    expected_joints = []
    for segment in range(1, sweep.N_SEGMENTS + 1):
        expected_joints.extend(
            [f"Shoulder{segment}_R", f"Leg{segment}_R",
             f"Shoulder{segment}_L", f"Leg{segment}_L"]
        )
        if segment < sweep.N_SEGMENTS:
            expected_joints.extend([f"Link{segment}_Y", f"Link{segment}_P"])
    order_matches = [item.get("joint") for item in actuators] == expected_joints
    dt = float(xml.find("option").get("timestep"))
    assets = SimpleNamespace(
        model=SimpleNamespace(nu=len(actuators)),
        indices=sweep._build_actuator_indices(sweep.N_SEGMENTS),
        compute_ctrl=sweep._body_only_compute_ctrl,
        dt=dt,
    )
    settle_steps = math.ceil(sweep.SETTLE_TIME_S / dt)
    bridge_steps = math.ceil(sweep.BRIDGE_TIME_S / dt)
    period = 2 * math.pi / sweep.OMEGA_BODY
    window = period / 2
    warmup_steps = math.ceil(4 * period / dt)
    evaluation_steps = math.ceil(12 * period / dt)
    first_dynamic = settle_steps + bridge_steps
    initial_target = sweep._initial_target(assets.model, assets.indices)
    seeds = [0, 127]
    rows = []
    for wave in sweep.DEFAULT_N_WAVE:
        controls, eval_start, warmup_time = sweep._build_open_loop_controls(
            assets, seeds=seeds, n_wave=wave, warmup_cycles=4, evaluation_cycles=12
        )
        # Sample all stages, plus both sides of every decision boundary.
        indices = set(np.linspace(0, len(controls[0]) - 1, 257, dtype=int))
        for offset, duration, decision_origin in (
            (first_dynamic, warmup_steps, -warmup_time),
            (eval_start, evaluation_steps, 0.0),
        ):
            indices.update([offset, offset + duration - 1])
            for boundary in range(-9, 25):
                local = math.ceil((boundary * window - decision_origin) / dt)
                for delta in [-1, 0, 1]:
                    step = local + delta
                    if 0 <= step < duration:
                        indices.add(offset + step)
        max_error = 0.0
        for batch_index, seed in enumerate(seeds):
            controller = sweep.WaveNumberIIDController(
                assets.model.nu, assets.indices, assets.compute_ctrl,
                seed=seed, n_wave=wave,
            )
            bridge_target = controller.control(0.0, -warmup_time)
            for index in sorted(indices):
                if index < settle_steps:
                    expected = (index + 1) / settle_steps * initial_target
                elif index < first_dynamic:
                    step = index - settle_steps
                    alpha = 0.5 - 0.5 * math.cos(math.pi * (step + 1) / bridge_steps)
                    expected = (1 - alpha) * initial_target + alpha * bridge_target
                elif index < eval_start:
                    body_time = (index - first_dynamic) * dt
                    expected = controller.control(body_time, body_time - warmup_time)
                else:
                    eval_time = (index - eval_start) * dt
                    expected = controller.control(warmup_time + eval_time, eval_time)
                max_error = max(max_error, float(np.max(np.abs(
                    controls[batch_index, index] - expected
                ))))
        rows.append({
            "N_wave": wave,
            "seeds": seeds,
            "checked_time_indices_per_seed": len(indices),
            "max_absolute_control_error_rad": max_error,
            "all_control_values_finite": bool(np.all(np.isfinite(controls))),
            "passed": max_error < 1e-12 and bool(np.all(np.isfinite(controls))),
        })
    result = {
        "passed": bool(order_matches and all(row["passed"] for row in rows)),
        "model": str(model_path),
        "model_sha256": hashlib.sha256(model_path.read_bytes()).hexdigest(),
        "actuator_order_matches_physical_joint_names": order_matches,
        "actuator_count": len(actuators),
        "integration_timestep_s": dt,
        "scope": "XML actuator order and vectorized controls versus scalar controller; not a dynamics-convergence proof",
        "leg_yaw_contract": "constant servo position targets, not removed degrees of freedom",
        "rows": rows,
    }
    output = root / "exp_data" / "reliability" / sweep.OUTPUT_NAME
    (output / "rollout_control_validation.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, indent=2), flush=True)
    if not result["passed"]:
        raise RuntimeError("Step-33 command-array audit failed")
    return result


if __name__ == "__main__":
    run()
