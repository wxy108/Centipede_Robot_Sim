"""Rebuild/verify Step-33 presentation outputs from completed CSVs only.

No trial is simulated again, except the short representative demo excerpts.
The simulation checkpoints and trial CSV remain untouched.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math

import numpy as np
from PIL import Image

import run_physical_robot_pitch_body_yaw_iid_wave_sweep as sweep


def run(skip_render: bool = False) -> None:
    root = sweep._repo_root()
    out = root / "exp_data" / "reliability" / sweep.OUTPUT_NAME
    config = json.loads((out / "run_configuration.json").read_text(encoding="utf-8"))
    trial_path = out / "iid_trials.csv"
    original_hash = hashlib.sha256(trial_path.read_bytes()).hexdigest()
    rows = sweep._read_csv(trial_path)
    waves = config["N_wave"]
    replays = config["trials_per_N_wave"]
    expected_count = replays * len(waves)
    assert len(rows) == expected_count, (len(rows), expected_count)
    identities = [(float(row["N_wave"]), int(row["seed"])) for row in rows]
    assert len(set(identities)) == expected_count
    for wave in waves:
        seeds = {seed for value, seed in identities if value == wave}
        assert seeds == set(range(replays)), (wave, sorted(seeds))
    for row in rows:
        assert row["status"] == "PASS", row
        displacement = float(row["forward_displacement_m"])
        absolute = float(row["absolute_forward_displacement_m"])
        assert math.isfinite(displacement)
        assert math.isclose(absolute, abs(displacement), abs_tol=1e-12)
        assert int(row["n_segments"]) == config["n_segments"]
        assert int(row["mujoco_warning_total"]) == 0
        assert math.isclose(float(row["integration_timestep_s"]), 0.001)
        assert math.isclose(float(row["evaluation_time_s"]), 18.850)
        pair_sum = sum(float(row[field]) for field in (
            "both_down_pair_fraction", "both_up_pair_fraction", "one_down_pair_fraction"
        ))
        assert math.isclose(pair_sum, 1.0, abs_tol=1e-12)
    audit = json.loads((out / "rollout_control_validation.json").read_text(encoding="utf-8"))
    assert audit["passed"]
    assert hashlib.sha256(sweep._model_path(root).read_bytes()).hexdigest() == audit["model_sha256"]
    summary = sweep._aggregate(rows, waves)
    sweep._write_csv(out / "iid_summary.csv", summary)
    sweep._plot_results(out, rows, summary)
    if not skip_render:
        sweep._render_demos(root, out, rows, waves,
                            warmup_cycles=config["warmup_cycles"], duration_s=6.0, fps=30)
    manifest = json.loads((out / "demo_manifest.json").read_text(encoding="utf-8"))
    assert len(manifest) == len(waves)
    assert {item["N_wave"] for item in manifest} == set(waves)
    import imageio.v2 as imageio
    videos = []
    for item in manifest:
        with imageio.get_reader(item["video"]) as reader:
            metadata = reader.get_meta_data()
            frame_count = reader.count_frames()
            first = reader.get_data(0)
            last = reader.get_data(frame_count - 1)
            assert first.shape == last.shape == (682, 960, 3)
            assert frame_count == 180, frame_count
            assert math.isclose(metadata["fps"], 30.0)
        with Image.open(item["preview"]) as preview:
            preview.verify()
        videos.append({"N_wave": item["N_wave"], "frames": frame_count,
                       "fps": metadata["fps"], "size": metadata["size"],
                       "video": item["video"]})
    figures = []
    for path in sorted((out / "figures").glob("*.png")):
        with Image.open(path) as image:
            figures.append({"file": path.name, "size": image.size})
            image.verify()
    assert len(figures) >= 5
    assert hashlib.sha256(trial_path.read_bytes()).hexdigest() == original_hash
    result = {"passed": True, "trial_count": len(rows),
              "trial_csv_sha256": original_hash,
              "raw_trial_csv_unchanged_by_finalization": True,
              "model_sha256": audit["model_sha256"],
              "figures": figures, "videos": videos,
              "notes": "CSV consistency, unique trials, image integrity and full video decode counts; visual inspection is separate."}
    (out / "artifact_validation.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--skip-render", action="store_true")
    run(parser.parse_args().skip_render)
