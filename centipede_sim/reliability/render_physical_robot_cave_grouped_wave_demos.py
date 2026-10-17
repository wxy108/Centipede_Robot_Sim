"""Render corrected physical-robot demos with half-wave cave pitch grouping."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import cave_grouped_wave_controller as cave
import render_center_rooted_physical_robot_wave_demos as physical


OUTPUT_NAME = "step32_physical_robot_cave_grouped_wave_demos"


def _plot_cave_assignments(output_dir: Path, n_wave_values: list[float]) -> None:
    figure_dir = output_dir / "figures"
    figure_dir.mkdir(parents=True, exist_ok=True)
    for n_wave in n_wave_values:
        delta = -2.0 * np.pi * float(n_wave) / 9.0
        segment_index = np.arange(10)
        segment_phase = (segment_index - 0.5) * delta
        cave_index = np.floor((segment_phase + cave.CAVE_BOUNDARY_TOL) / np.pi).astype(int)
        right_down = (cave_index % 2 == 0).astype(int)
        joint_index = np.arange(9)
        joint_phase = joint_index * delta
        joint_wave = np.sin(joint_phase)

        fig, axes = plt.subplots(2, 1, figsize=(8.0, 4.6), sharex=True)
        axes[0].plot(joint_index + 0.5, joint_wave, "o-", color="#355f8a")
        axes[0].axhline(0.0, color="#555555", linewidth=0.8)
        axes[0].set_ylabel("Body yaw / amplitude")
        axes[0].grid(alpha=0.2)
        axes[1].step(segment_index, right_down, where="mid", color="#24796f", linewidth=2.2)
        axes[1].scatter(segment_index, right_down, c=cave_index, cmap="tab20", s=55, zorder=3)
        for i, c in enumerate(cave_index):
            axes[1].text(i, right_down[i] + (0.11 if right_down[i] == 0 else -0.14), f"c={c}", ha="center", fontsize=8)
        axes[1].set_yticks([0, 1], ["left down", "right down"])
        axes[1].set_xticks(segment_index)
        axes[1].set_xlabel("Segment index")
        axes[1].set_ylim(-0.3, 1.3)
        axes[1].grid(alpha=0.2)
        fig.suptitle(rf"Cave-collocated step pattern, $N_{{wave}}={n_wave:g}$")
        fig.tight_layout()
        tag = f"{n_wave:04.1f}".replace(".", "p")
        fig.savefig(figure_dir / f"cave_assignment_Nwave{tag}.png", dpi=220)
        plt.close(fig)


def run(args: argparse.Namespace) -> None:
    output_dir = args.output_dir.resolve()
    audit = cave.validate_cave_contract(
        tuple(float(value) for value in args.n_wave), n_segments=args.n_segments
    )
    if not audit["passed"]:
        raise RuntimeError("cave-grouping contract failed")

    physical.CONTROLLER_CLASS = cave.CaveGroupedContactPatternController
    physical.run(args)

    (output_dir / "cave_grouping_validation.json").write_text(
        json.dumps(audit, indent=2), encoding="utf-8"
    )
    _plot_cave_assignments(output_dir, list(args.n_wave))

    configuration_path = output_dir / "run_configuration.json"
    configuration = json.loads(configuration_path.read_text(encoding="utf-8"))
    configuration.update(
        {
            "controller_source": "cave_grouped_wave_controller.py",
            "controller_semantics": audit["definition"],
            "supersedes_for_spatial_pitch_semantics": (
                "step31_physical_robot_wave_number_demos"
            ),
        }
    )
    configuration_path.write_text(
        json.dumps(configuration, indent=2), encoding="utf-8"
    )

    audit_by_wave = {float(row["N_wave"]): row for row in audit["rows"]}
    flagged_one = audit_by_wave[1.0]
    flagged_one_half = audit_by_wave[1.5]
    readme = f"""# Step 32: cave-grouped physical robot wave demos

This is the corrected replacement for the Step-31 spatial pitch pattern.
Step 31 evaluated an independent continuous pitch precursor at every segment;
that can split one visible body-curvature half-wave into different pitch
states.  Those videos must not be used as the final wave-number demos.

Here body-yaw actuators are treated as samples between adjacent segment
centers.  If body joint `j` has phase `omega*t+j*Delta_phi`, segment `i` has
the collocated phase `omega*t+(i-1/2)*Delta_phi`.  The integer half-wave index
is `floor(theta_segment/pi)`.  Every segment with the same index receives one
identical right/left stance choice; the opposite side is always complementary.

Validated wave numbers: {', '.join(f'{value:g}' for value in args.n_wave)}.
For the originally flagged cases, the legacy Step-31 command was uniform in
only {flagged_one['legacy_step31_same_cave_uniform_fraction']:.1%} of
true cave groups at `N_wave=1` and
{flagged_one_half['legacy_step31_same_cave_uniform_fraction']:.1%} at
`N_wave=1.5`; the corrected controller is 100%.
See `cave_grouping_validation.json`, `figures/`, and `demos/`.
"""
    (output_dir / "README.md").write_text(readme, encoding="utf-8")


def build_parser() -> argparse.ArgumentParser:
    parser = physical.build_parser()
    repo_root = physical._repo_root()
    parser.set_defaults(
        output_dir=repo_root / "exp_data" / "reliability" / OUTPUT_NAME,
        n_wave=[1.0, 1.5],
    )
    return parser


if __name__ == "__main__":
    run(build_parser().parse_args())
