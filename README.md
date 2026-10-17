# Centipede Robot Simulation

MuJoCo simulation of the CAD-derived centipede robot, including generated segment variants, flat-ground locomotion tests and rough-terrain benchmarks.

This is the maintained repository for the former [`Centipede_robot`](https://github.com/wxy108/Centipede_robot) project. All 45 files at that repository's source head (`163be1f3ffb78b153aac1415a629d570c6120af1`) were already present here byte-for-byte before consolidation. Its full history is an ancestor of this repository and is also preserved on [`legacy/centipede-robot/main`](https://github.com/wxy108/Centipede_Robot_Sim/tree/legacy/centipede-robot/main).

## Contents

| Path | Purpose |
| --- | --- |
| `model/` | MuJoCo XML models, including 3/5/6/7/8/15-segment variants and the original four-module model |
| `meshes/` | STL robot parts referenced by the models |
| `scripts/` | Model/terrain generation, locomotion tests, parameter sweeps and plotting |
| `terrain/` | Terrain heightfields and metadata |
| `output/` | Existing sensor, summary and video examples; new output is ignored by Git |
| `_archive/` | Original CAD/URDF/ROS exports and legacy examples |

## Running the existing examples

Use a Python environment with MuJoCo and NumPy installed. Terrain generation also uses Pillow; the terrain plotting script uses Matplotlib. Refer to each script's own arguments for additional options.

From the repository root:

```bash
# Open the five-segment model in the viewer
python scripts/test_models.py --viewer 5

# Run the existing headless model test without video recording
python scripts/test_models.py --no-video --segments 5
```

The scripts write generated data under `output/`. Repository consolidation did not rerun these simulations or change their control logic, models or parameters. Two tracked Python bytecode cache files were removed; source files and recorded examples are preserved.

## Related projects

- [Centipede_MuJoCo](https://github.com/wxy108/Centipede_MuJoCo): biological centipede locomotion research and controllers.
- [Centipede_isaac_rl](https://github.com/wxy108/Centipede_isaac_rl): Isaac Lab reinforcement learning and rendering reference.
- [Terrain_model](https://github.com/wxy108/Terrain_model): standalone printable terrain generation.
