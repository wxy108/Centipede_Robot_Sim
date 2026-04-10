#!/usr/bin/env python3
"""
generate_terrain.py — Generate rough terrain heightfield PNGs
=============================================================
Creates grid-based rough terrain for centipede robot testing.

Terrain structure:
  - Arena divided into discrete 8×8 inch tiles covering the entire space
  - Each tile is a flat platform at one of several height levels
  - NO smoothing — sharp tile edges (realistic step obstacles)
  - NO flat start zone — robot drops from above onto the terrain
  - Height levels maintain a 1× : 2× : 3× ratio

Output:
  - PNG heightfield image for MuJoCo (grayscale, 8-bit)
  - Terrain metadata JSON (for reproducibility)

Usage:
    python scripts/generate_terrain.py                          # 3 default levels
    python scripts/generate_terrain.py --levels 3 --base 0.8    # 0.8, 1.6, 2.4 in
    python scripts/generate_terrain.py --n-seeds 5              # 5 random seeds
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np
from PIL import Image

PROJECT_ROOT = Path(__file__).resolve().parent.parent
TERRAIN_DIR = PROJECT_ROOT / "terrain"

# ─────────────────────────────────────────────
# Unit conversions
# ─────────────────────────────────────────────
INCH_TO_M = 0.0254

# ─────────────────────────────────────────────
# Default terrain parameters
# ─────────────────────────────────────────────
TILE_SIZE_INCH = 8.0          # each tile is 8 × 8 inches
TILE_SIZE_M = TILE_SIZE_INCH * INCH_TO_M   # 0.2032 m

# Arena dimensions — 14m × 14m ≈ 10× the 15-seg total body length (~1.35m)
# Big enough to spawn any model in the center and have runway in all directions.
ARENA_WIDTH_M  = 14.0         # lateral (X direction)
ARENA_LENGTH_M = 14.0         # forward (Y direction)

# Heightfield resolution (pixels per meter)
HFIELD_RES = 200              # 200 px/m → ~5mm per pixel


def generate_terrain(base_height_inch=0.8, n_levels=3, seed=42,
                     arena_width=ARENA_WIDTH_M, arena_length=ARENA_LENGTH_M,
                     tile_size_m=TILE_SIZE_M, resolution=HFIELD_RES):
    """
    Generate a random grid terrain heightfield with discrete flat tiles.

    Each tile is a flat 8×8 inch platform at one of the height levels.
    No smoothing — tiles have sharp edges like real step obstacles.
    Tiles cover the entire arena with no flat start zone.

    Args:
        base_height_inch: Base height increment in inches (level 1).
                          Level k = k * base_height_inch inches.
        n_levels: Number of height levels (e.g., 3 → 1×, 2×, 3× base).
        seed: Random seed for reproducibility.
        arena_width: Arena width in meters (X, lateral).
        arena_length: Arena length in meters (Y, forward).
        tile_size_m: Tile side length in meters.
        resolution: Heightfield pixels per meter.

    Returns:
        dict with 'image' (numpy array, uint8), 'metadata' (dict)
    """
    rng = np.random.RandomState(seed)

    # Height levels in meters: [1*base, 2*base, 3*base]
    # NO level 0 — every tile is raised so there are no flat spots that
    # would visually merge with the backup floor plane.
    heights_m = [k * base_height_inch * INCH_TO_M for k in range(1, n_levels + 1)]
    z_top = max(heights_m) * 1.1  # 10% headroom for MuJoCo scaling

    # MuJoCo heightfield mapping:
    #   image rows → Y axis (row 0 = -y_half, last row = +y_half)
    #   image cols → X axis (col 0 = -x_half, last col = +x_half)
    #
    # Robot body chain extends along Y, walks in +Y direction.

    img_w = int(arena_width * resolution)   # X = columns (lateral)
    img_h = int(arena_length * resolution)  # Y = rows (forward direction)

    # Tile dimensions in pixels
    tile_px = int(tile_size_m * resolution)

    # Number of tiles
    n_tiles_x = int(np.ceil(arena_width / tile_size_m))    # lateral
    n_tiles_y = int(np.ceil(arena_length / tile_size_m))   # forward

    # Generate tile heights — ALL tiles are random, no flat zone
    tile_grid = np.zeros((n_tiles_y, n_tiles_x), dtype=float)
    for ty in range(n_tiles_y):
        for tx in range(n_tiles_x):
            tile_grid[ty, tx] = rng.choice(heights_m)

    # Render to pixel image — each tile is a flat block of pixels
    img = np.zeros((img_h, img_w), dtype=np.float64)
    for ty in range(n_tiles_y):
        for tx in range(n_tiles_x):
            y0 = ty * tile_px
            y1 = min(y0 + tile_px, img_h)
            x0 = tx * tile_px
            x1 = min(x0 + tile_px, img_w)
            img[y0:y1, x0:x1] = tile_grid[ty, tx]

    # NO Gaussian smoothing — discrete flat-top tiles with sharp edges

    # Normalize to [0, 255] for PNG (0 = ground, 255 = z_top)
    if z_top > 0:
        img_uint8 = np.clip(img / z_top * 255, 0, 255).astype(np.uint8)
    else:
        img_uint8 = np.zeros((img_h, img_w), dtype=np.uint8)

    metadata = {
        'base_height_inch': base_height_inch,
        'n_levels': n_levels,
        'seed': seed,
        'arena_width_m': arena_width,
        'arena_length_m': arena_length,
        'tile_size_inch': TILE_SIZE_INCH,
        'tile_size_m': tile_size_m,
        'resolution_px_per_m': resolution,
        'img_width': img_w,
        'img_height': img_h,
        'n_tiles_x': n_tiles_x,
        'n_tiles_y': n_tiles_y,
        'heights_m': heights_m,
        'z_top_m': z_top,
        'tile_grid': tile_grid.tolist(),
    }

    return {'image': img_uint8, 'metadata': metadata}


def main():
    parser = argparse.ArgumentParser(
        description="Generate rough terrain heightfield PNGs for centipede testing")
    parser.add_argument("--base", type=float, default=0.8,
                        help="Base height increment in inches (default: 0.8)")
    parser.add_argument("--levels", type=int, default=3,
                        help="Number of raised height levels (default: 3)")
    parser.add_argument("--n-seeds", type=int, default=5,
                        help="Number of random seeds to generate (default: 5)")
    parser.add_argument("--seed-start", type=int, default=0,
                        help="Starting seed number (default: 0)")
    parser.add_argument("--arena-width", type=float, default=ARENA_WIDTH_M,
                        help=f"Arena width in meters (default: {ARENA_WIDTH_M})")
    parser.add_argument("--arena-length", type=float, default=ARENA_LENGTH_M,
                        help=f"Arena length in meters (default: {ARENA_LENGTH_M})")
    parser.add_argument("--resolution", type=int, default=HFIELD_RES,
                        help=f"Heightfield pixels per meter (default: {HFIELD_RES})")
    args = parser.parse_args()

    # Build output subdirectory name
    level_str = f"base{args.base:.1f}in_{args.levels}levels"
    out_dir = TERRAIN_DIR / level_str
    out_dir.mkdir(parents=True, exist_ok=True)

    print("=" * 70)
    print("ROUGH TERRAIN GENERATOR")
    print("=" * 70)
    print(f"  Tile size:     {TILE_SIZE_INCH}×{TILE_SIZE_INCH} inches "
          f"({TILE_SIZE_M*100:.1f}×{TILE_SIZE_M*100:.1f} cm)")
    print(f"  Base height:   {args.base} inches ({args.base * INCH_TO_M * 100:.2f} cm)")
    print(f"  Height levels: {args.levels} "
          f"({', '.join(f'{k*args.base:.1f}in' for k in range(1, args.levels+1))})")
    print(f"  Arena:         {args.arena_length}m × {args.arena_width}m")
    print(f"  Tiles fill:    entire arena (no flat zone)")
    print(f"  Smoothing:     NONE (discrete flat-top tiles)")
    print(f"  Resolution:    {args.resolution} px/m")
    print(f"  Seeds:         {args.seed_start} .. {args.seed_start + args.n_seeds - 1}")
    print(f"  Output:        {out_dir}/")
    print("=" * 70)

    all_meta = []
    for i in range(args.n_seeds):
        seed = args.seed_start + i
        result = generate_terrain(
            base_height_inch=args.base,
            n_levels=args.levels,
            seed=seed,
            arena_width=args.arena_width,
            arena_length=args.arena_length,
            resolution=args.resolution,
        )

        # Save PNG
        png_name = f"terrain_seed{seed}.png"
        png_path = out_dir / png_name
        Image.fromarray(result['image']).save(str(png_path))

        meta = result['metadata']
        meta['filename'] = png_name
        all_meta.append(meta)

        print(f"  Seed {seed}: {png_path.name}  "
              f"({meta['img_width']}×{meta['img_height']} px, "
              f"z_top={meta['z_top_m']*100:.2f} cm)")

    # Save metadata JSON
    json_path = out_dir / "terrain_meta.json"
    with open(json_path, 'w') as f:
        json.dump({
            'base_height_inch': args.base,
            'n_levels': args.levels,
            'tile_size_inch': TILE_SIZE_INCH,
            'heights_inch': [0] + [k * args.base for k in range(1, args.levels + 1)],
            'heights_m': all_meta[0]['heights_m'],
            'z_top_m': all_meta[0]['z_top_m'],
            'arena_width_m': args.arena_width,
            'arena_length_m': args.arena_length,
            'terrains': all_meta,
        }, f, indent=2)
    print(f"\n  Metadata: {json_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
