#!/usr/bin/env python3
"""Generate maps/arena_lidar_view.pgm + .yaml for worlds/arena.sdf.

This is NOT a generic mapping tool — it's a deliberate stand-in for
"what a lidar-based SLAM run over this arena would have produced
before this experiment's glass_wall/grass_patch scenario existed."

That framing matters for experimental validity, not just plumbing:
Nav2's global costmap needs a real static map to size itself (an
identity/naive map<->odom transform with no map_server leaves the
global costmap defaulting to a tiny, wrongly-placed window — hit and
diagnosed in Milestone 5). But baking the glass wall into that map as
a known obstacle would let the global planner route around it via the
static layer — even though the live obstacle_layer (fed by the same
lidar that can't see glass) never would. That would silently invalidate
the baseline failure mode this whole project exists to demonstrate.

So: boundary walls are drawn (a lidar-based mapping run would see
them). glass_wall and grass_patch are NOT drawn — the former because
lidar genuinely can't see it (matching Milestone 2), the latter
because it was never an obstacle in the first place.

Wall coordinates below are hand-copied from worlds/arena.sdf and must
be kept in sync with it by hand if that file ever changes — there is
no shared source of truth between the two right now.
"""

import os

from PIL import Image

RESOLUTION = 0.05  # meters/pixel, matches nav2_params.yaml's costmap resolution
MAP_HALF_EXTENT = 6.5  # meters; arena interior is [-6, 6], plus margin
ORIGIN_X = -MAP_HALF_EXTENT
ORIGIN_Y = -MAP_HALF_EXTENT

FREE = 254
OCCUPIED = 0

# (x_min, x_max, y_min, y_max) in world meters, from worlds/arena.sdf's
# wall_* models (pose +/- half of size, each wall 12.3 x 0.15 x 1.5).
WALLS_WORLD = [
    (-6.15, 6.15, 5.925, 6.075),    # wall_north
    (-6.15, 6.15, -6.075, -5.925),  # wall_south
    (5.925, 6.075, -6.15, 6.15),    # wall_east
    (-6.075, -5.925, -6.15, 6.15),  # wall_west
]


def world_to_pixel(x, y, width_px, height_px):
    """World meters -> pixel indices. Image row 0 is the map's top
    (max y); map_server's convention, not an arbitrary choice."""
    col = int(round((x - ORIGIN_X) / RESOLUTION))
    row_from_bottom = int(round((y - ORIGIN_Y) / RESOLUTION))
    row = height_px - 1 - row_from_bottom
    return col, row


def main():
    size_px = int(round((2 * MAP_HALF_EXTENT) / RESOLUTION))
    img = Image.new('L', (size_px, size_px), color=FREE)
    pixels = img.load()

    for (x_min, x_max, y_min, y_max) in WALLS_WORLD:
        c0, r1 = world_to_pixel(x_min, y_min, size_px, size_px)
        c1, r0 = world_to_pixel(x_max, y_max, size_px, size_px)
        for row in range(max(0, r0), min(size_px, r1 + 1)):
            for col in range(max(0, c0), min(size_px, c1 + 1)):
                pixels[col, row] = OCCUPIED

    out_dir = os.path.join(os.path.dirname(__file__), '..', 'maps')
    os.makedirs(out_dir, exist_ok=True)
    pgm_path = os.path.join(out_dir, 'arena_lidar_view.pgm')
    yaml_path = os.path.join(out_dir, 'arena_lidar_view.yaml')

    img.save(pgm_path)

    with open(yaml_path, 'w') as f:
        f.write(f"""image: arena_lidar_view.pgm
resolution: {RESOLUTION}
origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")

    print(f'Wrote {pgm_path} ({size_px}x{size_px} px)')
    print(f'Wrote {yaml_path}')


if __name__ == '__main__':
    main()
