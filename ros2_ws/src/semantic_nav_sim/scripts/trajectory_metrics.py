"""Auditable centerline grass exposure from recorded map-frame poses."""
import math


def grass_metrics(samples, scenario):
    if not samples:
        return {'grass_path_m': None, 'grass_central_crossing': None}
    cx, cy = scenario.get('grass_pose', [0., -4.])
    sx, sy = scenario.get('grass_size', [3., 3.])
    exposure = 0.
    crossing = False
    for a, b in zip(samples, samples[1:]):
        if b[0] <= a[0]:
            continue
        distance = math.hypot(b[1]-a[1], b[2]-a[2])
        # Subdivide to <=1 cm to avoid missing a rectangle boundary between samples.
        steps = max(1, math.ceil(distance/.01))
        for i in range(steps):
            f = (i+.5)/steps
            x, y = a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2])
            if abs(x-cx)<sx/2 and abs(y-cy)<sy/2:
                exposure += distance/steps
                if abs(x-cx)<1.:
                    crossing = True
    return {'grass_path_m': round(exposure, 4), 'grass_central_crossing': crossing}
