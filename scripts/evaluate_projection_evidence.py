#!/usr/bin/env python3
"""Evaluate manually annotated captures; world geometry is scoring-only."""
import argparse
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'ros2_ws/src/semantic_nav_perception'))
from semantic_nav_perception.projection import Camera, GroundProjector, UnavailableGeometry


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('capture', type=Path)
    args = parser.parse_args()
    directory = args.capture.resolve()
    if not directory.is_relative_to(ROOT):
        raise SystemExit('Evidence must remain inside this project')
    annotations = json.loads((directory/'annotations.json').read_text())
    # Withheld from GroundProjector: only the evaluator reads the SDF.
    glass = ET.parse(directory/'arena.sdf').find('./world/model[@name="glass_wall"]')
    pose = list(map(float, glass.findtext('pose').split()))
    size = list(map(float, glass.findtext('./link/collision/geometry/box/size').split()))
    expected = np.array([[pose[0]-size[0]/2, pose[1]+size[1]/2],
                         [pose[0]-size[0]/2, pose[1]-size[1]/2]])
    results = {}
    for name, annotation in annotations['views'].items():
        metadata = json.loads((directory/name/'calibration.json').read_text())
        scan = json.loads((directory/name/'scan.json').read_text())
        k = metadata['k']
        camera = Camera(metadata['width'], metadata['height'], k[0], k[4], k[2], k[5])
        parameters = dict(pixel_error_bound=2, max_range_m=10)
        def projector(limit):
            return GroundProjector(camera, metadata['rotation_world_from_optical'],
                                   metadata['camera_position'], max_sensitivity_m=limit,
                                   **parameters)
        record = dict(annotation=annotation, camera_position=metadata['camera_position'])
        center = round(-scan['angle_min']/scan['angle_increment'])
        record['forward_lidar_range_m'] = scan['ranges'][center]
        record['camera_to_glass_front_x_m'] = float(expected[0,0]-metadata['camera_position'][0])
        try:
            points = projector(100).visible_base(annotation['endpoints'],
                                                 base_visible=annotation['base_visible'])
            actual = np.array([[p.x,p.y] for p in points])
            record.update(projected_endpoints_m=actual.tolist(),
                          endpoint_errors_m=np.linalg.norm(actual-expected,axis=1).tolist(),
                          midpoint_error_m=float(np.linalg.norm(actual.mean(axis=0)-expected.mean(axis=0))),
                          length_error_m=float(abs(np.linalg.norm(actual[1]-actual[0])-size[1])),
                          orientation_error_deg=float(math.degrees(math.acos(np.clip(
                              abs(np.dot(actual[1]-actual[0],expected[1]-expected[0])) /
                              (np.linalg.norm(actual[1]-actual[0])*size[1]), -1, 1)))),
                          assumed_2px_sensitivity_m=[p.sensitivity_m for p in points])
            projector(.25).visible_base(annotation['endpoints'], base_visible=True)
            record['example_025m_gate'] = 'accepted'
        except UnavailableGeometry as error:
            record['example_025m_gate'] = 'rejected'
            record['rejection_reason'] = str(error)
        results[name] = record
    output = dict(annotation_method=annotations['method'],
                  limits='Manual stationary calibration only, not VLM accuracy. 2 px / 0.25 m are sensitivity examples, not validated model limits.',
                  expected_front_endpoints_m=expected.tolist(), views=results)
    (directory/'projection_evaluation.json').write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__ == '__main__':
    main()
