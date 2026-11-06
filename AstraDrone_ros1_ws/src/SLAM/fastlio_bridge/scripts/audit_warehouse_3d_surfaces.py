#!/usr/bin/env python3
"""Offline shelf side-height and interior top-surface support audit.

Collision boxes are an evaluator-only reference. This is neither complete
visual-mesh ground truth nor a measure of reachable 3-D free-space coverage.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np

from warehouse_map_audit import box_local, read_xyz_pcd, side_samples, supported


def top_interior_samples(box, step, edge_margin):
    half = np.asarray(box['size'], dtype=float) / 2.
    if np.min(half[:2]) <= edge_margin:
        return np.empty((0, 3))
    coordinates = [np.linspace(-half[i] + edge_margin,
                               half[i] - edge_margin,
                               max(2, int(math.ceil((2*half[i]-2*edge_margin)/step))+1))
                   for i in range(2)]
    x, y = np.meshgrid(*coordinates)
    local = np.column_stack((x.ravel(), y.ravel(),
                             np.full(x.size, half[2])))
    c, s = math.cos(box['yaw']), math.sin(box['yaw'])
    result = np.column_stack((c*local[:, 0]-s*local[:, 1],
                              s*local[:, 0]+c*local[:, 1], local[:, 2]))
    return result + np.asarray(box['center'])


def exposed(points, box, boxes):
    keep = np.ones(len(points), dtype=bool)
    for other in boxes:
        if other is not box:
            keep &= ~(np.abs(box_local(points, other)) <
                      np.asarray(other['size'])/2 - 1e-5).all(axis=1)
    return points[keep]


def audit(scene, map_points, alignment, step=.20, tolerance=.12,
          side_bands=((.25, .70), (.70, 1.20), (1.20, 1.65), (1.65, 1.90))):
    if not (.05 <= tolerance < .14 and step > 0):
        raise ValueError('Top-face audit uses tolerance in [0.05,0.14) m')
    rotation = np.asarray(alignment['map_from_world_rotation'], dtype=float)
    translation = np.asarray(alignment['map_from_world_translation'], dtype=float)
    if (rotation.shape != (3, 3) or translation.shape != (3,) or
            not np.isfinite(rotation).all() or not np.isfinite(translation).all() or
            not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or
            np.linalg.det(rotation) < .99):
        raise ValueError('Invalid rigid world/map alignment')
    world_map = (map_points - translation) @ rotation
    boxes = scene['boxes']
    racks = [box for box in boxes if 'shelf' in box['name']]
    if not racks:
        raise ValueError('No collision-box shelves in scene')
    references, sections = [], []
    for index, (lower, upper) in enumerate(side_bands):
        if not lower < upper:
            raise ValueError('Invalid height band')
        points = [exposed(side_samples(box, step, lower, upper), box, boxes)
                  for box in racks]
        combined = np.vstack(points)
        references.append(combined)
        sections.append(('side_band_%d' % index, [lower, upper], len(combined)))
    # The inward 14 cm strip prevents vertical side points from satisfying a
    # top-surface match with the 12 cm Euclidean tolerance.
    margin = tolerance + .02
    tops = [exposed(top_interior_samples(box, step, margin), box, boxes)
            for box in racks]
    top = np.vstack(tops)
    references.append(top)
    sections.append(('shelf_top_interior', None, len(top)))
    all_reference = np.vstack(references)
    matched = supported(all_reference, world_map, tolerance)
    offset, stats = 0, []
    for name, band, count in sections:
        number = int(matched[offset:offset+count].sum())
        stats.append(dict(name=name, world_z_band_m=band,
                          reference_samples=count, supported_samples=number,
                          support_ratio=number/count if count else None))
        offset += count
    return dict(scope='Exposed shelf collision-box side bands plus TOP INTERIOR; '
                      'not complete visual mesh, reachable 3-D free space, or full 3-D map completeness.',
                sampling_step_m=step, euclidean_tolerance_m=tolerance,
                top_edge_margin_m=margin, shelf_count=len(racks),
                map_points=len(map_points), sections=stats)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', required=True)
    parser.add_argument('--pcd', required=True)
    parser.add_argument('--alignment', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    report = audit(json.loads(Path(args.scene).read_text()),
                   read_xyz_pcd(args.pcd),
                   json.loads(Path(args.alignment).read_text()))
    report.update(scene=args.scene, pcd=args.pcd, alignment=args.alignment)
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False,
                                            indent=2) + '\n')
    print(json.dumps(report['sections'], ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
