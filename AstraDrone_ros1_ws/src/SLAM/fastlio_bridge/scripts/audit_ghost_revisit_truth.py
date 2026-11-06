#!/usr/bin/env python3
"""Offline final-map audit near vacated pedestrian trajectory in paired SITL runs.

This is an evaluator-only 2-D swept-path proxy, not a full 3-D ghost label.
No truth is sent to the mapper, detector, or planner.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from warehouse_map_audit import read_xyz_pcd


def audit(prefix):
    prefix = Path(prefix)
    result = json.loads(Path(str(prefix) + '_result.json').read_text())
    truth = json.loads(Path(str(prefix) + '_pedestrian_truth.json').read_text())['samples']
    alignment = json.loads(Path(str(prefix) + '_alignment.json').read_text())
    scene = json.loads(Path(str(prefix) + '_scene.json').read_text())
    if len(truth) < 2:
        raise ValueError('No pedestrian trajectory samples')
    path = np.asarray([sample['world_xyz'][:2] for sample in truth])
    end = path[-1]
    travel = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    if travel < 1.:
        raise ValueError('Pedestrian did not move at least 1 m; vacated-path audit invalid')
    vacated = path[np.linalg.norm(path - end, axis=1) > .8]
    cells = np.unique(np.floor(vacated / .20).astype(np.int32), axis=0)
    centers = (cells + .5) * .20
    points = read_xyz_pcd(str(prefix) + '_final_map.pcd')
    rot = np.asarray(alignment['map_from_world_rotation'])
    trans = np.asarray(alignment['map_from_world_translation'])
    world = (points - trans) @ rot
    # Evaluate person-sized height and corridor around the swept, now empty path.
    lo, hi = centers.min(axis=0) - .65, centers.max(axis=0) + .65
    keep = ((world[:, 2] >= .3) & (world[:, 2] <= 1.9) &
            np.all((world[:, :2] >= lo) & (world[:, :2] <= hi), axis=1) &
            (np.linalg.norm(world[:, :2] - end, axis=1) > .8))
    candidates = world[keep]
    # Remove known box interiors and their 15 cm surface band from this proxy.
    static = np.zeros(len(candidates), dtype=bool)
    for box in scene['boxes']:
        center = np.asarray(box['center'])
        if not len(candidates) or not (lo[0] - 2 < center[0] < hi[0] + 2 and
                                       lo[1] - 2 < center[1] < hi[1] + 2):
            continue
        d = candidates - center
        c, s = math.cos(box['yaw']), math.sin(box['yaw'])
        local = np.column_stack((c*d[:, 0] + s*d[:, 1],
                                 -s*d[:, 0] + c*d[:, 1], d[:, 2]))
        static |= np.all(np.abs(local) <= np.asarray(box['size'])/2 + .15, axis=1)
    candidates = candidates[~static]
    within = np.zeros(len(candidates), dtype=bool)
    supported = np.zeros(len(centers), dtype=bool)
    for index, center in enumerate(centers):
        near = np.sum((candidates[:, :2] - center)**2, axis=1) <= .35**2
        within |= near
        supported[index] = bool(np.any(near))
    flight = json.loads(Path(str(prefix) + '_flight_samples.json').read_text())
    flight_xy = np.asarray([row['map_xyz'][:2] for row in flight]) if flight else np.empty((0, 2))
    flight_m = float(np.linalg.norm(np.diff(flight_xy, axis=0), axis=1).sum()) if len(flight_xy)>1 else None
    history = json.loads(Path(str(prefix) + '_status_history.json').read_text())
    revisiting = [row for row in history if row.get('mission_phase') == 'REVISITING']
    revisit_delta = None
    if revisiting:
        first, last = revisiting[0], revisiting[-1]
        first_status = first.get('ghost_revisit', {}).get('map_status', {})
        last_status = last.get('ghost_revisit', {}).get('map_status', {})
        revisit_delta = {
            'duration_sim_seconds': last['mission_sim_seconds'] - first['mission_sim_seconds'],
            'pending_first': first_status.get('pending_voxels'),
            'pending_last': last_status.get('pending_voxels'),
            'free_ray_cleared_during_phase': last_status.get('cleared_by_free_evidence', 0) -
                                            first_status.get('cleared_by_free_evidence', 0),
            'occupied_reconfirmed_during_phase': last_status.get('reobserved_occupied', 0) -
                                                 first_status.get('reobserved_occupied', 0)}
    shelf_file = Path(str(prefix) + '_shelf_audit.json')
    shelf_support = json.loads(shelf_file.read_text()).get('shelf_side_coverage') if shelf_file.exists() else None
    report = {
        'source_prefix': str(prefix),
        'arm': result['run_options'].get('ghost_challenge_arm') or
               ('revisit' if result['run_options']['ghost_revisit'] else 'baseline'),
        'valid_pedestrian_walk': True,
        'pedestrian_samples': len(truth),
        'pedestrian_travel_m': travel,
        'vacated_path_unique_20cm_cells': len(centers),
        'residual_map_points_near_vacated_path': int(within.sum()),
        'vacated_cells_with_nearby_map_point': int(supported.sum()),
        'vacated_cells_with_nearby_map_point_ratio': float(supported.mean()),
        'flight_sampled_path_m': flight_m,
        'revisit_phase': revisit_delta,
        'shelf_side_support_proxy': shelf_support,
        'final_ghost_status': result['mission'].get('ghost_revisit', {}).get('map_status'),
        'wall_elapsed_seconds': result['wall_elapsed_seconds'],
        'mission_sim_seconds': result['mission'].get('mission_sim_seconds'),
        'finish_reason': result['mission'].get('finish_reason'),
        'map_saved': result['map_saved'],
        'termination': result['termination'],
        'scope': 'Final XYZ map near departed path, 0.3-1.9m high, 0.35m XY radius, excluding static boxes/current actor. '
                 'This is a trajectory-proximity proxy, not per-point dynamic truth or full 3-D completeness.'}
    out = Path(str(prefix) + '_pedestrian_ghost_audit.json')
    out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prefix', nargs='+')
    args = parser.parse_args()
    reports = [audit(prefix) for prefix in args.prefix]
    print(json.dumps(reports, ensure_ascii=False, indent=2))
