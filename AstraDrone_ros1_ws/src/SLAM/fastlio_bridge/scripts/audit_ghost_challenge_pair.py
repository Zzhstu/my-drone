#!/usr/bin/env python3
"""Audit a completed half-warehouse ghost challenge pair from saved artifacts."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from warehouse_map_audit import read_xyz_pcd


def read_json(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def audit(prefix):
    prefix = str(prefix)
    result = read_json(prefix + '_result.json')
    rows = [json.loads(line) for line in Path(prefix + '_challenge.jsonl').read_text(
        encoding='utf-8').splitlines() if line]
    decisions = [row for row in rows if row['event'] == 'decision']
    if len(decisions) != 1:
        raise ValueError(prefix + ': expected exactly one challenge decision')
    decision = decisions[0]
    samples = [row for row in rows if row['event'] == 'sample']
    pre_return = [row for row in samples if row['sim_time'] >= decision['sim_time']
                  and row['mission_phase'] in ('EXPLORING', 'REVISITING')]
    cleared = next((row for row in samples if row['sim_time'] >= decision['sim_time']
                    and row['map_roi_points'] == 0 and row['suspect_roi_points'] == 0), None)
    shelf = read_json(prefix + '_shelf_audit.json')
    near = [rack for rack in shelf['racks'] if rack['center_world'][0] <= 4.]
    near_supported = sum(rack['supported_samples'] for rack in near)
    near_reference = sum(rack['reference_samples'] for rack in near)
    scene = read_json(prefix + '_scene.json')
    alignment = read_json(prefix + '_alignment.json')
    pcd = Path(prefix + '_final_map.pcd')
    meta = read_json(prefix + '_final_map.json')
    actual_sha = hashlib.sha256(pcd.read_bytes()).hexdigest()
    if actual_sha != meta['sha256']:
        raise ValueError(prefix + ': final PCD SHA-256 mismatch')
    rotation = np.asarray(alignment['map_from_world_rotation'], dtype=float)
    translation = np.asarray(alignment['map_from_world_translation'], dtype=float)
    world = (read_xyz_pcd(str(pcd)) - translation) @ rotation
    final_roi = (np.linalg.norm(world[:, :2] - [-8., -4.], axis=1) < .4) & \
                (world[:, 2] > .3) & (world[:, 2] < 1.9)
    history = read_json(prefix + '_status_history.json')
    revisits = [row for row in history if row.get('mission_phase') == 'REVISITING']
    revisit_phase = None
    if revisits:
        a, b = revisits[0], revisits[-1]
        a_status = a['ghost_revisit']['map_status']
        b_status = b['ghost_revisit']['map_status']
        revisit_phase = dict(
            duration_sim_seconds=b['mission_sim_seconds']-a['mission_sim_seconds'],
            cleared_by_free_evidence_delta=b_status['cleared_by_free_evidence']-
                                           a_status['cleared_by_free_evidence'],
            pending_first=a_status['pending_voxels'], pending_last=b_status['pending_voxels'])
    mission = result['mission']
    return dict(
        prefix=prefix, arm=result['run_options']['ghost_challenge_arm'],
        valid_trigger=result['map_saved'] and result['termination'] == 'returned_home_and_saved'
            and decision['person_departure_xy_m'] >= 2.5 and
            decision['map_roi_points'] >= 5 and decision['suspect_roi_points'] >= 1,
        scene_source_sha256=scene['source_sha256'],
        ghost_suspect_max_hits=result['configuration']['/static_map_builder/ghost_suspect_max_hits'],
        decision=dict(mission_sim_seconds=decision['mission_sim_seconds'],
                      pedestrian_departure_m=decision['person_departure_xy_m'],
                      map_roi_points=decision['map_roi_points'],
                      suspect_roi_points=decision['suspect_roi_points'],
                      global_pending=decision['ghost_status']['pending_voxels'],
                      global_free_cleared=decision['ghost_status']['cleared_by_free_evidence']),
        last_pre_return=dict(mission_sim_seconds=pre_return[-1]['mission_sim_seconds'],
                             map_roi_points=pre_return[-1]['map_roi_points'],
                             suspect_roi_points=pre_return[-1]['suspect_roi_points']),
        first_observed_empty=None if cleared is None else dict(
            mission_sim_seconds=cleared['mission_sim_seconds'],
            elapsed_since_decision_sim_seconds=cleared['mission_sim_seconds']-
                                               decision['mission_sim_seconds'],
            mission_phase=cleared['mission_phase'],
            global_free_cleared=cleared['ghost_status']['cleared_by_free_evidence']),
        revisit_phase=revisit_phase,
        final=dict(termination=result['termination'], finish_reason=mission['finish_reason'],
                   home_confirmed=mission['home_confirmed'],
                   map_saved=result['map_saved'], map_points=meta['points'],
                   map_sha256=actual_sha, roi_points=int(final_roi.sum()),
                   pending_voxels=mission['ghost_revisit']['map_status']['pending_voxels'],
                   travel_xy_m=mission['travel_xy_m'],
                   mission_sim_seconds=mission['mission_sim_seconds'],
                   wall_elapsed_seconds=result['wall_elapsed_seconds'],
                   shelf_side_support_proxy=shelf['shelf_side_coverage'],
                   near_shelf_side_support_proxy=near_supported / near_reference))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline')
    parser.add_argument('revisit')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    baseline, revisit = audit(args.baseline), audit(args.revisit)
    if baseline['arm'] != 'baseline' or revisit['arm'] != 'revisit':
        raise ValueError('Pair arm labels are not baseline/revisit')
    if baseline['scene_source_sha256'] != revisit['scene_source_sha256'] or \
            baseline['ghost_suspect_max_hits'] != revisit['ghost_suspect_max_hits']:
        raise ValueError('Pair source world or suspect gate differs')
    report = dict(baseline=baseline, revisit=revisit,
                  scope='One sequential Gazebo/PX4 SITL pair with different scans/trajectories. '
                        'ROI counts are local point/voxel proxies; all-map free-clear counters are '
                        'not exact ROI deletion counts. Final shelf support cannot identify false '
                        'deletions without a matched pre-decision static map snapshot.')
    Path(args.output).write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n',
                                 encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
