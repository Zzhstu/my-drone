#!/usr/bin/env python3
"""Evaluator-only same-input motion results; no truth enters detection."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np


def rows(path):
    # v1's uint8 predicted stream used character output; preserve and decode
    # that instrumentation mistake rather than silently editing the artifact.
    raw=Path(path).read_text().replace('\0','0').replace('\1','1')
    return [row for row in csv.DictReader(raw.splitlines()) if all(row.values())]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--flight-prefix',type=Path,required=True)
    parser.add_argument('--replay-prefix',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    flight=str(args.flight_prefix);prefix=str(args.replay_prefix)
    alignment=json.loads(Path(flight+'_alignment.json').read_text())
    truth=json.loads(Path(flight+'_pedestrian_truth.json').read_text())['samples']
    t=np.array([r['sim_stamp'] for r in truth]);world=np.array([r['world_xyz'] for r in truth])
    rotation=np.array(alignment['map_from_world_rotation']);translation=np.array(alignment['map_from_world_translation'])
    xyz=world@rotation.T+translation
    def actor(stamp):return np.array([np.interp(stamp,t,xyz[:,i]) for i in range(3)])
    detected=rows(prefix+'_objects.csv');frames=rows(prefix+'_frames.csv')
    arms=sorted(set(r['arm'] for r in frames))
    summary={}
    for arm in arms:
        object_rows=[r for r in detected if r['arm']==arm]
        match=[];unmatched=[]
        for row in object_rows:
            distance=float(np.linalg.norm(np.array([float(row['x']),float(row['y'])])-actor(float(row['stamp']))[:2]))
            (match if distance<=.75 else unmatched).append(row)
        wall=[r for r in object_rows if 19<float(r['x'])<19.5 and 9<float(r['y'])<11 and 224<float(r['stamp'])<230]
        history=json.loads(Path(prefix+'_'+arm+'_history.json').read_text())['records']
        pending=[r for r in history if r['state']=='suspected']
        wall_pending=[r for r in pending if 19<r['position'][0]<19.5 and 9<r['position'][1]<11]
        summary[arm]=dict(processed_frames=sum(r['arm']==arm for r in frames),
            dynamic_object_messages=len(object_rows),actor_matched_messages=len(match),
            unmatched_messages=len(unmatched),wall_dynamic_messages=len(wall),
            first_actor_detection=float(match[0]['stamp']) if match else None,
            last_actor_detection=float(match[-1]['stamp']) if match else None,
            final_pending=len(pending),final_wall_pending=len(wall_pending),
            final_actionable=sum(r['actionable'] for r in pending),
            unmatched_details=unmatched,wall_details=wall)
    args.output.write_text(json.dumps(dict(scope='Direct callbacks in recorded acquisition time, matched scan/odom; evaluator-only initial rigid transform and 0.75m XY actor association. Message counts, not full recall: scan visibility denominator not assessed here. Same-input baseline is flags-off production numerical logic, not original callback scheduling.',arms=summary),indent=2)+'\n')
    print(json.dumps({a:{k:v for k,v in s.items() if not k.endswith('details')} for a,s in summary.items()},indent=2))


if __name__=='__main__':main()
