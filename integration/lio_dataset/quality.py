"""Exclude ambiguous simulated image measurement stamps, preserve every source."""
import hashlib
import json
from pathlib import Path
import shutil
from collections import defaultdict


def filter_dataset(output):
    import rosbag
    output=Path(output);variants=defaultdict(set);counts=defaultdict(int)
    with rosbag.Bag(str(output/'sensors.bag')) as bag:
        for _,msg,_ in bag.read_messages(topics=['/lio_dataset/rgb/compressed']):
            stamp=msg.header.stamp.to_nsec();variants[stamp].add(hashlib.sha256(bytes(msg.data)).hexdigest());counts[stamp]+=1
    ambiguous={stamp for stamp,value in variants.items() if len(value)>1}
    frames=json.loads((output/'dataset/frames.json').read_text());removed=[row for row in frames if row['stamp_ns'] in ambiguous]
    previous=json.loads((output/'quality_filter.json').read_text()) if (output/'quality_filter.json').exists() else {}
    excluded=previous.get('excluded_frames',[])
    if removed:
        for name in ['dataset/frames.json','dataset/frames.jsonl']:
            backup=output/(name.replace('/','_')+'.before_quality_filter')
            if not backup.exists():shutil.copyfile(output/name,backup)
        folder=output/'excluded_images';folder.mkdir(exist_ok=True)
        for row in removed:
            old=output/'dataset'/row['file_path'];destination=folder/old.name
            if destination.exists():raise ValueError('Excluded source image already exists')
            old.rename(destination)
            excluded.append(dict(stamp_ns=row['stamp_ns'],original_file_path=row['file_path'],preserved_path=str(destination.relative_to(output)),sha256=row['sha256'],reason='Same measurement stamp has different actual RGB payloads in the source bag'))
        frames=[row for row in frames if row['stamp_ns'] not in ambiguous]
        (output/'dataset/frames.json').write_text(json.dumps(frames,indent=2)+'\n')
        (output/'dataset/frames.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in frames))
    report=dict(passed=True,final_frames=len(frames),excluded_frame_count=len(excluded),excluded_frames=excluded,
        ambiguous_measurement_stamps=sorted(ambiguous),bag_variant_hashes={str(stamp):sorted(variants[stamp]) for stamp in sorted(ambiguous)},
        duplicate_messages_with_identical_bytes=sum(counts[s]-1 for s in counts if len(variants[s])==1),
        policy='Exclude all selected RGB frames whose stamp has multiple payload variants. Preserve excluded JPEGs and original bag. Never rewrite timestamps or synthesize poses.')
    (output/'quality_filter.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Ambiguous image stamps',len(ambiguous),'excluded selected frames',len(excluded),'curated frames',len(frames),flush=True)
    return report


if __name__=='__main__':
    import sys
    filter_dataset(sys.argv[1])
