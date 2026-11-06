"""Resume offline acceptance after an auditor/environment error; never recapture or invent data."""
import argparse
import json
from pathlib import Path
import shutil
from audit import audit
from provenance import sha,verify_sources


def run(output):
    output=Path(output)
    original=output/'result_initial.json'
    if not original.exists():shutil.copyfile(output/'result.json',original)
    capture=json.loads(original.read_text())
    verify_sources(output,capture)
    session_check=None
    if (output/'source_sessions.json').exists():
        from multisession import verify_bundle
        session_check=verify_bundle(output)
    if not (capture.get('landed') and capture.get('disarmed') and capture.get('flight_finished')
            and capture.get('visual_updates',0)>10 and capture.get('colored_points',0)>100
            and capture.get('input_status',{}).get('errors',1)==0):
        raise RuntimeError('Acquisition itself did not finish successfully; cannot recover with offline audit')
    if sha(output/'rgb_pt.pcd')!=capture['rgb_map_sha256']:raise RuntimeError('Native map changed')
    frames=json.loads((output/'dataset/frames.json').read_text())
    for row in frames:
        if sha(output/'dataset'/row['file_path'])!=row['sha256']:raise RuntimeError('Image changed')
    if capture.get('scene')=='warehouse20':
        from warehouse20_audit import audit as mesh_audit
        coverage=mesh_audit(output)
    else:coverage=audit(output)
    tools={str(Path(__file__).resolve()):sha(__file__)}
    for name in ('audit.py','provenance.py'):
        p=Path(__file__).with_name(name).resolve();tools[str(p)]=sha(p)
    if capture.get('scene')=='warehouse20':
        p=Path(__file__).with_name('warehouse20_audit.py').resolve();tools[str(p)]=sha(p)
    recheck=dict(original_result_sha256=sha(original),source_sha256=tools,
        original_error=capture.get('error'),capture_sources_verified_from_snapshot=True,
        method='Same fixed targets, 20cm support and 100% criterion; exact measured map and indexed image/pose verification',
        coverage=coverage)
    if session_check:recheck['measured_session_verification']=session_check
    (output/'offline_recheck.json').write_text(json.dumps(recheck,indent=2)+'\n')
    capture.update(coverage=coverage,source_unchanged=True,success=coverage['passed'],
                   offline_recheck='offline_recheck.json',initial_result='result_initial.json')
    capture['output']=str(output.resolve())
    if session_check:
        capture.update(mode='offline_measured_multisession',reconstruction_scope=session_check['scope'],
            supplemental_images=session_check['supplemental_images'],primary_flight_finished=True)
    capture.pop('error',None)
    (output/'result.json').write_text(json.dumps(capture,indent=2)+'\n')
    print(json.dumps({k:coverage[k] for k in ('passed','total_samples','missing_samples','complete_fraction')}))
    if not coverage['passed']:raise RuntimeError('Still incomplete; 3DGS gate remains closed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture');run(p.parse_args().capture)
