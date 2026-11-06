"""Content-addressed acquisition snapshots survive later offline tool fixes."""
from pathlib import Path
import hashlib
import json
import shutil


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def snapshot_sources(output,fingerprints):
    output=Path(output);folder=output/'acquisition_sources';folder.mkdir(exist_ok=True)
    index={}
    for name,digest in fingerprints.items():
        if sha(name)!=digest:raise RuntimeError('Source changed before snapshot: '+name)
        dest=folder/digest
        if not dest.exists():shutil.copyfile(name,dest)
        index[name]=dict(sha256=digest,snapshot=str(dest.relative_to(output)))
    (output/'acquisition_sources.json').write_text(json.dumps(index,indent=2)+'\n')


def verify_sources(output,capture):
    output=Path(output);index_path=output/'acquisition_sources.json'
    index=json.loads(index_path.read_text()) if index_path.exists() else {}
    for name,digest in capture['runtime_source_sha256'].items():
        row=index.get(name)
        path=output/row['snapshot'] if row else Path(name)
        if (row and row['sha256']!=digest) or sha(path)!=digest:
            raise RuntimeError('Acquisition source snapshot mismatch: '+name)

