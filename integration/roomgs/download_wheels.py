"""Parallel range download of pinned official wheels, verified by upstream SHA256."""
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor,as_completed
import email
import hashlib
import html
import json
import os
import re
import time
import urllib.parse
import urllib.request
from pip._vendor.packaging.requirements import Requirement

ROOT=Path(__file__).resolve().parents[2]
DEST=ROOT/'.envs/roomgs_wheels'


def request(url):return urllib.request.urlopen(url,timeout=60)


def official_wheel(package,version):
    data=json.load(request('https://pypi.org/pypi/%s/%s/json'%(package,version)))
    candidates=[r for r in data['urls'] if r['filename'].endswith('.whl') and 'x86_64' in r['filename']
                and 'manylinux' in r['filename'] and ('cp311' in r['filename'] or 'py3-none' in r['filename'])]
    if not candidates:raise RuntimeError('No compatible official wheel: '+package)
    row=candidates[0]
    return row['url'],row['filename'],row['size'],row['digests']['sha256']


def torch_wheel(package,version):
    page=request('https://download.pytorch.org/whl/cu128/'+package+'/').read().decode()
    for raw in re.findall(r'href="([^"]+)"',page):
        raw=html.unescape(raw);decoded=urllib.parse.unquote(raw)
        if (package+'-'+version+'+cu128-cp311-cp311-' in decoded and 'x86_64.whl' in decoded):
            link=urllib.parse.urljoin('https://download.pytorch.org/whl/cu128/',raw)
            url,fragment=urllib.parse.urldefrag(link)
            digest=urllib.parse.parse_qs(fragment)['sha256'][0]
            row=request(urllib.request.Request(url,headers={'Range':'bytes=0-0'}))
            size=int(row.headers['Content-Range'].split('/')[-1]);row.close()
            return url,urllib.parse.unquote(url.rsplit('/',1)[-1]),size,digest
    raise RuntimeError('Pinned Torch wheel not found')


def download(spec):
    url,name,size,digest=spec;path=DEST/name
    if path.exists() and path.stat().st_size==size and hashlib.file_digest(path.open('rb'),'sha256').hexdigest()==digest:
        print('Verified cached '+name,flush=True);return
    partial=DEST/(name+'.partial');prefix=DEST/(name+'.prefix');start=0
    fd=os.open(partial,os.O_CREAT|os.O_RDWR,0o644);os.ftruncate(fd,size)
    if prefix.exists():
        start=min(prefix.stat().st_size,size)
        with prefix.open('rb') as f:
            offset=0
            while offset<start:
                block=f.read(min(4*1024**2,start-offset));os.pwrite(fd,block,offset);offset+=len(block)
    chunk=16*1024**2;ranges=[(a,min(a+chunk,size)-1) for a in range(start,size,chunk)]
    print('Downloading '+name+' bytes='+str(size)+' preserved='+str(start),flush=True)
    def part(bounds):
        low,high=bounds
        for attempt in range(5):
            try:
                req=urllib.request.Request(url,headers={'Range':'bytes=%d-%d'%(low,high)})
                with request(req) as response:
                    if response.status!=206 or not response.headers.get('Content-Range','').startswith('bytes %d-'%low):
                        raise RuntimeError('Server did not honour range request')
                    offset=low
                    while offset<=high:
                        block=response.read(min(1024**2,high-offset+1))
                        if not block:raise RuntimeError('Range ended early')
                        os.pwrite(fd,block,offset);offset+=len(block)
                return high-low+1
            except Exception:
                if attempt==4:raise
                time.sleep(attempt+1)
    try:
        with ThreadPoolExecutor(max_workers=8) as pool:
            done=start
            for future in as_completed([pool.submit(part,r) for r in ranges]):
                done+=future.result()
                print('  %.1f%% %s'%(done/size*100,name),flush=True)
    finally:os.close(fd)
    with partial.open('rb') as f:actual=hashlib.file_digest(f,'sha256').hexdigest()
    if actual!=digest:raise RuntimeError('SHA256 mismatch: '+name)
    partial.rename(path)
    print('SHA256 verified '+name,flush=True)


def main():
    DEST.mkdir(parents=True,exist_ok=True)
    torch_spec=torch_wheel('torch','2.7.1')
    metadata=email.message_from_bytes(request(torch_spec[0]+'.metadata').read())
    specs=[]
    for raw in metadata.get_all('Requires-Dist',[]):
        requirement=Requirement(raw)
        if requirement.marker and not requirement.marker.evaluate():continue
        if requirement.name.startswith('nvidia-') or requirement.name=='triton':
            version=str(requirement.specifier).removeprefix('==')
            specs.append(official_wheel(requirement.name,version))
    # Finish the preserved large wheel first, then the large Torch binary.
    specs.sort(key=lambda s:0 if 'cudnn' in s[1] else 1)
    specs.insert(1,torch_spec);specs.append(torch_wheel('torchvision','0.22.1'))
    (DEST/'upstream_manifest.json').write_text(json.dumps([dict(url=s[0],filename=s[1],size=s[2],sha256=s[3]) for s in specs],indent=2))
    for spec in specs:download(spec)


if __name__=='__main__':main()
