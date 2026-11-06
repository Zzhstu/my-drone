"""Measured capture limits; crossing a limit requests a normal landing."""
from pathlib import Path
import os
import shutil
import time

GIB=2**30


class ResourceMeter:
    def __init__(self,output,run_gib=7.,bag_gib=6.,rss_gib=6.,reserve_gib=20.):
        self.output=Path(output)
        self.limits=dict(run_bytes=int(run_gib*GIB),bag_bytes=int(bag_gib*GIB),
            process_tree_rss_bytes=int(rss_gib*GIB),reserve_disk_bytes=int(reserve_gib*GIB),minimum_available_memory_bytes=int(1.5*GIB))
        self.peak_run=0;self.peak_rss=0

    def sample(self):
        total=0;bag=0;dataset=0
        for root,_,files in os.walk(self.output):
            for name in files:
                p=Path(root)/name
                try:size=p.stat().st_size
                except FileNotFoundError:continue
                total+=size
                if '.bag' in name:bag+=size
                if 'dataset' in p.parts:dataset+=size
        processes={}
        for p in Path('/proc').iterdir():
            if not p.name.isdigit():continue
            try:
                fields={line.split(':',1)[0]:line.split(':',1)[1].strip() for line in (p/'status').read_text().splitlines() if ':' in line}
                processes[int(p.name)]=(int(fields['PPid']),int(fields.get('VmRSS','0 kB').split()[0])*1024)
            except (FileNotFoundError,ProcessLookupError,PermissionError,KeyError):continue
        ids={os.getpid()}
        while True:
            found={pid for pid,(parent,_) in processes.items() if parent in ids}
            if found.issubset(ids):break
            ids|=found
        rss=sum(processes.get(pid,(0,0))[1] for pid in ids)
        mem={line.split(':')[0]:int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines()}
        free=shutil.disk_usage(self.output).free
        self.peak_run=max(self.peak_run,total);self.peak_rss=max(self.peak_rss,rss)
        reasons=[]
        for value,key in [(total,'run_bytes'),(bag,'bag_bytes'),(rss,'process_tree_rss_bytes')]:
            if value>=self.limits[key]:reasons.append(key+' budget exceeded')
        if free<self.limits['reserve_disk_bytes']:reasons.append('disk reserve reached')
        if mem['MemAvailable']<self.limits['minimum_available_memory_bytes']:reasons.append('available RAM reserve reached')
        return dict(wall_unix=time.time(),run_bytes=total,bag_bytes=bag,dataset_bytes=dataset,
            process_tree_rss_bytes=rss,available_memory_bytes=mem['MemAvailable'],free_disk_bytes=free,
            peak_process_tree_rss_bytes=self.peak_rss,peak_run_bytes=self.peak_run,limits=self.limits,
            reason='; '.join(reasons) or None)
