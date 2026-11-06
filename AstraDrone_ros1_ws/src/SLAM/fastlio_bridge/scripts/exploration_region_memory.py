#!/usr/bin/env python3
"""Bounded sensor-derived task feedback. Never edits occupancy or flight clocks."""
from dataclasses import dataclass, field
import math
import numpy as np


@dataclass(frozen=True)
class BenefitClaim:
    key: tuple
    target: tuple
    mass: float
    normal: tuple = ()
    support: frozenset = field(default_factory=frozenset)
    critical_geometry: bool = True
    observed_support_voxels: int = 0
    missing_area_m2: float = 0.

    @property
    def parent(self):return tuple(self.key[1:3])

    @property
    def critical(self):return self.critical_geometry and self.key[0] in ('floor','boundary')

    def summary(self):return dict(key=list(self.key),target=list(self.target),mass=self.mass,
                                 normal=list(self.normal),support_cells=len(self.support),critical_geometry=self.critical,
                                 observed_support_voxels=self.observed_support_voxels,missing_area_m2=self.missing_area_m2)


def surface_claim(kind,target,normal=(),mass=1.,size=2.4,height_origin=0.,critical=True,support_voxels=0,missing_area_m2=0.):
    target=np.asarray(target,dtype=float);normal=tuple(map(float,normal))
    direction=(int(round(math.atan2(normal[1],normal[0])/(math.pi/4)))%8 if len(normal)>=2 else -1)
    band=int(math.floor((target[2]-height_origin)/.4))
    key=(kind,int(math.floor(target[0]/size)),int(math.floor(target[1]/size)),direction,band)
    if kind=='floor':key+=tuple(map(int,np.floor(target[:2]/.8)))
    return BenefitClaim(key,tuple(target),float(mass),normal,critical_geometry=critical,observed_support_voxels=int(support_voxels),missing_area_m2=float(missing_area_m2))


def floor_deficit_components(mask,resolution):
    from collections import deque
    mask=np.asarray(mask,dtype=bool);labels=np.zeros(mask.shape,dtype=np.int32)
    padded=np.pad(mask,1);core=mask.copy();h,w=mask.shape
    for dy in (-1,0,1):
        for dx in (-1,0,1):core &= padded[1+dy:1+dy+h,1+dx:1+dx+w]
    regions={};label=0
    for y,x in np.argwhere(mask):
        if labels[y,x]:continue
        label+=1;queue=deque([(int(y),int(x))]);labels[y,x]=label;count=interior=0
        while queue:
            yy,xx=queue.popleft();count+=1;interior+=int(core[yy,xx])
            for y2,x2 in ((yy-1,xx),(yy+1,xx),(yy,xx-1),(yy,xx+1)):
                if 0<=y2<h and 0<=x2<w and mask[y2,x2] and not labels[y2,x2]:
                    labels[y2,x2]=label;queue.append((y2,x2))
        regions[label]=dict(area_m2=count*resolution**2,core_cells=interior)
    return labels,regions


class MeasuredSurfaceSupport:
    """Measured face support only; never reads reference geometry or marks free.

    A coarse observed face may still have holes. This only distinguishes a
    missing-face protection exception from ordinary refinement after low gain.
    """
    def __init__(self,points,plane,size=2.4):
        self.plane=np.asarray(plane);self.size=size;self.groups={};self.cache={}
        points=np.asarray(points,dtype=float).reshape(-1,3)
        if not len(points):return
        bins=np.floor(points[:,:2]/size).astype(int)
        unique,inverse=np.unique(bins,axis=0,return_inverse=True)
        order=np.argsort(inverse,kind='stable');counts=np.bincount(inverse)
        for key,ids in zip(unique,np.split(order,np.cumsum(counts)[:-1])):
            self.groups[tuple(key)]=points[ids]

    def assess(self,kind,target,normal=()):
        target=np.asarray(target);parent=tuple(np.floor(target[:2]/self.size).astype(int))
        normal=tuple(map(float,normal))
        key=(kind,parent,normal,tuple(np.floor(target[:2]/.2).astype(int)))
        if key in self.cache:return self.cache[key]
        groups=[self.groups[k] for dx in (-1,0,1) for dy in (-1,0,1)
                for k in [(parent[0]+dx,parent[1]+dy)] if k in self.groups]
        if not groups:return False,0
        points=np.concatenate(groups);delta=points-target
        heights=points[:,2]-(points[:,:2]@self.plane[:2]+self.plane[2])
        if kind=='floor':
            mask=(np.linalg.norm(delta[:,:2],axis=1)<=.55)&(np.abs(heights)<=.10)
            bins=np.unique(np.floor(points[mask,:2]/.2).astype(int),axis=0) if mask.any() else np.empty((0,2),int)
            result=(len(bins)>=8,len(bins))
        elif kind=='boundary' and len(normal)>=2:
            n=np.asarray(normal[:2]);t=np.array([-n[1],n[0]])
            tangent=delta[:,:2]@t
            vertical=(heights>=.3)&(heights<=1.9)&(np.abs(tangent)<=1.2)
            nearby=vertical&(np.abs(delta[:,:2]@n)<=.8)
            extent=float(np.ptp(tangent[nearby])) if nearby.any() else 0.
            required_width=min(.8,max(.35,.65*extent))
            mask=vertical&(np.abs(delta[:,:2]@n)<=.24)
            selected=points[mask];count=len(np.unique(np.floor(selected/.4).astype(int),axis=0)) if len(selected) else 0
            seen=bool(count>=8 and np.ptp(heights[mask])>=.6 and np.ptp(tangent[mask])>=required_width) if count else False
            result=(seen,count)
        else:result=(False,0)
        self.cache[key]=result;return result


class RegionMemory:
    def __init__(self,size=2.4,cooldown=60.,low_limit=2,capacity=1000):
        self.size=float(size);self.cooldown=float(cooldown);self.low_limit=int(low_limit);self.capacity=int(capacity)
        self.records={};self.parents={};self.capacity_limited=False;self.outcomes=0;self.scene_states={};self.scene_reopens=0

    def reset_coordinates(self):
        self.records.clear();self.parents.clear();self.scene_states.clear();self.capacity_limited=False

    def entry(self,claim):
        if claim.key not in self.records:
            if len(self.records)>=self.capacity:self.capacity_limited=True;return None
            self.records[claim.key]=dict(low=0,attempts=0,scale=1.,until=None,signature=claim.support,
                sectors=set(),last_outcome=None,state='unobserved',last_view=None,
                target=claim.target,normal=claim.normal,critical_geometry=claim.critical,
                support_at_failure=claim.observed_support_voxels,last_considered=None)
        return self.records[claim.key]

    @staticmethod
    def sector(claim,view):
        delta=np.asarray(view)-np.asarray(claim.target[:2]);return int(math.floor((math.atan2(delta[1],delta[0])+math.pi)/(math.pi/4)))%8

    def weight(self,claim,now,view):
        record=self.entry(claim)
        if record is None:return 1.,'capacity_unresolved'
        record.update(target=claim.target,normal=claim.normal,critical_geometry=claim.critical,last_considered=now)
        if claim.critical and record['until'] is not None and claim.observed_support_voxels>=record['support_at_failure']+8:
            record.update(low=0,attempts=0,scale=1.,until=None,state='new_evidence',sectors=set())
        novel=(len(claim.support-record['signature'])*claim.mass/max(1,len(claim.support))>=.5 if claim.key[0]=='space' else False)
        if novel:
            record.update(low=0,scale=1.,until=None,state='new_evidence',signature=claim.support)
        # A genuinely new view can test an unresolved critical face, bounded
        # to three unproductive attempts. Opposite faces have independent keys.
        new_view=(claim.critical and self.sector(claim,view) not in record['sectors'] and
            (record['last_view'] is None or math.dist(view,record['last_view'])>=.75) and record['attempts']<3)
        if record['until'] is not None:
            if not new_view:return 0.,('cooling' if now<record['until'] else 'await_changed_evidence')
            return max(.25,record['scale']),'new_critical_view'
        parent=self.parents.get(claim.parent,{})
        shared=1. if claim.critical or record['state']=='new_evidence' else parent.get('scale',1.)
        return max(.1,record['scale'])*shared,record['state']

    def assess(self,claims,now,view):
        if not claims:return 1.,[]
        total=sum(max(0.,c.mass) for c in claims)
        values=[(c,*self.weight(c,now,view)) for c in claims]
        scale=sum(max(0.,c.mass)*w for c,w,_ in values)/max(1e-9,total)
        return scale,[dict(key=list(c.key),weight=w,state=state) for c,w,state in values]

    def capture(self,claims,grid,surface):
        """Actual current static-map keys and measured free cells near requests.

        Reuses SurfaceEvidence's current-key cache. No raw-scan density reward.
        Normal/height slabs keep a nearby opposite face from certifying a face.
        """
        keys=np.asarray(getattr(surface,'current_keys',np.empty((0,3))),dtype=int).reshape(-1,3)
        qualified=np.asarray(getattr(surface,'current_qualified_keys',np.empty((0,3))),dtype=int).reshape(-1,3)
        def choose(array,claim):
            if not len(array):return frozenset()
            xy=(array[:,:2]+.5)*surface.voxel
            if claim.key[0] in ('floor','boundary','measured'):
                target=np.asarray(claim.target);xyz=(array+.5)*surface.voxel;delta=xyz-target
                mask=(np.linalg.norm(delta[:,:2],axis=1)<=1.2)&(np.abs(delta[:,2])<=.45)
                if len(claim.normal)>=2:
                    mask &= np.abs(delta[:,:2]@np.asarray(claim.normal[:2]))<=.24
            else:
                mask=np.all(np.floor(xy/self.size).astype(int)==claim.parent,axis=1)
            return frozenset(map(tuple,array[mask]))
        points=np.asarray(getattr(surface,'current_points',np.empty((0,3))),dtype=float).reshape(-1,3)
        result={}
        for claim in claims:
            ix,iy=claim.parent;x0,y0=ix*self.size,iy*self.size
            lo=grid.cell((x0+1e-6,y0+1e-6));hi=grid.cell((x0+self.size-1e-6,y0+self.size-1e-6))
            yl,yh=max(0,lo[0]),min(grid.shape[0],hi[0]+1);xl,xh=max(0,lo[1]),min(grid.shape[1],hi[1]+1)
            cells=np.argwhere(grid.known[yl:yh,xl:xh]&~grid.occupied[yl:yh,xl:xh]) if yh>yl and xh>xl else np.empty((0,2),int)
            if len(cells):
                cells+=np.array([yl,xl]);xy=np.c_[grid.x0+(cells[:,1]+.5)*grid.resolution,grid.y0+(cells[:,0]+.5)*grid.resolution]
                free=frozenset(map(tuple,np.floor(xy/grid.resolution).astype(int)))
            else:free=frozenset()
            target=frozenset()
            if claim.key[0] in ('floor','boundary') and len(points):
                delta=points-np.asarray(claim.target)
                mask=(np.linalg.norm(delta[:,:2],axis=1)<=.35)&(np.abs(delta[:,2])<=.12)
                if len(claim.normal)>=2:mask &= np.abs(delta[:,:2]@np.asarray(claim.normal[:2]))<=.12
                target=frozenset(map(tuple,np.floor(points[mask]/.15).astype(int)))
            result[claim.key]=dict(free=free,surface=choose(keys,claim),qualified=choose(qualified,claim),target=target,cell_area=grid.resolution**2)
        return result

    @staticmethod
    def delta(before,after):
        return {key:len(after.get(key,frozenset())-before.get(key,frozenset())) for key in ('free','surface','qualified','target')}

    def finish(self,claims,before,after,now,view,valid=True):
        details=[];parents={}
        for claim in claims:
            delta=self.delta(before.get(claim.key,{}),after.get(claim.key,{}));record=self.entry(claim)
            # Distinct physical measures stay separate. Eight qualified keys
            # is an independent initial threshold, not eight new surface keys.
            useful=((not claim.critical and delta['free']*after.get(claim.key,{}).get('cell_area',.04)>=.5) or delta['surface']>=8 or delta['qualified']>=8 or (claim.critical and not before.get(claim.key,{}).get('target') and bool(after.get(claim.key,{}).get('target'))))
            if valid and record is not None:
                record['attempts']+=1;record['sectors'].add(self.sector(claim,view));record['last_view']=tuple(view)
                record['low']=0 if useful else record['low']+1
                record['scale']=min(1.,record['scale']+.25) if useful else max(.1,.5*record['scale'])
                record['last_outcome']=dict(stamp=now,**delta)
                record['state']='improved' if useful else 'low_yield_unresolved'
                if useful:record['until']=None
                elif record['low']>=self.low_limit:
                    record['until']=now+self.cooldown;record['signature']=claim.support
                    record['support_at_failure']=claim.observed_support_voxels
                parents[claim.parent]=parents.get(claim.parent,False) or useful
            details.append(dict(key=list(claim.key),valid=valid,useful=useful,**delta))
        if valid:
            self.outcomes+=1
            for parent,useful in parents.items():
                row=self.parents.setdefault(parent,dict(scale=1.,low=0))
                row['low']=0 if useful else row['low']+1
                row['scale']=min(1.,row['scale']+.25) if useful else max(.25,row['scale']*.5)
        return details

    def notify_scene(self,records,now,ignore_center=None,ignore_radius=0.):
        """Reopen on actual ghost-state transitions, never on a timer/receipt."""
        changed=set()
        for row in records:
            position=row.get('position',[])
            if len(position)!=3 or not np.isfinite(position).all():continue
            if ignore_center is not None and math.dist(position[:2],ignore_center)<=ignore_radius:continue
            raw_key=row.get('key')
            key=tuple(raw_key) if isinstance(raw_key,(list,tuple)) and len(raw_key) else tuple(np.floor(np.asarray(position)/.15).astype(int))
            signature=(row.get('state'),row.get('suspected_episodes'),row.get('cleared_episodes'),row.get('occupied_now'))
            old=self.scene_states.get(key)
            if old is not None and old!=signature:
                changed.add(tuple(math.floor(v/self.size) for v in position[:2]))
            if key in self.scene_states or len(self.scene_states)<self.capacity:self.scene_states[key]=signature
        for parent in changed:
            for key,record in self.records.items():
                if tuple(key[1:3])==parent and record['until'] is not None:
                    record.update(low=0,attempts=0,scale=max(.5,record['scale']),until=None,
                                  sectors=set(),state='scene_changed')
                    self.scene_reopens+=1
            if parent in self.parents:self.parents[parent].update(low=0,scale=max(.5,self.parents[parent]['scale']))
        return len(changed)

    def export(self,now,active=None):
        rows=[]
        for key,record in sorted(self.records.items()):
            rows.append(dict(key=list(key),target=list(record['target']),normal=list(record['normal']),
                critical_geometry=record['critical_geometry'],state=record['state'],
                attempts=record['attempts'],low_yield_count=record['low'],utility_scale=record['scale'],
                cooldown_until=record['until'],last_considered=record['last_considered'],
                last_outcome=record['last_outcome'],view_sectors=sorted(record['sectors'])))
        return dict(stamp=now,scope='Registered task history, not a complete inventory of the unknown world. Improved/cooling does not certify completion; unexecuted or unsuccessful requests remain explicit.',
            memory=self.report(),active_interrupted_task=active,requests=rows)

    def report(self):
        return dict(records=len(self.records),regions=len(self.parents),outcomes=self.outcomes,
            cooling=sum(v['until'] is not None for v in self.records.values()),capacity_limited=self.capacity_limited,scene_reopens=self.scene_reopens,
            scope='Observed-task feedback; cooling is unresolved, not map completion. No occupancy edits.')
