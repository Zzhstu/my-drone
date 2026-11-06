#!/usr/bin/env python3
"""Sensor-grid-only, bounded continuation of a partially observed corridor.

Never invents free cells, targets unknown cells, or reads a simulator model.
This schedules a chain of existing reachable frontier views before retreating
to a distant large frontier. Flight feasibility remains an independent gate.
"""
from collections import deque
import math
import numpy as np


class CorridorProgress:
    def __init__(self, maximum_seconds=100., step_m=4.5, maximum_width_m=4.5,
                 entry_enabled=False):
        self.maximum_seconds=float(maximum_seconds)
        self.step_m=float(step_m)
        self.maximum_width_m=float(maximum_width_m)
        self.entry_enabled=bool(entry_enabled)
        self.reset()

    def reset(self):
        self.positions=deque(maxlen=24)
        self.started=None
        self.cooldown=-float('inf')
        self.leg_started=None
        self.scan_started=None;self.scan_position=None
        self.entry_axis=None;self.entry_started=None;self.entry_origin=None
        self.report=dict(active=False,reason='not_started')

    def observe(self, position):
        p=np.asarray(position,dtype=float)
        if p.shape!=(2,) or not np.isfinite(p).all():return
        if self.positions:
            distance=float(np.linalg.norm(p-self.positions[-1]))
            if distance<.25:return
            if distance>1.5:
                self.reset()
        self.positions.append(p.copy())

    def heading(self):
        if self.entry_axis is not None:return self.entry_axis.copy()
        if len(self.positions)<2:return None
        current=self.positions[-1]
        for previous in reversed(list(self.positions)[:-1]):
            delta=current-previous;length=float(np.linalg.norm(delta))
            if length>=1.:
                return delta/length
        return None

    def structure(self,grid,xy,direction,wall_mask=None):
        """Opposing measured lateral boundaries, not unknown-space guesses."""
        lateral=np.array([-direction[1],direction[0]])
        samples=np.arange(.5,self.maximum_width_m+.05,.1)
        supported=[]
        for along in (-.4,0.,.8):
            origin=xy+along*direction;distances=[]
            for sign in (-1,1):
                found=None
                for d in samples:
                    cell=grid.cell(origin+sign*d*lateral)
                    if not grid.inside(cell):break
                    if grid.occupied[cell]:found=float(d);break
                distances.append(found)
            if all(v is not None for v in distances) and sum(distances)<=self.maximum_width_m:
                supported.append(distances)
        if not supported:return None
        # Missing shelf ends/doorways can interrupt one of the three probes;
        # this is only an intention cue. Navigation still uses original safe.
        sides=min(supported,key=lambda row:abs(row[0]-row[1]))
        long_boundary=False;maximum_span=0.
        # Demand a continuous observed boundary: separate collinear shelves
        # must not be stitched into a wall. The first occupied body-height
        # cell also needs measured near-ceiling support when available.
        for sign in (-1,1):
            run=[]
            for along in np.arange(-3.,3.01,.2):
                origin=xy+along*direction;hit=None
                for d in samples:
                    cell=grid.cell(origin+sign*d*lateral)
                    if not grid.inside(cell):break
                    if grid.occupied[cell]:
                        if wall_mask is None or wall_mask[cell]:hit=float(d)
                        break
                if hit is None:
                    run=[];continue
                run.append((float(along),hit))
                while run and max(r[1] for r in run)-min(r[1] for r in run)>.8:
                    run.pop(0)
                maximum_span=max(maximum_span,run[-1][0]-run[0][0])
                if maximum_span>=4.19:
                    long_boundary=True;break
            if long_boundary:break
        return dict(width_m=sum(sides),center_offset_m=abs(sides[0]-sides[1]),
                    supported_cross_sections=len(supported),long_parallel_boundary=long_boundary,
                    continuous_wall_span_m=maximum_span)

    def entry_structure(self,grid,position,xy,heading,wall_mask):
        """Measured high-wall turn cue; it never authorizes unknown motion.

        Evaluate the aisle axis independently of the diagonal approach vector.
        A low shelf, or a wall hidden behind its nearest occupied cell, cannot
        supply the high-wall cue. A bounded entrance task must subsequently
        establish the original 4.2m straight-corridor evidence.
        """
        if not self.entry_enabled or wall_mask is None:return None
        delta=xy-position;length=float(np.linalg.norm(delta));choices=[]
        for angle in (-math.pi/2,-math.pi/4,0.,math.pi/4,math.pi/2):
            c,s=math.cos(angle),math.sin(angle)
            axis=np.array([c*heading[0]-s*heading[1],s*heading[0]+c*heading[1]])
            forward=float(delta@axis)
            if forward<max(.6,.3*length):continue
            cue=self.structure(grid,xy,axis,wall_mask)
            if (cue is None or cue['continuous_wall_span_m']<2.39 or
                    cue['supported_cross_sections']<2 or cue['center_offset_m']>.8):continue
            choices.append((cue['continuous_wall_span_m']+.1*forward,axis,cue))
        if not choices:return None
        _,axis,cue=max(choices,key=lambda row:row[0])
        return axis,cue

    def local_frontiers(self,grid,position,distances,mask,memory):
        """Bounded local views before the normal 1.8m goal suppression.

        A short step may be the only observed safe step into a narrow passage.
        Its actual-passage memory still excludes already flown goals.
        """
        heading=self.heading()
        if heading is None:return np.empty((0,2),int)
        cells=np.argwhere(mask & grid.safe & np.isfinite(distances))
        if not len(cells):return cells
        xy=np.column_stack((grid.x0+(cells[:,1]+.5)*grid.resolution,
                            grid.y0+(cells[:,0]+.5)*grid.resolution))
        delta=xy-np.asarray(position);length=np.linalg.norm(delta,axis=1)
        alignment=delta@heading/np.maximum(length,1e-6)
        ok=(length>=1.)&(length<=self.step_m)&(alignment>=-.15)
        ids=np.flatnonzero(ok);ids=ids[np.argsort(-(length[ids]*(.7+.3*np.maximum(0,alignment[ids]))))]
        result=[];seen=set()
        for i in ids:
            tile=tuple(np.floor(xy[i]/.6).astype(int))
            if tile in seen or memory.weight(xy[i])>0:continue
            result.append(cells[i]);seen.add(tile)
            if len(result)>=32:break
        return np.asarray(result,dtype=int).reshape(-1,2)

    def choose(self,grid,position,distances,parent,home_distances,cells,gains,
               memory,budget,now,current=None,indoor_weights=None,wall_mask=None):
        self.report=dict(active=self.started is not None,reason='no_local_continuation',
                         considered=0,rejected={},selected=None,candidate_checks=[],
                         stage='entrance' if self.entry_axis is not None else 'straight')
        def reject(name):
            d=self.report['rejected'];d[name]=d.get(name,0)+1
        if now<self.cooldown:
            self.report['reason']='cooldown';return None
        if self.entry_started is not None and now-self.entry_started>=30.:
            self.entry_axis=None;self.entry_started=None;self.started=None
            self.cooldown=now+15.;self.report.update(active=False,reason='entrance_time_limit')
            return None
        if self.started is not None and now-self.started>=self.maximum_seconds:
            self.started=None;self.entry_axis=None;self.entry_started=None;self.cooldown=now+30.
            self.report.update(active=False,reason='task_time_limit');return None
        heading=self.heading()
        if current is not None and current.corridor_task:
            cell=grid.cell(current.xy)
            if (grid.inside(cell) and grid.safe[cell] and np.isfinite(distances[cell]) and
                    budget.feasible(now,distances[cell],home_distances[cell],1.) and
                    self.leg_started is not None and now-self.leg_started<25.):
                self.report.update(active=True,reason='committed_safe_leg',selected=list(current.xy))
                return cell
            self.started=None;self.entry_axis=None;self.entry_started=None
            self.report['reason']='leg_invalidated';return None
        if heading is None or parent is None:
            self.report['reason']='insufficient_actual_motion';return None
        if current is not None:
            vector=np.asarray(current.xy)-position
            retreat=(np.linalg.norm(vector)>6. and
                     (current.repeated_path_m>=1. or float(vector@heading)<0.))
            if current.kind!='frontier' or not retreat:
                self.report['reason']='finish_current_view';return None
        rows=[];position=np.asarray(position)
        for cell,gain in zip(cells,gains):
            cell=tuple(map(int,cell));xy=np.asarray(grid.xy(cell));delta=xy-position
            length=float(np.linalg.norm(delta))
            if not 1.<=length<=self.step_m or gain<.5:continue
            direction=delta/length;alignment=float(direction@heading)
            if alignment<-.15:continue
            if not grid.safe[cell] or not np.isfinite(distances[cell]):continue
            if memory.weight(xy)>0:continue
            if indoor_weights is not None and indoor_weights[cell]<.5:continue
            if not budget.feasible(now,distances[cell],home_distances[cell],1.):continue
            rows.append((cell,xy,direction,alignment,length,float(gain)))
        # Bound work independently of map size. Favor physical forward progress,
        # not dozens of duplicate high-area rays into the same distant pocket.
        rows.sort(key=lambda r:-(r[4]*(.7+.3*max(0.,r[3]))+.05*min(4.,r[5])))
        choices=[];start=grid.cell(position)
        entry_rows=[]
        for cell,xy,direction,alignment,length,gain in rows[:32]:
            self.report['considered']+=1
            path=grid.path(parent,start,cell)
            if not path:reject('no_safe_path');continue
            p=np.asarray([grid.xy(c) for c in path])
            actual_length=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
            if actual_length>1.6*length+1.:reject('path_detour');continue
            forward=next((q-position for q in p if np.linalg.norm(q-position)>=.8),xy-position)
            if float(forward@heading)<-.15*np.linalg.norm(forward):reject('initial_backtrack');continue
            structure=self.structure(grid,xy,direction,wall_mask)
            detail=dict(xy=xy.tolist(),path_length_m=actual_length,approach_direction=direction.tolist(),
                        straight_structure=structure)
            self.report['candidate_checks'].append(detail)
            if structure is None:
                reject('no_opposing_structure');entry_rows.append((cell,xy,alignment,length,gain,actual_length,detail));continue
            if not structure['long_parallel_boundary'] or structure['center_offset_m']>.8:
                reject('short_or_nonparallel_structure')
                detail['reason']=('short_measured_high_wall' if not structure['long_parallel_boundary'] else 'off_center')
                entry_rows.append((cell,xy,alignment,length,gain,actual_length,detail));continue
            score=(length*(.7+.3*max(0.,alignment))+.05*min(4.,gain))/(1.+.3*structure['center_offset_m'])
            choices.append((score,cell,xy,structure,actual_length,gain,None))
        # Only bounded local candidates already passing safe/path/budget checks.
        if not choices:
            for cell,xy,alignment,length,gain,actual_length,detail in entry_rows[:8]:
                cue=self.entry_structure(grid,position,xy,heading,wall_mask)
                if cue is None:continue
                axis,structure=cue;detail.update(entry_structure=structure,entry_axis=axis.tolist())
                score=(float((xy-position)@axis)+.05*min(4.,gain))/(1.+.3*structure['center_offset_m'])
                choices.append((score,cell,xy,structure,actual_length,gain,axis))
        if not choices:
            # A brief fresh scan at a safe turning point can fill the corridor
            # entrance's ray evidence. It never commands motion into unknown.
            structural_hint=False
            for angle in (-math.pi/2,-math.pi/4,0.,math.pi/4,math.pi/2):
                c,s=math.cos(angle),math.sin(angle)
                direction=np.array([c*heading[0]-s*heading[1],s*heading[0]+c*heading[1]])
                hint=self.structure(grid,position+2.*direction,direction,wall_mask)
                if hint is not None and hint['long_parallel_boundary']:
                    structural_hint=True;break
            if structural_hint and budget.feasible(now,0.,home_distances[start],2.):
                if self.scan_position is None or np.linalg.norm(position-self.scan_position)>2.5:
                    self.scan_position=position.copy();self.scan_started=now
                if now-self.scan_started<2.:
                    self.report.update(reason='scan_before_turn',hold=True,hold_until=self.scan_started+2.)
                    return None
            if self.started is not None:self.cooldown=now+10.
            self.started=None;self.entry_axis=None;self.entry_started=None;self.report['active']=False
            return None
        _,cell,xy,structure,length,gain,axis=max(choices,key=lambda row:row[0])
        if axis is not None:
            if self.entry_started is None:self.entry_started=now;self.entry_origin=position.copy()
            self.entry_axis=axis.copy()
        else:self.entry_axis=None;self.entry_started=None
        if self.started is None:self.started=now
        self.leg_started=now
        self.report.update(active=True,reason='enter_observed_turn' if axis is not None else 'advance_observed_corridor',
            stage='entrance' if axis is not None else 'straight',selected=xy.tolist(),
            entry_axis=axis.tolist() if axis is not None else None,
            elapsed_seconds=now-self.started,path_length_m=length,unknown_gain_m2=gain,**structure)
        return cell
