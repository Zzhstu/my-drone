#!/usr/bin/env python3
"""Opt-in endurance policy. Time is simulated flight time, never Gazebo wall time."""
import math
import numpy as np


class FlightBudget:
    def __init__(self,total=600.,reserve=60.,speed=.35,factor=1.3,
                 landing=25.,home_dwell=5.,overhead=15.,review=0.):
        values=(total,reserve,speed,factor,landing,home_dwell,overhead,review)
        self.unlimited=total==0 and reserve==0
        if not all(math.isfinite(v) for v in values) or not (
                (self.unlimited or (total>0 and 0<=reserve<total and
                    reserve+landing+home_dwell+overhead+review<total)) and
                speed>0 and factor>=1 and landing>0 and home_dwell>=0 and overhead>=0 and review>=0):
            raise ValueError('invalid flight endurance policy')
        self.total,self.reserve,self.speed,self.factor=total,reserve,speed,factor
        self.landing,self.home_dwell,self.overhead=landing,home_dwell,overhead
        self.review=review
        self.origin=None;self.saw_disarmed=False;self.origin_verified=False
        self.last_disarmed_stamp=None
        self.last_valid_home=None
        self.home_unavailable_since=None

    def observe_arm(self,armed,stamp):
        if not math.isfinite(stamp) or stamp<=0:return
        if not armed:
            self.saw_disarmed=True
            self.last_disarmed_stamp=stamp
        elif self.origin is None:
            self.origin=self.last_disarmed_stamp if self.saw_disarmed else stamp
            self.origin_verified=self.saw_disarmed

    def elapsed(self,now):
        if not math.isfinite(now) or self.origin is None or not self.origin_verified or now<self.origin:
            return None
        return now-self.origin

    def travel_seconds(self,distance):
        return np.asarray(distance)*self.factor/self.speed

    @property
    def fixed_return_seconds(self):
        return self.landing+self.home_dwell+self.overhead+self.review

    def report(self,now,home_distance):
        elapsed=self.elapsed(now)
        eta=float(self.travel_seconds(home_distance))+self.fixed_return_seconds
        remaining=(self.total-self.reserve-elapsed
                   if elapsed is not None and not self.unlimited else None)
        # A live occupancy update can temporarily disconnect the home cell.
        # Hold in place while recomputing it; never use the cached path to
        # authorize a new flight. A persistent loss, stale cache, or insufficient
        # return margin retains the original fail-closed return decision.
        hold=False
        unavailable_seconds=0.
        if math.isfinite(eta):
            self.last_valid_home=(now,eta)
            self.home_unavailable_since=None
        else:
            if self.home_unavailable_since is None or now<self.home_unavailable_since:
                self.home_unavailable_since=now
            unavailable_seconds=now-self.home_unavailable_since
            if self.last_valid_home is not None and remaining is not None:
                stamp,last_eta=self.last_valid_home
                hold=bool(0<=now-stamp<=2. and unavailable_seconds<1.5 and
                          remaining>last_eta+1.5)
        return dict(enabled=True,time_limit_enabled=not self.unlimited,
            total_seconds=None if self.unlimited else self.total,reserve_seconds=self.reserve,
            arm_stamp=self.origin,origin_verified=self.origin_verified,elapsed_seconds=elapsed,
            usable_remaining_seconds=remaining,
            reserved_return_review_seconds=self.review,
            home_path_cost_m=float(home_distance) if math.isfinite(home_distance) else None,
            return_and_land_eta_seconds=eta if math.isfinite(eta) else None,
            hold_for_home_path=hold,home_path_unavailable_seconds=unavailable_seconds,
            return_now=bool(not self.unlimited and elapsed is not None and not hold and
                            (not math.isfinite(eta) or remaining<=eta)),
            nominal_deadline_exceeded=bool(not self.unlimited and elapsed is not None and elapsed>=self.total),
            scope='conservative time estimate from observed safe path; not a battery-energy model')

    def feasible(self,now,outbound,homeward,view_seconds=2.,spend_review=False):
        elapsed=self.elapsed(now)
        if elapsed is None:return np.zeros(np.broadcast(outbound,homeward).shape,dtype=bool)
        if self.unlimited:
            return np.isfinite(np.asarray(outbound)+np.asarray(homeward))
        fixed=self.fixed_return_seconds-(self.review if spend_review else 0.)
        need=self.travel_seconds(np.asarray(outbound)+np.asarray(homeward))+view_seconds+fixed
        return np.isfinite(need)&(need <= self.total-self.reserve-elapsed)
