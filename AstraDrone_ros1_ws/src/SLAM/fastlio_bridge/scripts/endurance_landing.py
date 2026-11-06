#!/usr/bin/env python3
"""SITL runner landing phase; AUTO.LAND owns descent, no second setpoint writer."""
import threading
import time
import rospy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import SetMode, ParamGet, ParamSet


class EnduranceLanding:
    def __init__(self):
        self.lock=threading.Lock();self.state=None;self.extended=None
        self.state_wall=self.extended_wall=None
        self.last_disarmed=None;self.origin=None;self.origin_verified=False
        self.airborne_started=None;self.ground_stamp=None;self.disarmed_stamp=None
        self.descent_configuration=None
        self.subscribers=[rospy.Subscriber('/mavros/state',State,self.receive_state,queue_size=5),
            rospy.Subscriber('/mavros/extended_state',ExtendedState,self.receive_extended,queue_size=5)]

    def configure_slow_descent(self):
        """SITL demo setup before arming; FCU remains the only descent owner.

        PX4's land detector requires its vertical-motion threshold to be
        lower than the commanded landing speed. Read back every change;
        a partially configured vehicle must not start an autonomous mission.
        """
        with self.lock:
            if self.state is None or self.state.armed:
                raise RuntimeError('Slow-descent configuration requires disarmed telemetry')
        targets={'MPC_LAND_SPEED':.20,'MPC_LAND_CRWL':.12,'LNDMC_Z_VEL_MAX':.08}
        rospy.wait_for_service('/mavros/param/get',timeout=8.)
        rospy.wait_for_service('/mavros/param/set',timeout=8.)
        get=rospy.ServiceProxy('/mavros/param/get',ParamGet)
        set_value=rospy.ServiceProxy('/mavros/param/set',ParamSet)
        previous={};verified={}
        for name,value in targets.items():
            before=get(param_id=name)
            if not before.success:raise RuntimeError('Cannot read PX4 '+name)
            previous[name]=before.value.real
            changed=set_value(param_id=name,value=ParamValue(integer=0,real=value))
            after=get(param_id=name)
            if not changed.success or not after.success or abs(after.value.real-value)>1e-5:
                raise RuntimeError('PX4 slow-descent readback failed: '+name)
            verified[name]=after.value.real
        self.descent_configuration=dict(previous=previous,verified=verified,
            scope='SITL parameter changes, read back before arming; no forced disarm or second setpoint writer')
        return self.descent_configuration

    def receive_state(self,msg):
        now=rospy.Time.now().to_sec();stamp=msg.header.stamp.to_sec()
        if not stamp or not -.1<=now-stamp<=2.5:return
        with self.lock:
            self.state=msg;self.state_wall=time.monotonic()
            if not msg.armed:
                self.last_disarmed=stamp
                if self.origin is not None and self.airborne_started is not None:
                    self.disarmed_stamp=stamp
            elif self.origin is None:
                self.origin_verified=self.last_disarmed is not None
                self.origin=self.last_disarmed if self.origin_verified else stamp

    def receive_extended(self,msg):
        now=rospy.Time.now().to_sec();stamp=msg.header.stamp.to_sec()
        if not stamp or not -.1<=now-stamp<=2.5:return
        with self.lock:
            self.extended=msg;self.extended_wall=time.monotonic()
            if msg.landed_state==ExtendedState.LANDED_STATE_IN_AIR and self.airborne_started is None:
                self.airborne_started=stamp
            if self.airborne_started is not None and msg.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
                self.ground_stamp=stamp

    def land(self,total=600.,reserve=60.,sim_timeout=45.,wall_timeout=150.,unlimited=False):
        began_sim=rospy.Time.now().to_sec();began_wall=time.monotonic();last_request=-float('inf')
        accepted=False;landed=False;request_errors=0;last_error=None
        mode=rospy.ServiceProxy('/mavros/set_mode',SetMode)
        while time.monotonic()-began_wall<wall_timeout and rospy.Time.now().to_sec()-began_sim<sim_timeout:
            with self.lock:
                state,extended=self.state,self.extended
                fresh=(self.state_wall is not None and self.extended_wall is not None and
                    time.monotonic()-self.state_wall<=2.5 and time.monotonic()-self.extended_wall<=2.5)
                if fresh and state and extended and not state.armed and \
                        extended.landed_state==ExtendedState.LANDED_STATE_ON_GROUND:
                    landed=True;break
            if state and state.mode!='AUTO.LAND' and time.monotonic()-last_request>2.:
                last_request=time.monotonic()
                try:
                    rospy.wait_for_service('/mavros/set_mode',timeout=.5)
                    accepted=bool(mode(base_mode=0,custom_mode='AUTO.LAND').mode_sent) or accepted
                except (rospy.ROSException,rospy.ServiceException) as error:
                    # A single MAVROS service interruption must not escape
                    # the bounded landing loop and skip every later retry.
                    request_errors+=1;last_error=str(error)
            time.sleep(.1)
        with self.lock:
            elapsed=(self.disarmed_stamp-self.origin if landed and self.origin is not None and
                     self.disarmed_stamp is not None else None)
            airborne=(self.ground_stamp-self.airborne_started if self.ground_stamp is not None and
                      self.airborne_started is not None else None)
            result=dict(mode_request_accepted=accepted,landed_disarmed_confirmed=landed,
                descent_configuration=self.descent_configuration,
                mode_request_errors=request_errors,last_mode_request_error=last_error,
                budget_origin_verified=self.origin_verified,budget_origin_stamp=self.origin,
                armed_to_disarmed_upper_bound_sim_seconds=elapsed,airborne_sim_seconds=airborne,
                landing_sim_seconds=rospy.Time.now().to_sec()-began_sim,
                within_nominal_budget=None if unlimited else bool(elapsed is not None and elapsed<=total),
                reserve_preserved=None if unlimited else bool(elapsed is not None and elapsed<=total-reserve),
                nominal_seconds=None if unlimited else total,reserve_seconds=None if unlimited else reserve,
                time_limit_enabled=not unlimited,
                note='SITL telemetry; origin is last disarmed heartbeat before first arm, '
                     'conservatively including that heartbeat interval. Not measured battery endurance.')
        result['passed']=bool(landed and self.origin_verified and
            (unlimited or (result['within_nominal_budget'] and result['reserve_preserved'])))
        return result

    def close(self):
        for sub in self.subscribers:sub.unregister()

    def is_armed(self):
        with self.lock:
            return bool(self.state is not None and self.state.armed)

    def forced_landing_due(self,total=600.,reserve=60.,landing_allowance=40.):
        """Independent runner guard, including when a planner is waiting/stalled."""
        with self.lock:
            return bool(self.origin_verified and self.origin is not None and self.state and
                self.state.armed and rospy.Time.now().to_sec()-self.origin>=
                total-reserve-landing_allowance)
