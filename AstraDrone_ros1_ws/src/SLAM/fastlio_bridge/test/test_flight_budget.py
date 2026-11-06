import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import rospy
from mavros_msgs.msg import State,ExtendedState
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from flight_budget import FlightBudget
from endurance_landing import EnduranceLanding
from exploration_coverage import ObservedMapAcceptance


class BudgetTest(unittest.TestCase):
    def ready(self):
        budget=FlightBudget();budget.observe_arm(False,10.);budget.observe_arm(True,11.)
        return budget

    def test_origin_includes_last_disarmed_heartbeat(self):
        budget=self.ready()
        self.assertEqual(budget.origin,10.)
        self.assertEqual(budget.elapsed(20.),10.)
        self.assertIsNone(budget.elapsed(9.))

    def test_missing_disarmed_observation_cannot_authorize_views(self):
        budget=FlightBudget();budget.observe_arm(True,11.)
        self.assertFalse(budget.origin_verified)
        self.assertFalse(budget.feasible(20.,np.array([1.]),np.array([1.])).any())

    def test_long_safe_detour_forces_earlier_return(self):
        budget=self.ready()
        self.assertFalse(budget.report(450.,3.)['return_now'])
        self.assertTrue(budget.report(450.,20.)['return_now'])

    def test_blocked_home_is_not_treated_as_zero_cost(self):
        report=self.ready().report(50.,float('inf'))
        self.assertTrue(report['return_now']);self.assertIsNone(report['home_path_cost_m'])

    def test_single_missing_home_path_holds_then_recovers(self):
        budget=self.ready()
        self.assertFalse(budget.report(215.7,41.734)['return_now'])
        missing=budget.report(215.9,float('inf'))
        self.assertTrue(missing['hold_for_home_path'])
        self.assertFalse(missing['return_now'])
        self.assertIsNone(missing['return_and_land_eta_seconds'])
        # The cached route does not make any missing-path view affordable.
        self.assertFalse(budget.feasible(215.9,[1.],[float('inf')]).any())
        recovered=budget.report(216.1,41.934)
        self.assertFalse(recovered['hold_for_home_path'])
        self.assertFalse(recovered['return_now'])

    def test_persistent_home_path_loss_still_forces_return(self):
        budget=self.ready();budget.report(100.,20.)
        self.assertTrue(budget.report(100.1,float('inf'))['hold_for_home_path'])
        report=budget.report(101.6,float('inf'))
        self.assertFalse(report['hold_for_home_path']);self.assertTrue(report['return_now'])

    def test_missing_path_cannot_spend_return_margin_or_use_stale_cache(self):
        budget=self.ready();budget.report(500.,1.)
        report=budget.report(501.,float('inf'))
        self.assertTrue(report['return_now']);self.assertFalse(report['hold_for_home_path'])
        budget=self.ready();budget.report(100.,1.)
        self.assertTrue(budget.report(103.,float('inf'))['return_now'])
        self.assertTrue(budget.report(99.,float('inf'))['return_now'])

    def test_round_trip_and_landing_must_fit_not_only_outbound(self):
        budget=self.ready()
        mask=budget.feasible(450.,np.array([1.,2.]),np.array([1.,20.]),5.)
        np.testing.assert_array_equal(mask,[True,False])

    def test_unlimited_sim_flight_keeps_safe_path_gate_without_deadline(self):
        budget=FlightBudget(total=0.,reserve=0.)
        budget.observe_arm(False,10.);budget.observe_arm(True,11.)
        report=budget.report(100000.,3.)
        self.assertFalse(report['time_limit_enabled'])
        self.assertIsNone(report['total_seconds'])
        self.assertFalse(report['return_now'])
        np.testing.assert_array_equal(
            budget.feasible(100000.,np.array([1.,1.]),np.array([3.,float('inf')])),
            [True,False])

    def test_invalid_endurance_policy_rejected(self):
        for kwargs in ({'total':float('nan')},{'reserve':600.},{'speed':0.},{'factor':.5}):
            with self.subTest(kwargs=kwargs),self.assertRaises(ValueError):FlightBudget(**kwargs)

    def test_relaxed_acceptance_still_needs_fresh_observation_and_hold(self):
        acceptance=ObservedMapAcceptance(.85,45.,5.,150.)
        surface=dict(observed_surface_quality_ratio=.86,observed_surface_voxels=1500)
        for t in range(156):
            result=acceptance.update(t,float(t),2000,.2,6,float(t),surface)
            if t<155:self.assertFalse(result)
        self.assertTrue(result)
        strict=ObservedMapAcceptance(.90,45.,5.,150.)
        for t in range(160):self.assertFalse(strict.update(t,float(t),2000,.2,6,float(t),surface))

    def test_landing_needs_airborne_origin_ground_and_disarm(self):
        clock=[100.]
        with patch('endurance_landing.rospy.Subscriber'),\
             patch('endurance_landing.rospy.Time.now',side_effect=lambda:rospy.Time.from_sec(clock[0])),\
             patch('endurance_landing.time.monotonic',return_value=1.),\
             patch('endurance_landing.rospy.wait_for_service'),\
             patch('endurance_landing.rospy.ServiceProxy'):
            observer=EnduranceLanding()
            def state(t,armed):
                clock[0]=t;m=State(connected=True,armed=armed);m.header.stamp=rospy.Time.from_sec(t);observer.receive_state(m)
            def extended(t,value):
                clock[0]=t;m=ExtendedState(landed_state=value);m.header.stamp=rospy.Time.from_sec(t);observer.receive_extended(m)
            state(100.,False);state(101.,True)
            extended(105.,ExtendedState.LANDED_STATE_IN_AIR)
            extended(200.,ExtendedState.LANDED_STATE_ON_GROUND);state(201.,False)
            result=observer.land()
            self.assertTrue(result['passed'])
            self.assertEqual(result['armed_to_disarmed_upper_bound_sim_seconds'],101.)
            unlimited=observer.land(unlimited=True)
            self.assertTrue(unlimited['passed'])
            self.assertFalse(unlimited['time_limit_enabled'])
            self.assertIsNone(unlimited['nominal_seconds'])

    def test_runner_budget_guard_does_not_depend_on_planner_updates(self):
        clock=[100.]
        with patch('endurance_landing.rospy.Subscriber'),\
             patch('endurance_landing.rospy.Time.now',side_effect=lambda:rospy.Time.from_sec(clock[0])):
            observer=EnduranceLanding()
            self.assertFalse(observer.is_armed())
            observer.origin=100.;observer.origin_verified=True;observer.state=State(armed=True)
            self.assertTrue(observer.is_armed())
            clock[0]=599.;self.assertFalse(observer.forced_landing_due())
            clock[0]=600.;self.assertTrue(observer.forced_landing_due())
            observer.origin_verified=False;self.assertFalse(observer.forced_landing_due())

    def test_transient_landing_service_failure_is_retried(self):
        wall=[0.];sim=[100.]
        def sleep(duration):
            wall[0]+=duration;sim[0]+=duration
        with patch('endurance_landing.rospy.Subscriber'),\
             patch('endurance_landing.rospy.Time.now',side_effect=lambda:rospy.Time.from_sec(sim[0])),\
             patch('endurance_landing.time.monotonic',side_effect=lambda:wall[0]),\
             patch('endurance_landing.time.sleep',side_effect=sleep),\
             patch('endurance_landing.rospy.wait_for_service'),\
             patch('endurance_landing.rospy.ServiceProxy') as proxy:
            observer=EnduranceLanding()
            observer.origin=90.;observer.origin_verified=True
            observer.state=State(armed=True,mode='OFFBOARD')
            observer.extended=ExtendedState(landed_state=ExtendedState.LANDED_STATE_IN_AIR)
            observer.state_wall=observer.extended_wall=0.
            def retry(**kwargs):
                if proxy.return_value.call_count==1:
                    raise rospy.ServiceException('temporary service reset')
                observer.state.armed=False
                observer.extended.landed_state=ExtendedState.LANDED_STATE_ON_GROUND
                observer.state_wall=observer.extended_wall=wall[0]
                observer.disarmed_stamp=sim[0];observer.airborne_started=91.;observer.ground_stamp=sim[0]
                return type('Reply',(),{'mode_sent':True})()
            proxy.return_value.side_effect=retry
            result=observer.land(sim_timeout=10.,wall_timeout=10.)
            self.assertEqual(proxy.return_value.call_count,2)
            self.assertEqual(result['mode_request_errors'],1)
            self.assertTrue(result['landed_disarmed_confirmed'])
            self.assertTrue(result['passed'])


if __name__=='__main__':unittest.main()
