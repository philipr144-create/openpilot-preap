"""Offline V4 route-stop, brake handoff, projection, and acceleration regression tests."""
import copy
import importlib
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch

from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.resume_status import ResumeStatus
from opendbc.car.tesla.preap.pedal_feedback import PedalFeedback
from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_PROFILES
from openpilot.selfdrive.controls.lib.map_stop import MapStop, stop_candidate, read_snapshot
from openpilot.selfdrive.modeld.navigation_desire import reliable_position, read_navigation
from phone_navigation.route_tracking import Route
from phone_navigation.desire_bridge import DesireBridge


def nav(now=10., distance=30.):
  return dict(version=4, enabled=True, route_active=True, route_state='active', route_id='route',
              maneuver_id='1', maneuver=dict(type='turn', modifier='left', distance_m=distance),
              received_mono=now, expires_mono=now+2,
              position_quality=dict(projection_valid=True, path_ambiguous=False,
                                    heading_error_deg=2., match_error_m=1., gps_accuracy_m=3., gps_age_s=.1),
              mapped_stop_sign=dict(id='sign', distance_m=distance))


def status(now=10., **kw):
  return dict(dict(session='session', sequence=0, cruise=True, paused=False, brake=False,
                   healthy=True, enabled=True, received_mono=now, expires_mono=now+.5), **kw)


class Stops(unittest.TestCase):
  def setUp(self):
    self.fsm = MapStop()

  def step(self, now=10., distance=30., speed=5., resume=None, **kw):
    options=dict(enabled=True, active=True, inputs_valid=True, v_ego=speed, gas=False, now=now)
    options.update(kw)
    return self.fsm.update(nav(now, distance), status(now) if resume is None else resume, **options)

  def hold(self):
    self.step()
    self.step(10.1, 7., 0., status(10.1, paused=True, brake=True), active=False)
    self.assertEqual(self.fsm.phase, 'held')

  def test_approach_stop_brake_hold_fresh_resume_consumes(self):
    cap, stop = self.step()
    self.assertGreater(cap, 0)
    self.assertFalse(stop)
    self.assertEqual(self.step(10.05, 7., 2.), (0., True))
    self.assertEqual(self.step(10.1, 7., 0., status(10.1, paused=True, brake=True), active=False), (None, False))
    self.assertEqual(self.fsm.phase, 'held')
    self.assertEqual(self.step(10.2, 7., 0., status(10.2, paused=True)), (0., True))
    self.assertEqual(self.step(10.3, 7., 0., status(10.3, sequence=1)), (None, False))
    self.assertIn('sign', self.fsm.consumed)
    self.assertEqual(self.step(10.4, 15., 0.), (None, False))

  def test_no_resume_on_old_or_new_session_ack(self):
    self.hold()
    self.assertEqual(self.step(10.2, 7., 0.), (0., True))
    self.step(10.3, 7., 0., status(10.3, sequence=1, session='restarted'))
    self.assertNotIn('sign', self.fsm.consumed)

  def test_hold_survives_map_loss_and_pedal_override(self):
    self.hold()
    args=dict(enabled=True, active=True, inputs_valid=True, v_ego=0., gas=False, now=10.2)
    self.assertEqual(self.fsm.update(None, status(10.2, paused=True), **args), (0., True))
    self.assertEqual(self.step(10.3, 7., 0., gas=True), (None, False))
    self.assertEqual(self.fsm.phase, 'held')

  def test_hard_overrides_and_disabled(self):
    for options in (dict(enabled=False), dict(inputs_valid=False), dict(resume=status(10.2, cruise=False))):
      self.fsm=MapStop(); self.hold()
      self.assertEqual(self.step(10.2, 7., 0., **options), (None, False))
      self.assertEqual(self.fsm.phase, 'idle')

  def test_missing_stale_unhealthy_status_no_authority(self):
    for s in ({}, status(8), status(10, healthy=False), status(10, enabled=False)):
      self.assertEqual(self.step(resume=s), (None, False))

  def test_no_initial_capture_near_or_past_stop_or_overridden(self):
    for d in (-5., 0., 8., 221.):
      self.assertEqual(self.step(distance=d), (None, False))
    self.assertEqual(self.step(gas=True), (None, False))
    self.assertEqual(self.step(active=False), (None, False))
    self.assertEqual(self.step(speed=26.), (None, False))

  def test_bad_position_rejected(self):
    for name, value in [('projection_valid',False), ('path_ambiguous',True), ('heading_error_deg',84.1),
                        ('gps_accuracy_m',0), ('gps_accuracy_m',9), ('gps_age_s',1.6),
                        ('match_error_m',9), ('heading_error_deg',float('nan'))]:
      n=nav();n['position_quality'][name]=value
      self.assertIsNone(stop_candidate(n,10.), name)
    n=nav();self.assertIsNone(stop_candidate(n,11.5))
    for change in ({'version':3}, {'route_active':False}, {'enabled':False}, {'route_id':''}):
      n=nav();n.update(change);self.assertIsNone(stop_candidate(n,10.))

  def test_consumed_survives_reroute_and_time_at_rest(self):
    self.hold(); self.step(10.2,7.,0.,status(10.2,sequence=1))
    n=nav(1000,30);n['route_id']='new-route'
    self.assertEqual(self.fsm.update(n,status(1000,sequence=1),enabled=True,active=True,
      inputs_valid=True,v_ego=0.,gas=False,now=1000), (None,False))
    self.assertIn('sign',self.fsm.consumed)

  def test_snapshot_age_reduces_distance(self):
    self.step()
    cap,_=self.fsm.update(nav(), status(10.2), enabled=True,active=True,inputs_valid=True,
                         v_ego=10.,gas=False,now=10.2)
    self.assertAlmostEqual(cap,math.sqrt(1.6*20))


class Resume(unittest.TestCase):
  def engagement(self):
    e=PreAPEngagement(False,750)
    e.process_buttons(2,0,1000,5.,'KPH',True,True,True,False)
    e.process_buttons(0,0,2000,0.,'KPH',True,True,True,True)
    return e

  def step(self,e,t,gas=False,healthy=True,brake=False,speed=0.):
    return e.process_brake_resume(t,speed,brake,gas,healthy,True,True,True)

  def test_transient_health_loss_recovers_but_requires_fresh_tap(self):
    e=self.engagement()
    self.step(e,2100);self.step(e,2200,True)
    self.step(e,2250,True,False)
    self.assertTrue(e.brake_paused_long)
    self.assertFalse(self.step(e,2320))
    self.step(e,2400); self.step(e,2500,True)
    self.assertTrue(self.step(e,2620))
    self.assertEqual(e.resume_sequence,1)

  def test_tap_limits_held_gas_brake_and_speed(self):
    for duration in (20,801):
      e=self.engagement();self.step(e,2100);self.step(e,2200,True)
      self.assertFalse(self.step(e,2200+duration))
    e=self.engagement();self.step(e,2100,True,brake=True)
    self.step(e,2200,True);self.assertFalse(self.step(e,2320))
    e=self.engagement();self.step(e,2100);self.step(e,2200,True,speed=5.)
    self.assertFalse(self.step(e,2320))
    e=self.engagement();self.step(e,2100);self.step(e,2200,True,speed=4.)
    self.assertTrue(self.step(e,2320,speed=5.))

  def test_missing_feedback_not_reported_healthy(self):
    p=PedalFeedback();p.available=True;p.timeout=False
    self.assertFalse(p.update({},1000));self.assertFalse(p.available);self.assertTrue(p.timeout)

  def test_real_engagement_ack_releases_held_stop(self):
    e=self.engagement();m=MapStop()
    with tempfile.TemporaryDirectory() as d:
      p=Path(d)/'state';writer=ResumeStatus(str(p))
      # Acquire while engaged, then brake and stop.
      e.brake_paused_long=False
      writer.publish(e,False,True,True,True,now=10)
      args=dict(enabled=True,active=True,inputs_valid=True,v_ego=5.,gas=False,now=10.)
      m.update(nav(),read_snapshot(p),**args)
      e.brake_paused_long=True
      writer.publish(e,True,True,True,True,now=10.1)
      args.update(active=False,v_ego=0.,now=10.1)
      m.update(nav(10.1,7),read_snapshot(p),**args)
      self.assertEqual(m.phase,'held')
      self.step(e,2100);self.step(e,2200,True);self.assertTrue(self.step(e,2320))
      writer.publish(e,False,True,True,True,now=10.3)
      args.update(active=True,now=10.3)
      self.assertEqual(m.update(nav(10.3,7),read_snapshot(p),**args),(None,False))
      self.assertIn('sign',m.consumed)


class Projection(unittest.TestCase):
  def route(self):
    r=object.__new__(Route)
    r.points=[([0.,0.],0.),([0.,.001],111.),([.001,.001],222.)]
    return r

  def test_heading_selects_correct_direction_and_rejects_perpendicular(self):
    r=self.route()
    self.assertAlmostEqual(r.match([0.,.0005],0,120,0)[1],55.5)
    self.assertIsNone(r.match([0.,.0005],0,60,180))
    self.assertIsNone(r.match([0.,.0005],0,60,84.1))

  def test_crossing_loop_ambiguity(self):
    r=self.route();r.points += [([0.,0.],400.),([0.,.001],511.)]
    self.assertIsNone(r.match([0.,.0005],0,600,0))
    self.assertTrue(r.match_ambiguous)

  def test_bridge_exports_projected_quality_and_consumer_rejects_expiry(self):
    n=nav();n['route_context']=n.pop('position_quality')
    with tempfile.TemporaryDirectory() as d:
      p=Path(d)/'nav';b=DesireBridge(p);b.enabled=True;b.route_active=True
      with patch('phone_navigation.desire_bridge.time.monotonic',return_value=10.):
        b.publish(n,100)
      data=json.loads(p.read_text())
      self.assertEqual(data['version'],4);self.assertTrue(reliable_position(data))
      self.assertIsNotNone(read_navigation(p,10.1)[0])
      self.assertIsNone(read_navigation(p,11.5)[0])
      data['position_quality']['heading_error_deg']=84.1
      self.assertFalse(reliable_position(data))

  def test_relaxed_high_speed_envelope_unchanged(self):
    old=[.3,.6,.9,.8,.74,.68,.66,.64,.62,.61,.6,.58,.55]
    new=ACCEL_PREAP_PROFILES[2]
    self.assertLess(new[0],old[0]);self.assertLess(new[1],old[1])
    self.assertEqual(new[2:],old[2:])


class PlannerIntegration(unittest.TestCase):
  def run_planner(self, *, map_enabled=True, distance=30., speed=5., cruise=50.,
                  mpc_stop=False, model_stop=False, experimental=False, map_state=None):
    import numpy as np
    from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
    cp=NS(brand='tesla', carFingerprint='TESLA_MODEL_S_PREAP', openpilotLongitudinalControl=True,
          pcmCruise=False, steerRatio=15.,wheelbase=2.96,longitudinalActuatorDelay=.4,vEgoStopping=.1)
    class SM(dict):
      def all_checks(self,service_list=None): return True
    sm=SM(carState=NS(vEgo=speed,vCruise=cruise,standstill=speed==0.,aEgo=0.,steeringAngleDeg=0.,gasPressed=False),
          carControl=NS(orientationNED=[]),controlsState=NS(longControlState=lp.LongCtrlState.pid,forceDecel=False),
          liveParameters=NS(angleOffsetDeg=0.),radarState=NS(leadOne=NS(status=False)),
          selfdriveState=NS(enabled=True,personality=2,experimentalMode=experimental),
          modelV2=NS(position=NS(x=[]),meta=NS(disengagePredictions=NS(gasPressProbs=[])),
                     action=NS(desiredAcceleration=-1.5,shouldStop=model_stop)))
    with patch.object(lp,'Params') as params, patch.object(lp,'LongitudinalMpc') as factory, \
         patch.object(lp,'CornerAssist') as corner, patch.object(lp,'read_snapshot') as read, \
         patch.object(lp,'map_curve_speed_cap',return_value=None), \
         patch.object(lp,'navigation_speed_cap',return_value=None), \
         patch.object(lp,'get_accel_from_plan',return_value=(-1.,mpc_stop)), \
         patch.object(lp.time,'monotonic',return_value=10.):
      params.return_value.get_bool.side_effect=lambda key: key=='NAPMapDrivingAssist' and map_enabled
      params.return_value.get.return_value=None
      mpc=factory.return_value
      mpc.v_solution=np.full(len(lp.T_IDXS_MPC),speed)
      mpc.a_solution=np.full(len(lp.T_IDXS_MPC),-1.)
      mpc.j_solution=np.zeros(len(lp.T_IDXS_MPC)-1);mpc.crash_cnt=0
      corner.return_value.apply.side_effect=lambda a,*args:a
      planner=lp.LongitudinalPlanner(cp)
      if map_state is not None: planner.map_stop=map_state
      read.side_effect=[nav(10.,distance),status(10.)]
      planner.update(sm)
      return planner,mpc.update.call_args.args[1],mpc.update.call_args.kwargs['max_accel']

  def test_cruise_ceiling_and_model_mpc_stop_preserved(self):
    p,cap,_=self.run_planner()
    self.assertLess(cap,50/3.6)
    p,cap,_=self.run_planner(cruise=3.)
    self.assertAlmostEqual(cap,3/3.6)
    for mpc,model in [(True,False),(False,True)]:
      p,_,_=self.run_planner(map_enabled=False,mpc_stop=mpc,model_stop=model,experimental=True)
      self.assertTrue(p.output_should_stop)
      self.assertLessEqual(p.output_a_target,-1.)

  def test_actual_planner_adds_stop_only_with_map_gate(self):
    fsm=MapStop();fsm.update(nav(9.9),status(9.9),enabled=True,active=True,
                            inputs_valid=True,v_ego=5.,gas=False,now=9.9)
    p,cap,_=self.run_planner(distance=7.,speed=2.,map_state=fsm)
    self.assertEqual(cap,0.);self.assertTrue(p.output_should_stop)
    p,cap,_=self.run_planner(map_enabled=False,distance=7.,speed=2.)
    self.assertFalse(p.output_should_stop)

  def test_low_speed_softening_does_not_change_high_speed_limit(self):
    _,_,low=self.run_planner(speed=3.)
    _,_,high=self.run_planner(speed=20.)
    self.assertAlmostEqual(low,.6)
    self.assertAlmostEqual(high,1.)


class TrackerIntegration(unittest.TestCase):
  def test_tick_projects_with_wheels_and_expires_stale_fix(self):
    from phone_navigation.route_tracking import NavigationTracker
    raw=dict(duration=100.,legs=[dict(steps=[
      dict(geometry=dict(coordinates=[[0.,0.],[0.,.001]]),maneuver=dict(type='depart'),name=''),
      dict(geometry=dict(coordinates=[[0.,.001],[.001,.001]]),maneuver=dict(type='turn',modifier='right'),name='')])])
    with tempfile.TemporaryDirectory() as d, patch.object(NavigationTracker,'restore_cache'), \
         patch.object(NavigationTracker,'maybe_fetch_mapped_stops'), patch.object(NavigationTracker,'save_cache'), \
         patch('phone_navigation.route_tracking.mapped_stops_enabled',return_value=True):
      bridge=DesireBridge(Path(d)/'nav');bridge.enabled=True
      tracker=NavigationTracker(None,bridge)
      tracker.route=Route(raw);tracker.route_id='route';tracker.destination=dict(label='test')
      tracker.stop_state='ready';tracker.stop_coverage_end=200.;tracker.stop_points=[dict(id='sign',at_m=80.)]
      fix=dict(point=[0.,.0005],mono=10.,speed=4.,bearing_deg=0.,accuracy_m=3.,wheel_speed=4.)
      with patch('phone_navigation.desire_bridge.time.monotonic',return_value=10.5):
        tracker.tick(fix,10.5)
      self.assertAlmostEqual(tracker.progress,tracker.route.points[1][1]/2+2.)
      self.assertTrue(tracker.nav['route_context']['projection_valid'])
      self.assertAlmostEqual(tracker.nav['mapped_stop_sign']['distance_m'],round(80-tracker.progress,1))
      tracker.tick(fix,10.5)
      self.assertAlmostEqual(tracker.progress,tracker.route.points[1][1]/2+2.)
      tracker.tick(fix,12.)
      self.assertFalse(tracker.nav['route_context']['projection_valid'])
