"""Geometry targets and constrained recovery; offline, no actuator output."""
import json
import math
import tempfile
import unittest
from types import SimpleNamespace as N
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.curve_comfort import PROFILES, road_curve_speed
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist

class RoadGeometry(unittest.TestCase):
  def test_speed_depends_on_radius_not_vehicle_history(self):
    for profile in PROFILES.values():
      targets=[road_curve_speed(k,profile) for k in [.002,.004,.008,.0113,.02,.08]]
      self.assertEqual(targets,sorted(targets,reverse=True))
      for k,v in zip([.002,.004,.008,.0113,.02,.08],targets):
        self.assertLessEqual(v*v*k,profile.map_lateral_accel+1e-8)
    # The observed bend's model-path radius is illustrative, not ground truth.
    self.assertTrue(29 < road_curve_speed(.0113,PROFILES['mild'])*2.236936 < 34)
  def test_high_speed_and_profile_order(self):
    for name,p in PROFILES.items():
      self.assertAlmostEqual(road_curve_speed(.002,p),math.sqrt(p.map_lateral_accel/.002),places=7)
    for k in [.002,.005,.01,.04]:
      a=[road_curve_speed(k,PROFILES[n]) for n in ['more','standard','mild']]
      self.assertEqual(a,sorted(a))
  def test_bad_geometry(self):
    for k in [0.,-.01,float('nan'),float('inf'),.121]:self.assertIsNone(road_curve_speed(k,PROFILES['mild']))

class FollowingRecovery(unittest.TestCase):
  def run_case(self,**changes):
    p=N(nap_adaptive_accel=True,output_a_target=.05)
    args=dict(model_accel=.03,mpc_accel=.22,v_cruise=33.,driver_cruise_speed=33.,v_ego=27.,has_lead=True,
              should_stop_e2e=False,should_stop_mpc=False,map_should_stop=False,force_slow_decel=False,
              allow_throttle=True,gas_pressed=False,reset_state=False,follow_recovery_eligible=True)
    args.update(changes)
    return lp.LongitudinalPlanner._experimental_longitudinal_arbiter(p,**args),p
  def test_positive_following_is_bounded_by_mpc_and_existing_support(self):
    v,p=self.run_case();self.assertEqual(v,.22);self.assertEqual(p.exp_long_state,'FOLLOW_RECOVER')
    v,p=self.run_case(mpc_accel=.7);self.assertAlmostEqual(v,.28)
    v,p=self.run_case(model_accel=.5,mpc_accel=.4);self.assertEqual(v,.4)
  def test_braking_and_stops_and_overrides_unchanged(self):
    for kw in [dict(model_accel=-.05),dict(mpc_accel=-.5),dict(should_stop_e2e=True),dict(should_stop_mpc=True),
               dict(map_should_stop=True),dict(force_slow_decel=True),dict(model_braking_required=True),
               dict(follow_recovery_eligible=False),dict(gas_pressed=True),dict(reset_state=True),
               dict(v_ego=4.),dict(v_cruise=27.,driver_cruise_speed=33.)]:
      with self.subTest(kw=kw):
        v,p=self.run_case(**kw);self.assertNotEqual(p.exp_long_state,'FOLLOW_RECOVER')
    self.assertEqual(self.run_case(model_accel=-.05)[0],-.05)
    self.assertEqual(self.run_case(mpc_accel=-.5)[0],-.5)
  def test_no_permission_manufactured(self):
    v,p=self.run_case(allow_throttle=False);self.assertLessEqual(v,0)

class CityRelease(unittest.TestCase):
  def test_tightening_preserved_recovery_bounded(self):
    c=CornerAssist();c.config['corner_assist']=True;c.comfort=PROFILES['standard']
    c.update(.5,5.,.06,-.3,True,1.,12.)
    self.assertAlmostEqual(c.target_speed,math.sqrt(1.2/.06))
    c.update(.5,5.,.012,-.3,True,1.05,12.)
    self.assertAlmostEqual(c.target_speed,5.03)
    self.assertGreaterEqual(c.cap,0.)
    c.update(-1.5,5.,.10,-.3,True,1.10,12.)
    self.assertLess(c.target_speed,4.)
    self.assertLessEqual(c.update(-1.5,5.,.10,-.3,True,1.15,12.),-1.5)
    c.update(.5,5.,.01,-.3,False,1.20,12.)
    self.assertIsNone(c.target_speed)

class MapDiagnostics(unittest.TestCase):
  def test_actual_reject_reason(self):
    with tempfile.NamedTemporaryFile(mode='w') as f:
      json.dump({'enabled':False,'status':'disabled','source':'offline'},f);f.flush()
      d={};self.assertIsNone(lp.map_curve_speed_cap(f.name,now=10.,v_ego=15.,diagnostic=d))
      self.assertEqual(d['reason'],'SNAPSHOT_DISABLED_OR_INVALID');self.assertEqual(d['snapshot_status'],'disabled')
    d={};self.assertIsNone(lp.map_curve_speed_cap('/does/not/exist',now=10.,v_ego=15.,diagnostic=d))
    self.assertEqual(d['reason'],'SNAPSHOT_READ_ERROR')
