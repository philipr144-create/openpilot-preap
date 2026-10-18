"""Shared permission and low-speed recovery regressions; no CAN output."""
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
import numpy as np
from openpilot.selfdrive.controls.lib.normal_propulsion import NormalPropulsion
from openpilot.selfdrive.controls.lib.tests import test_unified_long_v4 as v4
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from opendbc.car.tesla.preap.accel_policy import accel_ceiling
from opendbc.car.tesla.preap.constants import ACCEL_PREAP_PROFILES


def model(accel=.02, speeds=None):
  return N(action=N(desiredAcceleration=accel,shouldStop=False),
           velocity=N(t=[0.,1.,3.,6.,10.],x=speeds or [9.6,9.5,8.8,8.5,9.1]))


class Permission(unittest.TestCase):
  def test_weak_permission_with_speed_deficit_needs_sustained_forecast(self):
    p=NormalPropulsion(.05)
    for _ in range(5):self.assertFalse(p.update(model(),9.,13.,enabled=True,valid=True,blocked=False))
    self.assertTrue(p.update(model(),9.,13.,enabled=True,valid=True,blocked=False))
    self.assertEqual(p.reason,'SPEED_DEFICIT')
  def test_braking_stop_invalid_data_and_reached_target_revoke_immediately(self):
    for case in ['near','farstop','strong','explicit','invalid','disabled','blocked','target','nan','stale_speed','short']:
      with self.subTest(case=case):
        p=NormalPropulsion(.05);m=model()
        for _ in range(6):p.update(m,9.,13.,enabled=True,valid=True,blocked=False)
        if case=='near':m.velocity.x=[9.6,9.,8.,8.5,9.1]
        if case=='farstop':m.velocity.x=[9.6,9.5,9.4,3.,0.]
        if case=='strong':m.action.desiredAcceleration=-.7
        if case=='explicit':m.action.shouldStop=True
        if case=='nan':m.velocity.x[2]=float('nan')
        if case=='stale_speed':m.velocity.x[0]=20.
        if case=='short':m.velocity.t[-1]=8.
        self.assertFalse(p.update(m,9.,9.5 if case=='target' else 13.,enabled=case!='disabled',valid=case!='invalid',blocked=case=='blocked'))
        self.assertEqual(p.evidence_time,0.)
  def test_no_new_permission_when_already_at_speed(self):
    p=NormalPropulsion(.05)
    for _ in range(100):self.assertFalse(p.update(model(),9.,9.5,enabled=True,valid=True,blocked=False))


class SharedPlanner(unittest.TestCase):
  setUp=v4.PlannerPath.setUp
  tick=v4.PlannerPath.tick
  def prepare(self):
    v4.PlannerPath.prepare(self)
    self.sm['carState'].vEgo=9.;self.sm['carState'].vCruise=46.8
    self.sm['carState'].brakePressed=False;self.sm['carState'].brake=0.
    self.sm['carControl'].orientationNED=[0.,0.,0.]
    m=model();self.sm['modelV2'].action=m.action;self.sm['modelV2'].velocity=m.velocity
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs=[.2,.30]
    self.plan.return_value=(.5,False)
  def test_one_permission_history_and_smoother_across_modes(self):
    self.prepare();owner=self.p.normal_propulsion;smoother=self.p.cruise_smoother
    # Positive action permits the Experimental branch's existing arbitration.
    self.sm['modelV2'].action.desiredAcceleration=.08
    for _ in range(12):self.tick()
    self.assertTrue(self.p.normal_propulsion_recovery)
    for mode in [True,False]*20:
      previous=self.p.output_a_target
      self.sm['selfdriveState'].experimentalMode=mode
      value=self.tick()
      self.assertIs(owner,self.p.normal_propulsion);self.assertIs(smoother,self.p.cruise_smoother)
      self.assertTrue(self.p.normal_propulsion_recovery)
      self.assertGreaterEqual(value,previous-.02000001)
      self.assertLessEqual(value,previous+.01500001)
  def test_normal_weak_coast_does_not_rewrite_positive_cruise_request(self):
    self.prepare()
    for _ in range(100):value=self.tick()
    self.assertFalse(self.p.allow_throttle)
    self.assertTrue(self.p.propulsion_allowed)
    self.assertAlmostEqual(value,.5)
  def test_restrictive_requests_and_driver_override_remain_authoritative(self):
    for mode in [False,True]:
      for case in ['lead1','lead2','stop','force','brake','gas','invalid','disabled','modelstop']:
        with self.subTest(mode=mode,case=case):
          self.prepare();self.sm['selfdriveState'].experimentalMode=mode
          for _ in range(10):self.tick()
          self.sm['radarState'].leadOne.status=case=='lead1'
          self.sm['radarState'].leadTwo.status=case=='lead2'
          self.sm['controlsState'].forceDecel=case=='force'
          self.sm['carState'].brakePressed=case=='brake'
          self.sm['carState'].gasPressed=case=='gas'
          self.sm['modelV2'].action.shouldStop=case=='modelstop'
          if case in ['lead1','lead2','stop','force','modelstop']:self.plan.return_value=(-1.,case in ['stop','modelstop'])
          if case=='disabled':self.p.nap_adaptive_accel=False;self.p._params.get_bool.side_effect=lambda k:False
          with patch.object(self.sm,'all_checks',return_value=case!='invalid'):value=self.tick()
          if case not in ['stop']:self.assertFalse(self.p.normal_propulsion_recovery)
          if case in ['lead1','lead2','stop','force','modelstop']:self.assertLessEqual(value,-1.)
          self.sm['carState'].brakePressed=False;self.sm['carState'].gasPressed=False
          self.sm['controlsState'].forceDecel=False
  def test_map_and_turn_constraints_still_restrict_recovery(self):
    self.prepare();self.p.nap_map_driving_assist=True
    self.p._params.get_bool.side_effect=lambda k:k in ['NAPAdaptiveAccel','NAPMapDrivingAssist']
    self.plan.return_value=(-.5,False)
    with patch.object(lp,'map_curve_speed_cap',return_value=7.):
      for _ in range(10):value=self.tick()
    self.assertLessEqual(value,-.5)
    self.prepare();self.sm['carState'].steeringAngleDeg=100.
    self.assertLessEqual(self.tick(),0.)


class Profiles(unittest.TestCase):
  def test_only_relaxed_low_speed_envelope_changes(self):
    self.assertEqual(ACCEL_PREAP_PROFILES[1],[.3,.7,1,.981,.951,.921,.9,.871,.826,.8,.782,.737,.65])
    self.assertEqual(ACCEL_PREAP_PROFILES[0],[.4,.8,1.1,1.081,1.051,1.021,1,.957,.89,.85,.832,.787,.7])
    self.assertEqual(ACCEL_PREAP_PROFILES[2][8:],[.56,.54,.52,.50,.47])
    for v in np.linspace(0.,40.,401):self.assertLess(accel_ceiling(v,2),accel_ceiling(v,1))
    self.assertAlmostEqual(accel_ceiling(0.,2),.24)
    self.assertAlmostEqual(accel_ceiling(11.176,2),.54)
    self.assertAlmostEqual(accel_ceiling(13.4112,2),.55)


class RecordedRecovery(unittest.TestCase):
  def test_recorded_085011_forecast_can_recover_without_masking_braking(self):
    # Actual 10/17 forecast during the below-target Chill coast interruption.
    m=N(action=N(desiredAcceleration=0.10261361300945282,shouldStop=False),velocity=N(t=[0.0, 0.009765625, 0.0390625, 0.087890625, 0.15625, 0.244140625, 0.3515625, 0.478515625, 0.625, 0.791015625, 0.9765625, 1.181640625, 1.40625, 1.650390625, 1.9140625, 2.197265625, 2.5, 2.822265625, 3.1640625, 3.525390625, 3.90625, 4.306640625, 4.7265625, 5.166015625, 5.625, 6.103515625, 6.6015625, 7.119140625, 7.65625, 8.212890625, 8.7890625, 9.384765625, 10.0],x=[10.071245193481445, 10.068686485290527, 10.060650825500488, 10.054356575012207, 10.028047561645508, 10.016263008117676, 10.011809349060059, 9.999444961547852, 9.998543739318848, 9.974167823791504, 9.965678215026855, 9.94925308227539, 9.90806770324707, 9.873940467834473, 9.811509132385254, 9.737465858459473, 9.642422676086426, 9.515429496765137, 9.383496284484863, 9.240227699279785, 9.043456077575684, 8.83833122253418, 8.657264709472656, 8.452042579650879, 8.285219192504883, 8.139374732971191, 8.036330223083496, 7.991547107696533, 7.964269638061523, 8.008539199829102, 8.055166244506836, 8.149223327636719, 8.220524787902832]))
    p=NormalPropulsion(.05)
    for _ in range(6):allowed=p.update(m,9.001962661743164,12.9548,enabled=True,valid=True,blocked=False)
    self.assertTrue(allowed)
    m.action.desiredAcceleration=-.60
    self.assertFalse(p.update(m,9.001962661743164,12.9548,enabled=True,valid=True,blocked=False))
