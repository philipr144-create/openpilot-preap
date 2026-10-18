"""Shared command ownership, curve preview and braking authority. No CAN output."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch
from openpilot.selfdrive.controls.lib.acceleration_handoff import UnifiedCruiseSmoother, curve_recovery_accel_limit
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as base

class SharedHandoff(unittest.TestCase):
  def test_ordinary_positive_withdrawal_and_regen_are_continuous(self):
    for personality,decel in ((0,.8),(1,.5),(2,.4)):
      s=UnifiedCruiseSmoother();previous=.6
      for request in [.1]*20+[-.3]*50+[.4]*80:
        current=s.update(request,previous,.05,personality,15.,25.,0.,-2.5,.8)
        self.assertGreaterEqual(current-previous,-decel*.05-1e-10)
        self.assertLessEqual(current-previous,.06)
        previous=current
  def test_braking_bypasses_comfort_and_release_keeps_history(self):
    for reason in ('STOP','FORCE_DECEL','INVALID_INPUT','LEAD_BRAKING','STRONG_BRAKING','SPEED_CEILING','CORNER_BRAKING','PLANNED_MODEL_BRAKING'):
      s=UnifiedCruiseSmoother()
      self.assertEqual(s.update(-2.,.6,.05,2,15.,25.,0.,-2.5,.8,reason),-2.)
      self.assertAlmostEqual(s.update(.4,-2.,.05,2,15.,25.,0.,-2.5,.8),-1.97)
  def test_recorded_capture_threshold_has_no_step(self):
    values=[]
    for gap in (1.115,1.000001,.999999,.967):
      s=UnifiedCruiseSmoother()
      values.append(s.update(.733,.733,.05,1,20.-gap,20.,2.48,-2.5,.9))
    self.assertAlmostEqual(values[1],values[2],places=5)
    self.assertAlmostEqual(values[-1],.733-.5*.05)
  def test_physical_bounds_remain_hard(self):
    s=UnifiedCruiseSmoother()
    self.assertEqual(s.update(.8,.8,.05,1,15.,25.,0.,-2.5,.2),.2)
  def test_preview_cannot_authorize_acceleration_or_braking(self):
    self.assertIsNone(curve_recovery_accel_limit(None,20.,3.))
    self.assertIsNone(curve_recovery_accel_limit(float('nan'),20.,3.))
    self.assertEqual(curve_recovery_accel_limit(18.,20.,3.),0.)
    self.assertEqual(curve_recovery_accel_limit(21.5,20.,3.),.5)

class MapPreview(unittest.TestCase):
  def state(self):
    return dict(version=3,enabled=True,valid=True,control_valid=True,status='ok',mono_time=10.,expires_mono=12.,
                match=dict(distance_m=1.,heading_error_deg=2.,confidence=.9),lookahead_m=300.,
                control_curves=[dict(start_distance_m=100.,end_distance_m=150.,length_m=50.,target_speed_mps=18.,curvature=.01)])
  def test_preview_retains_freshness_quality_and_current_curve(self):
    with tempfile.TemporaryDirectory() as td:
      path=Path(td)/'map';state=self.state();path.write_text(json.dumps(state))
      current=lp.map_curve_speed_cap(path,now=10.,v_ego=20.,include_stops=False)
      future=lp.map_curve_speed_cap(path,now=10.,v_ego=20.,include_stops=False,preview_distance=60.)
      self.assertLess(future,current)
      self.assertEqual(lp.map_curve_speed_cap(path,now=10.,v_ego=20.,include_stops=False,preview_distance=160.),18.)
      self.assertIsNone(lp.map_curve_speed_cap(path,now=12.,v_ego=20.,include_stops=False,preview_distance=60.))
      state['match']['confidence']=.1;path.write_text(json.dumps(state))
      self.assertIsNone(lp.map_curve_speed_cap(path,now=10.,v_ego=20.,include_stops=False,preview_distance=60.))
  def test_successive_curve_tightens_preview_without_changing_present_cap(self):
    with tempfile.TemporaryDirectory() as td:
      path=Path(td)/'map';state=self.state()
      state['control_curves'].append(dict(start_distance_m=160.,end_distance_m=200.,length_m=40.,target_speed_mps=12.,curvature=.025))
      path.write_text(json.dumps(state))
      a=lp.map_curve_speed_cap(path,now=10.,v_ego=20.,include_stops=False)
      b=lp.map_curve_speed_cap(path,now=10.,v_ego=20.,include_stops=False,preview_distance=60.)
      self.assertLess(b,a)
      self.assertLess(curve_recovery_accel_limit(b,20.,3.),.3)

class PlannerUnified(unittest.TestCase):
  setUp=base.PlannerTransitions.setUp
  tick=base.PlannerTransitions.tick
  def prepare(self,experimental=False):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    self.sm['selfdriveState'].experimentalMode=experimental
    self.sm['selfdriveState'].personality=1
    self.sm['carState'].vEgo=20.
    self.sm['radarState'].leadTwo=N(status=False)
    self.sm['modelV2'].velocity=N(t=[0.,1.,3.,6.,10.],x=[20.]*5)
    self.sm['modelV2'].action.desiredAcceleration=.4
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs=[1.,1.]
    self.p.corner_assist.target_speed=None
    self.mpc.source=lp.LongitudinalPlanSource.cruise
    self.plan.return_value=(.4,False)
  def test_mode_switch_keeps_one_state(self):
    self.prepare()
    owner=self.p.cruise_smoother
    for _ in range(100):self.tick()
    previous=self.p.output_a_target
    for mode in [True,False]*10:
      self.sm['selfdriveState'].experimentalMode=mode
      value=self.tick()
      self.assertIs(owner,self.p.cruise_smoother)
      self.assertAlmostEqual(value,previous)
      previous=value
  def test_both_modes_withdraw_positive_drive_gently(self):
    for mode in (False,True):
      self.prepare(mode)
      for _ in range(80):self.tick()
      previous=self.p.output_a_target
      self.plan.return_value=(.1,False)
      self.sm['modelV2'].action.desiredAcceleration=.1
      value=self.tick()
      self.assertAlmostEqual(value,previous-.025)
  def test_priority_leads_and_stops_in_both_modes(self):
    for mode in (False,True):
      self.prepare(mode)
      for case in ('lead1','lead2','stop','force','overspeed'):
        with self.subTest(mode=mode,case=case):
          self.sm['radarState'].leadOne.status=case=='lead1'
          self.sm['radarState'].leadTwo.status=case=='lead2'
          self.sm['controlsState'].forceDecel=case=='force'
          self.sm['carState'].vEgo=26. if case=='overspeed' else 20.
          self.plan.return_value=(-.5,case=='stop')
          self.p.output_a_target=.4
          self.assertEqual(self.tick(),-.5)
  def test_map_preview_same_in_both_modes_and_does_not_invent_regen(self):
    for mode in (False,True):
      self.prepare(mode)
      self.p.nap_map_driving_assist=True
      self.p._params.get_bool.side_effect=lambda k:k in ('NAPAdaptiveAccel','NAPMapDrivingAssist')
      with patch.object(lp,'map_curve_speed_cap',side_effect=lambda **kw:20.15 if kw.get('preview_distance',0)>0 else 30.):
        for _ in range(100):value=self.tick()
        self.assertAlmostEqual(value,.05)
        self.assertFalse(self.p.output_should_stop)
  def test_positive_lead_constraint_cannot_carry_old_drive(self):
    for mode in (False,True):
      self.prepare(mode)
      self.sm['radarState'].leadOne.status=True
      self.p.output_a_target=.4;self.plan.return_value=(.1,False)
      self.assertAlmostEqual(self.tick(),.1)
      self.assertEqual(self.p.handoff_reason,'LEAD_CONSTRAINT')
  def test_disabled_adaptive_retains_existing_path(self):
    self.prepare()
    self.p.nap_adaptive_accel=False
    self.p._params.get_bool.return_value=False;self.p._params.get_bool.side_effect=None
    self.p.output_a_target=.4;self.plan.return_value=(.1,False)
    self.assertEqual(self.tick(),.1)
    self.assertEqual(self.p.handoff_reason,'OFF')
