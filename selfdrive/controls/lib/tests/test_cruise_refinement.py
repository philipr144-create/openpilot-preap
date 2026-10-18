"""Offline speed-support and planner arbitration regressions; no CAN output."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace as N

from openpilot.selfdrive.controls.lib.cruise_refinement import CruiseRefinement, CruiseRefinementConfig, model_support_weight
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as existing
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


class TestRefinement(unittest.TestCase):
  def test_progressive_speed_error_and_filtered_fall(self):
    def settled(error, accel):
      r = CruiseRefinement(.05)
      previous = 0.
      for _ in range(100):
        value = r.speed_support(20., 20.-error, accel, True)
        self.assertLessEqual(value-previous, .01+1e-9)
        self.assertLessEqual(value, .25)
        previous = value
      return value
    self.assertEqual(settled(0., -1.), 0.)
    self.assertEqual(settled(-1., -1.), 0.)
    self.assertLess(settled(.3, 0.), settled(.7, 0.))
    self.assertGreater(settled(.7, -.5), settled(.7, 0.))
    self.assertEqual(settled(5., -2.), .25)

  def test_disable_invalid_and_overspeed_clear_support(self):
    for target, speed, accel, enabled in ((20.,19.,0.,False), (20.,21.,-1.,True),
                                         (float('nan'),19.,0.,True)):
      r = CruiseRefinement(.05)
      for _ in range(100): r.speed_support(20.,19.,-.5,True)
      self.assertEqual(r.speed_support(target,speed,accel,enabled),0.)

  def test_curve_corridor_recovery_and_tighter_limit(self):
    r = CruiseRefinement(.05)
    self.assertEqual(r.curve_target(15.,25.,20.,True),15.)
    self.assertAlmostEqual(r.curve_target(18.,25.,15.,True),15.03)
    self.assertEqual(r.curve_target(12.,25.,15.,True),12.)
    self.assertAlmostEqual(r.curve_target(None,25.,16.,True),16.03)
    for _ in range(400): value = r.curve_target(None,25.,16.,True)
    self.assertEqual(value,25.)
    self.assertEqual(r.curve_target(15.,10.,16.,True),10.)
    self.assertEqual(r.curve_target(None,25.,16.,False),25.)
    self.assertIsNone(r.map_cap)

  def test_dt_and_configuration(self):
    results=[]
    for dt in (.01,.05,.1):
      r=CruiseRefinement(dt, CruiseRefinementConfig(support_max=.1, map_release_accel=.4))
      r.curve_target(10.,20.,10.,True)
      for _ in range(round(1./dt)):
        cap=r.curve_target(20.,20.,10.,True)
        support=r.speed_support(20.,18.,0.,True)
      self.assertAlmostEqual(cap,10.4)
      self.assertAlmostEqual(support,.1)
      results.append(cap)


class TestPlannerRefinement(unittest.TestCase):
  setUp = existing.PlannerTransitions.setUp
  tick = existing.PlannerTransitions.tick

  def prepare(self, experimental=False):
    self.p.nap_adaptive_accel=True
    # Keep periodic refresh from changing the test setting.
    self.p._params.get_bool.side_effect=lambda key:key=='NAPAdaptiveAccel'
    self.sm['carState'].vCruise=10.7*3.6
    self.sm['carState'].aEgo=-.3
    self.sm['radarState'].leadTwo=N(status=False)
    self.sm['selfdriveState'].experimentalMode=experimental
    self.sm['modelV2'].action.desiredAcceleration=0.
    self.plan.return_value=(.05,False)

  def test_normal_support_is_floor_and_experimental_keeps_mpc_ceiling(self):
    self.prepare()
    for _ in range(50): value=self.tick()
    self.assertGreater(value,.05)
    self.assertLessEqual(value,.25)
    self.sm['selfdriveState'].experimentalMode=True
    for _ in range(50): value=self.tick()
    self.assertAlmostEqual(value,.05)

  def test_lead_model_stop_force_decel_and_override_remove_support(self):
    gates = ('leadOne','leadTwo','modelBrake','modelStop','mpcStop','forceDecel','gas','off','toggle')
    for gate in gates:
      with self.subTest(gate=gate):
        self.prepare(experimental=True)
        self.plan.return_value=(.4,False)
        for _ in range(40): self.tick()
        # Experimental now has a single arbiter recovery, no additive helper.
        self.assertEqual(self.p.cruise_refinement.support,0.)
        if gate in ('leadOne','leadTwo'): getattr(self.sm['radarState'],gate).status=True
        elif gate=='modelBrake': self.sm['modelV2'].action.desiredAcceleration=-.3
        elif gate=='modelStop': self.sm['modelV2'].action.shouldStop=True
        elif gate=='mpcStop': self.plan.return_value=(-2.,True)
        elif gate=='forceDecel': self.sm['controlsState'].forceDecel=True
        elif gate=='gas': self.sm['carState'].gasPressed=True
        elif gate=='off': self.sm['controlsState'].longControlState=lp.LongCtrlState.off
        else:
          self.p.nap_adaptive_accel=False
          self.p._params.get_bool.side_effect=lambda key:False
        value=self.tick()
        self.assertEqual(self.p.cruise_refinement.support,0.)
        if gate=='modelBrake': self.assertEqual(value,-.3)
        if gate=='mpcStop': self.assertEqual(value,-2.)
        self.sm['radarState'].leadOne.status=False
        self.sm['modelV2'].action.shouldStop=False
        self.sm['controlsState'].forceDecel=False
        self.sm['carState'].gasPressed=False
        self.sm['controlsState'].longControlState=lp.LongCtrlState.pid

  def test_curve_recovery_keeps_ceiling_and_corner_braking_wins(self):
    self.prepare()
    self.p.nap_map_driving_assist=True
    self.p._params.get_bool.side_effect=lambda key:key in ('NAPAdaptiveAccel','NAPMapDrivingAssist')
    with patch.object(lp,'map_curve_speed_cap',return_value=10.2) as cap:
      self.tick()
      self.assertEqual(self.mpc.update.call_args.args[1],10.2)
      self.assertGreater(self.p.cruise_refinement.support,0.)
      cap.return_value=None
      self.tick()
      self.assertAlmostEqual(self.mpc.update.call_args.args[1],10.23)
      self.p.corner_assist.apply.side_effect=lambda a,*args:min(a,-.8)
      self.assertEqual(self.tick(),-.8)


class TestModelSupport(unittest.TestCase):
  def test_weak_coast_support_fades_and_preserves_planned_slowdown(self):
    model=N(velocity=N(t=[0.,1.,2.,3.],x=[20.,19.9,19.8,19.7]))
    self.assertAlmostEqual(model_support_weight(model,20.,-.05),.75)
    self.assertAlmostEqual(model_support_weight(model,20.,-.1),.5)
    self.assertEqual(model_support_weight(model,20.,-.2),0.)
    model.velocity.x=[20.,19.5,18.,17.]
    self.assertEqual(model_support_weight(model,20.,-.05),0.)
    self.assertEqual(model_support_weight(N(),20.,-.05),0.)
    model.velocity.x=[20.,20.,float('nan'),20.]
    self.assertEqual(model_support_weight(model,20.,-.05),0.)

  def test_loosening_map_cannot_keep_braking_below_actual_speed(self):
    r=CruiseRefinement(.05)
    r.curve_target(12.,25.,16.,True)
    self.assertAlmostEqual(r.curve_target(20.,25.,16.,True),16.03)
    self.assertEqual(r.curve_target(10.,25.,16.,True),10.)


class TestMappedCoastSupport(unittest.TestCase):
  setUp = existing.PlannerTransitions.setUp
  tick = existing.PlannerTransitions.tick

  def test_weak_coast_below_curve_target_gets_bounded_support(self):
    self.p.nap_adaptive_accel=True
    self.p.nap_map_driving_assist=True
    self.p._params.get_bool.side_effect=lambda key:key in ('NAPAdaptiveAccel','NAPMapDrivingAssist')
    self.sm['carState'].vEgo=17.
    self.sm['carState'].vCruise=90.
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['modelV2'].action.desiredAcceleration=-.05
    self.sm['modelV2'].velocity=N(t=[0.,1.,2.,3.],x=[17.,16.9,16.8,16.7])
    self.plan.return_value=(.4,False)
    with patch.object(lp,'map_curve_speed_cap',return_value=20.):
      for _ in range(80): output=self.tick()
      self.assertGreater(output,0.)
      # Recovery below the curve ceiling may use the existing recovery
      # envelope, but never exceed the MPC acceleration constraint.
      self.assertLessEqual(output,.4)
      self.assertEqual(self.mpc.update.call_args.args[1],20.)
      self.sm['radarState'].leadOne.status=True
      self.assertEqual(self.tick(),-.05)
      self.sm['radarState'].leadOne.status=False
      self.sm['modelV2'].action.desiredAcceleration=-.5
      # A steady forecast permits a continuous ordinary deceleration ramp.
      self.assertAlmostEqual(self.tick(),-.07)
      for _ in range(30): output=self.tick()
      self.assertEqual(output,-.5)
      self.sm['modelV2'].action.desiredAcceleration=-1.5
      self.assertEqual(self.tick(),-1.5)
