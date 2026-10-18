"""Relative-speed forecast and persistent weak-coasting regressions; no CAN I/O."""
import unittest
import json
from pathlib import Path
from types import SimpleNamespace as N
from openpilot.selfdrive.controls.lib.cruise_refinement import model_support_weight
from openpilot.selfdrive.controls.tests import test_intent_and_accel_constraints as constraints
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as transitions
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from cereal import log
from openpilot.selfdrive.controls.lib.acceleration_handoff import handoff_acceleration

class RelativeForecast(unittest.TestCase):
  def test_recorded_57_mph_coasting_forecasts(self):
    samples=json.loads((Path(__file__).parent / "fixtures/experimental_model_offset.json").read_text())
    for sample in samples:
      with self.subTest(speed=sample['speed'],request=sample['request']):
        model=N(velocity=N(t=sample['times'],x=sample['speeds']))
        self.assertGreater(model_support_weight(model,sample['speed'],sample['request']),0.)

  def test_constant_model_speed_offset_does_not_invent_slowdown(self):
    # Recorded wheel speed 20.74, model initial 19.22, ~3s model 19.27 m/s.
    for offset in (0., -1.52, 1.52):
      model=N(velocity=N(t=[0.,1.,2.,3.],x=[20.74+offset,20.76+offset,20.77+offset,20.79+offset]))
      self.assertAlmostEqual(model_support_weight(model,20.74,-.118),.41)

  def test_real_prediction_drop_and_unreliable_offset_keep_braking(self):
    for speeds in ([19.22,19.,18.8,18.6], [17.,17.,17.,17.]):
      model=N(velocity=N(t=[0.,1.,2.,3.],x=speeds))
      self.assertEqual(model_support_weight(model,20.74,-.118),0.)
    self.assertEqual(model_support_weight(N(),20.,-.3),0.)

class PersistentRecovery(unittest.TestCase):
  setUp=constraints.AccelConstraints.setUp
  call=constraints.AccelConstraints.call

  def test_validated_weak_coast_does_not_turn_into_regen_after_timer(self):
    previous=self.p.output_a_target
    for _ in range(100):
      value=self.call(model_accel=-.18,mpc_accel=.5,v_ego=17.)
      self.assertGreaterEqual(value,0.)
      self.assertLessEqual(value,.5)
      # Selection is bounded; the final handoff/actuator owns command slew.
      self.assertLessEqual(value - (-.18), .25 + 1e-9)
      self.p.output_a_target=previous=value

  def test_alternating_coast_and_accel_remain_continuous(self):
    previous=.4
    for i in range(100):
      selected=self.call(model_accel=-.18 if i%2 else .03,mpc_accel=.5,v_ego=18.5)
      value=handoff_acceleration(selected,previous,self.p.dt,1,-2.5,.5)
      self.assertGreaterEqual(value,0.)
      self.assertGreaterEqual(value-previous,-.6*self.p.dt-1e-9)
      self.assertLessEqual(value-previous,.4*self.p.dt+1e-9)
      self.p.output_a_target=previous=value

  def test_recovery_below_curve_cap_keeps_effective_ceiling(self):
    self.assertGreater(self.call(model_accel=-.05,mpc_accel=.4,v_ego=17.,v_cruise=20.,driver_cruise_speed=25.),0.)
    self.assertEqual(self.call(model_accel=-.3,mpc_accel=-.4,v_ego=21.,v_cruise=20.,driver_cruise_speed=25.),-.4)

  def test_new_constraints_interrupt_recovery_immediately(self):
    for gate in ('has_lead','should_stop_e2e','should_stop_mpc','map_should_stop','force_slow_decel','model_braking_required'):
      with self.subTest(gate=gate):
        self.assertEqual(self.call(model_accel=-.3,mpc_accel=.5,**{gate:True}),-.3)
    self.assertEqual(self.call(model_accel=-1.5,mpc_accel=.5),-1.5)

class PlannerSource(unittest.TestCase):
  setUp=transitions.PlannerTransitions.setUp
  tick=transitions.PlannerTransitions.tick

  def test_model_braking_is_not_mislabeled_cruise(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['modelV2'].action.desiredAcceleration=-.3
    self.plan.return_value=(.5,False)
    self.mpc.source=log.LongitudinalPlan.LongitudinalPlanSource.cruise
    self.assertEqual(self.tick(),-.3)
    self.assertEqual(self.mpc.source,log.LongitudinalPlan.LongitudinalPlanSource.e2e)

  def test_recorded_offset_recovers_through_actual_planner_arbitration(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    self.sm['carState'].vEgo=20.74
    self.sm['carState'].vCruise=91.73
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['modelV2'].velocity=N(t=[0.,1.,2.,3.],x=[19.22,19.24,19.25,19.27])
    self.sm['modelV2'].action.desiredAcceleration=-.118
    self.plan.return_value=(.8,False)
    for _ in range(100): value=self.tick()
    self.assertGreater(value,0.)
    previous=value
    for i in range(100):
      self.sm['modelV2'].action.desiredAcceleration=-.18 if i%2 else .03
      value=self.tick()
      # This recorded fixture has only a 3 s forecast. It can qualify
      # bounded support, but cannot authorize the new 10 s comfort gate.
      # Preserve the model request as a lower bound, with MPC as ceiling.
      self.assertGreaterEqual(value,min(0.,self.sm["modelV2"].action.desiredAcceleration))
      self.assertLessEqual(value,.8)
      self.assertLessEqual(value-previous,(.03 if previous<0. else .015)+1e-9)
      previous=value
    # A newly detected lead revokes the optional recovery immediately.
    self.sm['radarState'].leadOne.status=True
    self.sm['modelV2'].action.desiredAcceleration=-.3
    self.assertEqual(self.tick(),-.3)
