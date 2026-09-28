"""Regression tests for stopped intent and post-smoothing acceleration gates."""
import unittest
from cereal import log
from openpilot.selfdrive.controls.tests.test_city_turn_gates import CityTurnGates
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner

class StoppedIntent(unittest.TestCase):
  setUp = CityTurnGates.setUp
  prepare = CityTurnGates.prepare
  tick = CityTurnGates.tick

  def test_brake_held_straight_prediction_receives_intent_through_speed_noise(self):
    self.prepare()
    self.cs.brakePressed = True
    self.model.orientation.z = [0.] * 4
    self.model.position.y = [0.] * 4
    self.model.action.desiredCurvature = 0.
    for i in range(20):
      self.cs.vEgo = -.002 if i % 2 else .002
      self.tick()
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)
    self.cs.brakePressed = False
    self.cs.vEgo = .6
    self.assertEqual(self.tick(), log.Desire.turnLeft)
    # Braking while moving still cancels the request immediately.
    self.cs.brakePressed = True
    self.assertEqual(self.tick(), log.Desire.none)
    self.assertTrue(self.helper.manual_entry.blocked)

  def test_reverse_motion_cannot_arm(self):
    self.prepare()
    self.cs.vEgo = -.5
    for _ in range(20):
      self.assertEqual(self.tick(), log.Desire.none)

class AccelConstraints(unittest.TestCase):
  def setUp(self):
    self.p = object.__new__(LongitudinalPlanner)
    self.p.nap_adaptive_accel = True
    self.p.exp_model_brake_frames = 0
    self.p.output_a_target = .4
    self.p.dt = .05

  def call(self, **overrides):
    args = dict(model_accel=-.1, mpc_accel=-.1, v_cruise=20., driver_cruise_speed=20.,
                v_ego=19., has_lead=False, should_stop_e2e=False, should_stop_mpc=False,
                map_should_stop=False, force_slow_decel=False, allow_throttle=True,
                gas_pressed=False, reset_state=False)
    args.update(overrides)
    return self.p._experimental_longitudinal_arbiter(**args)

  def test_smoothing_cannot_exceed_mpc_braking_ceiling(self):
    self.assertLessEqual(self.call(model_accel=-.15), -.1)

  def test_smoothing_cannot_reintroduce_disallowed_throttle(self):
    self.assertLessEqual(self.call(model_accel=-.1, mpc_accel=.5, allow_throttle=False), 0.)

  def test_unverified_visual_brake_is_not_assumed_phantom(self):
    self.assertEqual(self.call(model_accel=-.3, mpc_accel=.5, model_braking_required=True), -.3)

  def test_positive_recovery_remains_bounded_by_mpc(self):
    value=self.call(model_accel=.01, mpc_accel=.2)
    self.assertGreater(value,.01)
    self.assertLessEqual(value,.2)
