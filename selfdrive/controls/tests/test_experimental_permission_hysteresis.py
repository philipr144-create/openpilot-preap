"""Offline permission chatter and braking-priority regressions; no CAN output."""
import unittest
from types import SimpleNamespace as N

from openpilot.selfdrive.controls.tests import test_experimental_longitudinal_v2 as base
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongCtrlState


class ExperimentalPermissionHysteresis(unittest.TestCase):
  setUp = base.ExperimentalLongitudinalV2.setUp
  tick = base.ExperimentalLongitudinalV2.tick
  enable_v2 = base.ExperimentalLongitudinalV2.enable_v2

  def prepare(self, probability):
    self.enable_v2()
    self.sm['modelV2'].action.desiredAcceleration = 0.0
    self.probability(probability)
    self.tick()

  def probability(self, value):
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs = [1.0, value]

  def test_permitted_chatter_keeps_recovery_and_raw_diagnostics(self):
    self.prepare(0.5)
    for probability in [0.39, 0.41] * 40:
      self.probability(probability)
      previous = self.p.output_a_target
      value = self.tick()
      self.assertEqual(self.p.allow_throttle, probability > 0.4)
      self.assertTrue(self.p._exp_allow_throttle)
      self.assertEqual(self.p.exp_long_state, "RECOVER")
      self.assertGreaterEqual(value, previous)
      self.assertLessEqual(value - previous, 0.01500001)

  def test_denied_chatter_stays_coast_until_upper_threshold(self):
    self.prepare(0.3)
    for probability in [0.39, 0.41, 0.45] * 20:
      self.probability(probability)
      self.tick()
      self.assertFalse(self.p._exp_allow_throttle)
      self.assertEqual(self.p.exp_long_state, "COAST")
    previous = self.p.output_a_target
    self.probability(0.46)
    self.assertAlmostEqual(self.tick(), previous + 0.03)
    self.assertTrue(self.p._exp_allow_throttle)

  def test_clear_coast_crosses_zero_with_existing_slew_policy(self):
    self.prepare(0.5)
    for _ in range(30):
      self.tick()
    self.probability(0.35)
    crossed_zero = False
    for _ in range(70):
      previous = self.p.output_a_target
      value = self.tick()
      self.assertFalse(self.p._exp_allow_throttle)
      self.assertAlmostEqual(value, max(-0.3, previous - 0.02))
      crossed_zero |= previous > 0.0 and value <= 0.0
    self.assertTrue(crossed_zero)
    self.assertAlmostEqual(value, -0.3)

  def test_latched_permission_never_delays_stop_or_strong_braking(self):
    self.prepare(0.5)
    self.probability(0.39)
    self.sm['modelV2'].action.shouldStop = True
    self.sm['modelV2'].action.desiredAcceleration = -0.6
    self.assertAlmostEqual(self.tick(), -0.6)
    self.assertTrue(self.p.output_should_stop)
    self.assertEqual(self.p.handoff_reason, "STOP")
    self.sm['modelV2'].action.shouldStop = False
    self.sm['modelV2'].action.desiredAcceleration = -1.5
    self.assertAlmostEqual(self.tick(), -1.5)
    self.assertEqual(self.p.handoff_reason, "STRONG_BRAKING")

  def test_planned_model_stop_and_lead_braking_keep_immediate_authority(self):
    self.prepare(0.5)
    self.probability(0.39)
    self.sm['modelV2'].action.desiredAcceleration = -0.3
    self.sm['modelV2'].velocity = N(t=[0., 1., 3., 6., 10.], x=[12., 11., 8., 3., 0.])
    self.assertAlmostEqual(self.tick(), -0.3)
    self.assertEqual(self.p.handoff_reason, "PLANNED_MODEL_BRAKING")
    self.sm['radarState'].leadOne.status = True
    self.plan.return_value = (-1.2, False)
    self.assertAlmostEqual(self.tick(), -1.2)
    self.assertEqual(self.p.handoff_reason, "LEAD_BRAKING")

  def test_permission_history_resets_outside_active_v2(self):
    self.prepare(0.5)
    self.probability(0.39)
    cases = [(self.sm['selfdriveState'], 'experimentalMode', False, True),
             (self.sm['carState'], 'gasPressed', True, False),
             (self.sm['carState'], 'vEgo', 4.0, 12.0),
             (self.sm['controlsState'], 'longControlState', LongCtrlState.off, LongCtrlState.pid),
             (self.p, 'nap_adaptive_accel', False, True)]
    self.p._frame = 0
    for obj, name, outside, active in cases:
      with self.subTest(name=name):
        self.probability(0.5)
        self.tick()
        self.assertTrue(self.p._exp_allow_throttle)
        self.probability(0.39)
        setattr(obj, name, outside)
        self.tick()
        self.assertIsNone(self.p._exp_allow_throttle)
        setattr(obj, name, active)
        self.tick()
        self.assertFalse(self.p._exp_allow_throttle)

  def test_nonfinite_probability_cannot_retain_permission(self):
    self.prepare(0.5)
    self.probability(float('nan'))
    self.tick()
    self.assertFalse(self.p._exp_allow_throttle)
    self.assertEqual(self.p.exp_long_state, "COAST")
