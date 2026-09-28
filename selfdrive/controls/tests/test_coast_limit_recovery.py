"""A temporary model coast ceiling must not become a remembered turn limit."""
from types import SimpleNamespace as N
import unittest
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as transitions
from opendbc.car.tesla.preap.accel_policy import accel_ceiling

class CoastLimitRecovery(unittest.TestCase):
  setUp = transitions.PlannerTransitions.setUp
  tick = transitions.PlannerTransitions.tick

  def set_permission(self, allowed):
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs = [1., float(allowed)]

  def test_throttle_return_restores_mpc_headroom_without_command_jump(self):
    self.sm['selfdriveState'].experimentalMode = True
    self.sm['carControl'].orientationNED = [0., .04, 0.]
    for _ in range(80):
      self.set_permission(False)
      self.tick()
    self.assertFalse(self.p.allow_throttle)
    self.assertLessEqual(self.p.output_a_target, 0.)
    previous = self.p.output_a_target
    self.set_permission(True)
    current = self.tick()
    self.assertAlmostEqual(self.mpc.update.call_args.kwargs['max_accel'], accel_ceiling(10., 2))
    self.assertLessEqual(current - max(previous, 0.), .01000001)

  def test_current_no_throttle_and_turn_limits_still_apply(self):
    self.sm['carControl'].orientationNED = [0., .04, 0.]
    self.set_permission(False)
    for _ in range(80): self.tick()
    self.assertLessEqual(self.p.output_a_target, 0.)
    self.set_permission(True)
    self.sm['carState'].steeringAngleDeg = 100.
    self.tick()
    self.assertEqual(self.mpc.update.call_args.kwargs['max_accel'], 0.)
    self.assertLessEqual(self.p.output_a_target, 0.)
