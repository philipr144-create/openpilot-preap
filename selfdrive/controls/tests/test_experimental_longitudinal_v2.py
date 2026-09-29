"""Pre-AP Experimental longitudinal V2 regressions; no CAN output."""

import unittest
from types import SimpleNamespace as N

from opendbc.car.tesla.preap.accel_policy import accel_ceiling
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as transitions


class ExperimentalLongitudinalV2(unittest.TestCase):
  setUp = transitions.PlannerTransitions.setUp
  tick = transitions.PlannerTransitions.tick

  def enable_v2(self):
    self.p.nap_adaptive_accel = True
    self.p._params = N(
      get_bool=lambda key: key == "NAPAdaptiveAccel",
      get=lambda key, **kwargs: None,
    )

    self.sm['selfdriveState'].experimentalMode = True
    self.sm['carControl'].orientationNED = [0.0, 0.0, 0.0]
    self.sm['carState'].vEgo = 12.0
    self.sm['carState'].vCruise = 90.0
    self.sm['radarState'].leadTwo = N(status=False)
    self.plan.return_value = (0.5, False)

  def set_permission(self, allowed):
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs = [
      1.0,
      1.0 if allowed else 0.0,
    ]

  def test_false_raw_permission_does_not_become_mpc_ceiling(self):
    self.enable_v2()
    self.set_permission(False)
    self.sm['modelV2'].action.desiredAcceleration = 0.0

    value = self.tick()

    self.assertFalse(self.p.allow_throttle)
    self.assertFalse(self.p.propulsion_allowed)
    self.assertEqual(self.p.exp_long_state, "COAST")

    # V2 keeps the real geometry/profile headroom going into MPC.
    self.assertAlmostEqual(
      self.mpc.update.call_args.kwargs['max_accel'],
      accel_ceiling(12.0, 2),
    )

    # Coast begins gently rather than jumping straight to -0.3 m/s².
    self.assertLessEqual(value, 0.0)
    self.assertGreaterEqual(value, -0.021)

  def test_positive_model_mpc_disagreement_gets_bounded_recovery(self):
    self.enable_v2()
    self.set_permission(False)

    # Mirrors the recorded failure mode: model acceleration itself is positive
    # while the separate gasPressProb permission is low.
    self.sm['modelV2'].action.desiredAcceleration = 0.44
    self.plan.return_value = (0.50, False)

    value = self.tick()

    self.assertFalse(self.p.allow_throttle)
    self.assertTrue(self.p.propulsion_allowed)
    self.assertEqual(self.p.exp_long_state, "RECOVER_LIMITED")
    self.assertGreater(value, 0.0)
    self.assertLessEqual(value, 0.25)

  def test_no_throttle_enters_neutral_before_regen(self):
    self.enable_v2()
    self.set_permission(True)
    self.sm['modelV2'].action.desiredAcceleration = 0.40

    for _ in range(12):
      previous = self.tick()

    self.assertGreater(previous, 0.0)

    # A raw permission loss with no real braking reason reaches neutral first.
    self.set_permission(False)
    self.sm['modelV2'].action.desiredAcceleration = 0.0

    first = self.tick()
    second = self.tick()

    self.assertEqual(first, 0.0)
    self.assertLess(second, 0.0)
    self.assertGreaterEqual(second, -0.021)
    self.assertFalse(self.p.propulsion_allowed)

  def test_real_braking_interrupts_recovery_immediately(self):
    self.enable_v2()
    self.set_permission(False)
    self.sm['modelV2'].action.desiredAcceleration = 0.44
    self.plan.return_value = (0.50, False)

    self.assertGreater(self.tick(), 0.0)
    self.assertTrue(self.p.propulsion_allowed)

    # Strong E2E braking retains immediate authority.
    self.sm['modelV2'].action.desiredAcceleration = -1.50
    value = self.tick()

    self.assertAlmostEqual(value, -1.50)
    self.assertEqual(self.p.exp_long_state, "VERIFIED")

  def test_normal_mode_retains_legacy_no_throttle_behavior(self):
    self.enable_v2()
    self.sm['selfdriveState'].experimentalMode = False
    self.set_permission(False)
    self.sm['modelV2'].action.desiredAcceleration = 0.44
    self.plan.return_value = (0.50, False)

    # Normal mode intentionally keeps the existing temporary coast ceiling.
    # Its existing prev_accel_clip/rise smoothing may leave a tiny positive
    # command during the first few transition frames, so do not require an
    # instantaneous sign change.
    value = self.tick()

    self.assertLessEqual(
      self.mpc.update.call_args.kwargs['max_accel'],
      0.0,
    )

    for _ in range(80):
      value = self.tick()

    self.assertLessEqual(value, 0.0)
    self.assertFalse(self.p.propulsion_allowed)


if __name__ == "__main__":
  unittest.main()
