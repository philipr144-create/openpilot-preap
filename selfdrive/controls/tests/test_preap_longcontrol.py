"""Pre-AP longitudinal regressions using synthetic states; no CAN transmission."""
import math
import unittest
from types import SimpleNamespace as N

from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from opendbc.car.tesla.preap.virtual_das import JerkLimiter


def setup_controller(fingerprint="TESLA_MODEL_S_PREAP"):
  cp = N(carFingerprint=fingerprint, startingState=False, vEgoStarting=.1,
         stopAccel=-1.5, stoppingDecelRate=1., startAccel=.5,
         longitudinalTuning=N(kpBP=[0., 3., 6., 35.], kpV=[0., 0., 0., 0.],
                              kiBP=[0., 3., 6., 35.], kiV=[.04, .06, .10, .15]))
  cs = N(vEgo=31., aEgo=0., brakePressed=False, cruiseState=N(standstill=False))
  return LongControl(cp), cs


class TestPreAPLongControl(unittest.TestCase):
  def test_measurement_fluctuation_preserves_cruise_correction(self):
    controller, cs = setup_controller()
    controller.pid.i = .2
    cs.aEgo = .25
    controller.update(True, cs, .12, False, (-1.5, .8))
    self.assertGreater(controller.pid.i, .199)
    self.assertLess(controller.pid.i, .2)

  def test_persistent_error_with_noise_builds_correction(self):
    for sign in (-1, 1):
      with self.subTest(sign=sign):
        controller, cs = setup_controller()
        for tick in range(2000):
          cs.aEgo = .22 * math.sin(2 * math.pi * tick * .01 / .8)
          output = controller.update(True, cs, sign * .12, False, (-1.5, .8))
          self.assertTrue(-1.5 <= output <= .8)
        self.assertGreater(sign * controller.pid.i, .30)

  def test_braking_requests_unwind_positive_correction(self):
    for target, maximum_after_step in ((-.1, .195), (-1., .181)):
      controller, cs = setup_controller()
      controller.pid.i = .2
      controller.update(True, cs, target, False, (-1.5, .8))
      self.assertLess(controller.pid.i, maximum_after_step)

  def test_lower_request_changes_output_immediately(self):
    for speed in (2.2352, 6.7056, 31.):
      for target in (.1, 0., -.1, -1.):
        with self.subTest(speed=speed, target=target):
          controller, cs = setup_controller()
          controller.pid.i = .2
          cs.vEgo, cs.aEgo = speed, .5
          before = controller.update(True, cs, .5, False, (-1.5, .8))
          after = controller.update(True, cs, target, False, (-1.5, .8))
          self.assertLessEqual(after, before - (.5 - target) + 1e-12)

  def test_disengage_resets_correction(self):
    controller, cs = setup_controller()
    controller.pid.i = .3
    self.assertEqual(controller.update(False, cs, .2, False, (-1.5, .8)), 0.)
    self.assertEqual(controller.pid.i, 0.)

  def test_stopping_reaches_configured_regen(self):
    controller, cs = setup_controller()
    controller.pid.i = .3
    for _ in range(200):
      output = controller.update(True, cs, -1., True, (-1.5, .8))
    self.assertEqual(output, -1.5)
    self.assertEqual(controller.pid.i, 0.)

  def test_fast_target_changes_respect_command_jerk_limit(self):
    controller, cs = setup_controller()
    limiter = JerkLimiter(dt=.02)
    previous = 0.
    for target in ([.5] * 3 + [-1.] * 3 + [0.] * 3 + [.2] * 3) * 20:
      command = controller.update(True, cs, target, False, (-1.5, .8))
      limited = limiter.update(command)
      self.assertLessEqual(abs(limited - previous), .05 + 1e-12)
      previous = limited


if __name__ == '__main__':
  unittest.main()
