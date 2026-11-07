"""Pre-AP longitudinal regressions using synthetic states; no CAN transmission."""
import math
import unittest
from types import SimpleNamespace as N

from openpilot.selfdrive.controls.lib.longcontrol import LongControl
from opendbc.car.tesla.preap.virtual_das import JerkLimiter


def setup_controller(fingerprint="TESLA_MODEL_S_PREAP"):
  cp = N(carFingerprint=fingerprint, startingState=False, vEgoStarting=.1,
         stopAccel=-2.5, stoppingDecelRate=2.5, startAccel=.5,
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

  def test_stopping_reaches_full_regen_with_smooth_firm_ramp(self):
    controller, cs = setup_controller()
    controller.pid.i = .3
    previous = 0.
    outputs = []
    for _ in range(100):
      output = controller.update(True, cs, -1., True, (-2.5, .8))
      outputs.append(output)
      self.assertLessEqual(previous - output, .02500001)
      self.assertLessEqual(output, previous)
      previous = output
    self.assertAlmostEqual(outputs[-1], -2.5, places=9)
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


  def test_takeoff_clears_held_stop(self):
    from openpilot.selfdrive.controls.lib.map_stop import MapStop
    ms = MapStop()
    ms.phase = "held"
    ms.event = ("route1", "stop_node_123")

    # When driver presses gas while held at stop
    cap, should_stop = ms.update(
      nav={}, resume={"session": "s1", "sequence": 1, "cruise": True, "enabled": True, "healthy": True, "brake": False, "paused": False, "received_mono": 100., "expires_mono": 102.},
      enabled=True, active=True, inputs_valid=True, v_ego=0.5, gas=True, now=100.1)

    self.assertIsNone(cap)
    self.assertFalse(should_stop)
    self.assertEqual(ms.phase, "idle")
    self.assertIn("stop_node_123", ms.consumed)

    # When driver lets off gas into the turn:
    cap2, should_stop2 = ms.update(
      nav={}, resume={"session": "s1", "sequence": 1, "cruise": True, "enabled": True, "healthy": True, "brake": False, "paused": False, "received_mono": 100.2, "expires_mono": 102.},
      enabled=True, active=True, inputs_valid=True, v_ego=3.0, gas=False, now=100.3)

    self.assertIsNone(cap2)
    self.assertFalse(should_stop2)

if __name__ == '__main__':
  unittest.main()
