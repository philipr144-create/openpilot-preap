"""Offline model-curvature -> angle controller -> actual Pre-AP CAN sign tests."""
import math
import unittest
from unittest.mock import patch

import numpy as np
from cereal import car, log
from opendbc.can import CANPacker, CANParser
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.preap.teslacan import TeslaCANPreAP
from opendbc.car.tesla.values import CANBUS
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.drive_helpers import get_curvature_from_plan
from openpilot.selfdrive.controls.lib.latcontrol_angle import LatControlAngle
from openpilot.selfdrive.controls.lib.navigation_turn_commit import NavigationTurnCommit


class TurnSignConventions(unittest.TestCase):
  def setUp(self):
    cp = CarInterface.get_non_essential_params('TESLA_MODEL_S_PREAP')
    self.vm = VehicleModel(cp)
    self.controller = LatControlAngle(cp, None, .01)
    self.cs = car.CarState.new_message()
    self.cs.vEgo = 4.
    self.params = log.LiveParametersData.new_message()
    self.params.roll = 0.
    self.params.angleOffsetDeg = 0.
    self.can = TeslaCANPreAP({CANBUS.party: CANPacker('tesla_preap')})
    with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
      self.nav = NavigationTurnCommit()
    self.nav.enabled = True
    self.nav._refresh = lambda _: None

  def request(self, direction, model_curvature, now=100.):
    self.nav.decision = dict(desire='turnLeft' if direction == 'left' else 'turnRight',
                             confirmed=True, distance_m=6., modifier=direction)
    return self.nav.update(model_curvature, self.cs, True, now)

  def wire_angle(self, curvature):
    _, angle, _ = self.controller.update(True, self.cs, self.vm, self.params, False, curvature, False, .2)
    frame = self.can.create_steering_control(0, angle, True)
    parser = CANParser('tesla_preap', [('DAS_steeringControl', 0)], CANBUS.party)
    parser.update([(1000000000, [frame])])
    return angle, parser.vl['DAS_steeringControl']['DAS_steeringAngleRequest']

  def test_model_yaw_to_angle_to_tesla_can(self):
    # Model coordinates: +Y/+yaw point right. At constant yaw rate k*v,
    # get_curvature_from_plan must preserve the signed model curvature k.
    for sign in (-1., 1.):
      k = sign * .01
      ts = np.array([0., 1., 2.])
      curve = get_curvature_from_plan(k*self.cs.vEgo*ts,
                                      np.full(3, k*self.cs.vEgo), ts, self.cs.vEgo, 1.)
      self.assertAlmostEqual(curve, k)
      angle, wire = self.wire_angle(curve)
      self.assertLess(sign * angle, 0.)
      self.assertGreater(sign * wire, 0.)
      self.assertAlmostEqual(wire, -angle, delta=.11)
      # The inverse vehicle-model transform used by controlsd closes the loop.
      self.assertAlmostEqual(-self.vm.calc_curvature(math.radians(angle), self.cs.vEgo, 0.), curve)

  def test_navigation_direction_through_real_controller_and_can(self):
    for direction, sign in (('left', -1.), ('right', 1.)):
      for speed in (1., 4., 8.):
        with self.subTest(direction=direction, speed=speed):
          self.cs.vEgo = speed
          curve = self.request(direction, 0.)
          self.assertAlmostEqual(curve, sign * .055)
          angle, wire = self.wire_angle(curve)
          self.assertLess(sign * angle, 0.)
          self.assertGreater(sign * wire, 0.)

  def test_matching_stronger_model_is_preserved_opposite_is_corrected(self):
    for direction, sign in (('left', -1.), ('right', 1.)):
      self.assertEqual(self.request(direction, sign*.08), sign*.08)
      self.assertEqual(self.request(direction, -sign*.03), sign*.055)

  def test_driver_handoff_ramps_in_correct_direction(self):
    for direction, sign in (('left', -1.), ('right', 1.)):
      self.setUp()
      self.cs.steeringPressed = True
      self.assertEqual(self.request(direction, 0., 100.), 0.)
      self.cs.steeringPressed = False
      self.assertEqual(self.request(direction, 0., 100.1), 0.)
      halfway = self.request(direction, 0., 100.475)
      self.assertAlmostEqual(halfway, sign*.055*.5)
      self.assertAlmostEqual(self.request(direction, 0., 101.), sign*.055)
