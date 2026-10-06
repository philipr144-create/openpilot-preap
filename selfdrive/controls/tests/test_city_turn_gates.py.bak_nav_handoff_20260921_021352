"""Exercise actual turn-request code with isolated settings and signal files."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

from cereal import log
from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState, LANE_CHANGE_SPEED_MIN, CITY_TURN_SPEED_MAX
from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire


class CityTurnGates(unittest.TestCase):
  def setUp(self):
    with patch('openpilot.selfdrive.controls.lib.desire_helper.Params'):
      self.helper = DesireHelper()
    self.helper.params = N(get=lambda _: b'1')
    self.cs = N(vEgo=10*.44704, leftBlinker=True, rightBlinker=False,
                steeringPressed=False, steeringTorque=0., leftBlindspot=False,
                rightBlindspot=False, brakePressed=False, gearShifter='drive')

  def test_manual_turn_speed_gate(self):
    self.assertEqual(CITY_TURN_SPEED_MAX, 25*CV.MPH_TO_MS)
    self.assertEqual(LANE_CHANGE_SPEED_MIN, 20*CV.MPH_TO_MS)
    for mph in (0, 10, 19.99, 20, 24, 24.99, 25, 25.01, 30):
      with self.subTest(mph=mph):
        self.setUp()
        self.cs.vEgo=mph*CV.MPH_TO_MS
        self.helper.update(self.cs, True, 1.)
        self.assertEqual(self.helper.desire == log.Desire.turnLeft, mph < 25)

  def test_right_turn_in_extended_range(self):
    self.cs.vEgo=24*CV.MPH_TO_MS
    self.cs.leftBlinker=False
    self.cs.rightBlinker=True
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.turnRight)

  def test_held_blinker_while_slowing_and_speed_exit(self):
    self.cs.vEgo=26*CV.MPH_TO_MS
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.preLaneChange)
    self.cs.vEgo=24*CV.MPH_TO_MS
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.off)
    self.cs.vEgo=25*CV.MPH_TO_MS
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.off)

  def test_disabled_city_turns_keep_ordinary_lane_change_threshold(self):
    self.helper.params=N(get=lambda _: b'0')
    self.cs.vEgo=21*CV.MPH_TO_MS
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.preLaneChange)
    self.cs.steeringPressed=True
    self.cs.steeringTorque=1.
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.laneChangeLeft)

  def test_manual_takeover_latches_until_blinker_cancelled(self):
    self.cs.vEgo=24*CV.MPH_TO_MS
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)
    self.cs.steeringPressed=True
    self.helper.update(self.cs, True, 1.)
    self.assertTrue(self.helper.manual_turn_cancelled)
    self.cs.steeringPressed=False
    for _ in range(30): self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.cs.leftBlinker=False
    self.helper.update(self.cs, True, 1.)
    self.cs.leftBlinker=True
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)

  def test_blindspot_and_lateral_gates(self):
    self.cs.vEgo=24*CV.MPH_TO_MS
    self.cs.leftBlindspot=True
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.cs.leftBlindspot=False
    self.helper.update(self.cs, False, 1.)
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)

  def test_nav_speed_distance_and_override(self):
    with tempfile.TemporaryDirectory() as d:
      root=Path(d)
      nav=NavigationDesire(str(root/'nav'), str(root/'diag'), str(root/'signal'))
      nav.params=N(get_bool=lambda _: True)
      state=dict(enabled=True, route_active=True, received_mono=100., expires_mono=103.,
                 route_state='active', route_id='test', maneuver_id='turn1',
                 position_quality=dict(match_error_m=2., gps_accuracy_m=3., gps_age_s=.1),
                 maneuver=dict(type='turn', modifier='left', distance_m=10.))
      cc=N(latActive=True)
      def tick():
        (root/'nav').write_text(json.dumps(state))
        return nav.update(self.cs, cc, True, now=101., physical_direction=0)
      self.assertEqual(tick(), 'turnLeft')
      self.cs.vEgo=25*.44704
      self.assertEqual(tick(), 'none')
      self.assertIn('25 mph', nav.decision['reason'])
      self.cs.vEgo=10*.44704
      state['maneuver']['distance_m']=25.
      self.assertEqual(tick(), 'none')
      state['maneuver']['distance_m']=10.
      self.cs.steeringPressed=True
      self.assertEqual(tick(), 'none')
      self.cs.steeringPressed=False
      self.assertEqual(tick(), 'none')
      self.assertTrue(nav.blocked)
      state['maneuver']['distance_m']=0.
      self.assertEqual(tick(), 'none')


if __name__ == '__main__': unittest.main(verbosity=2)
