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
    self.helper.params = N(get=lambda _: True)
    self.cs = N(vEgo=10*.44704, leftBlinker=True, rightBlinker=False,
                steeringPressed=False, steeringTorque=0., leftBlindspot=False,
                rightBlindspot=False, brakePressed=False, gearShifter='drive')

  def tick(self, direction=1, valid=True):
    self.now += .05
    self.model.frameId += 1
    self.helper.update(self.cs, True, 1., manual_turn_direction=direction,
                       model=self.model, inputs_valid=valid, now=self.now)
    return self.helper.desire

  def prepare(self):
    self.now = 100.
    self.cs.canValid = True
    self.model = N(frameId=0, position=N(t=[0., 1., 2., 3.], x=[0., 4., 8., 10.], y=[0., -.5, -2., -4.]),
                   orientation=N(z=[0., -.1, -.3, -.6]), action=N(desiredCurvature=-.03))

  def test_early_signal_straight_road_never_requests_turn(self):
    self.prepare()
    self.model.orientation.z = [0.] * 4
    self.model.position.y = [0.] * 4
    self.model.action.desiredCurvature = 0.
    for _ in range(100):
      self.assertEqual(self.tick(), log.Desire.none)

  def test_obvious_future_corner_conditions_model_before_near_path(self):
    self.prepare()
    # The first second is still straight, but the three-second scene is an
    # unmistakable intersection. Condition the model early enough to avoid a
    # late, wide entry; the independent steering gate remains near-path based.
    self.model.orientation.z = [0., 0., -.3, -.6]
    self.model.position.y = [0., 0., -2., -4.]
    self.model.action.desiredCurvature = 0.
    for _ in range(4):
      self.assertEqual(self.tick(), log.Desire.none)
    for _ in range(4):
      self.tick()
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)

  def test_early_signal_can_wait_at_stop_and_reconfirm_after_interruption(self):
    for case in ('stopped', 'brake', 'invalid', 'missing'):
      self.setUp(); self.prepare()
      for _ in range(3): self.assertEqual(self.tick(), log.Desire.none)
      saved_speed, saved_yaw = self.cs.vEgo, self.model.orientation.z
      if case == 'stopped': self.cs.vEgo = 0.
      if case == 'brake': self.cs.brakePressed = True
      if case == 'missing': self.model.orientation.z = []

      desires = [
        self.tick(valid=case != 'invalid')
        for _ in range(10)
      ]

      if case == 'stopped':
        # A genuine sustained corner may condition the model while stopped.
        # Actuator-side ManualTurnCommit still requires vehicle motion.
        self.assertEqual(desires[-1], log.Desire.turnLeft)
      else:
        self.assertTrue(all(d == log.Desire.none for d in desires))

      self.assertFalse(self.helper.manual_entry.blocked)
      self.cs.vEgo = saved_speed
      self.cs.brakePressed = False
      self.model.orientation.z = saved_yaw

      if case == 'stopped':
        self.assertEqual(self.tick(), log.Desire.turnLeft)
      else:
        for _ in range(4): self.assertEqual(self.tick(), log.Desire.none)
        for _ in range(4): self.tick()
        self.assertEqual(self.helper.desire, log.Desire.turnLeft)

  def test_shrinking_remaining_turn_keeps_desire_but_cannot_start_one(self):
    self.prepare()
    for _ in range(10): self.tick()
    self.model.orientation.z = [0., -.02, -.06, -.10]
    self.model.position.y = [0., -.1, -.25, -.5]
    self.model.action.desiredCurvature = -.013
    for _ in range(30): self.assertEqual(self.tick(), log.Desire.turnLeft)
    self.helper.manual_entry.reset()
    for _ in range(30): self.assertEqual(self.tick(), log.Desire.none)

  def test_tap_and_unclassified_lamps_never_request_city_turn(self):
    self.prepare()
    for _ in range(30): self.assertEqual(self.tick(direction=0), log.Desire.none)
    self.assertFalse(self.helper.manual_turn_command_active)

  def test_near_path_requires_sustained_confirmation(self):
    self.prepare()
    for _ in range(4): self.assertEqual(self.tick(), log.Desire.none)
    for _ in range(3): self.tick()
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)

  def test_right_path_and_speed_bounds(self):
    for speed in (0., 4., 8., 9., 11.):
      self.setUp(); self.prepare()
      self.cs.vEgo = speed
      self.cs.leftBlinker, self.cs.rightBlinker = False, True
      self.model.position.y = [-v for v in self.model.position.y]
      self.model.orientation.z = [-v for v in self.model.orientation.z]
      self.model.action.desiredCurvature *= -1
      for _ in range(20): self.tick(direction=2)
      # At zero speed a sustained physical blinker may condition the model,
      # while actuator-side steering remains motion-gated.
      self.assertEqual(
        self.helper.desire,
        log.Desire.turnRight if 0.0 <= speed <= 25 * CV.MPH_TO_MS else log.Desire.none,
      )

  def test_override_cancel_and_opposite_do_not_rearm_until_neutral(self):
    for field in ('steeringPressed', 'brakePressed', 'rightBlinker'):
      self.setUp(); self.prepare()
      for _ in range(10): self.tick()
      self.assertEqual(self.helper.desire, log.Desire.turnLeft)
      setattr(self.cs, field, True)
      self.assertEqual(self.tick(), log.Desire.none)
      setattr(self.cs, field, False)
      # Hazards explicitly clear the direction; steering/brake remain latched.
      if field != 'rightBlinker':
        for _ in range(20): self.assertEqual(self.tick(), log.Desire.none)
      self.cs.leftBlinker = False
      self.tick(direction=0)
      self.cs.leftBlinker = True
      for _ in range(10): self.tick()
      self.assertEqual(self.helper.desire, log.Desire.turnLeft)

  def test_loss_of_path_releases_and_does_not_retrigger(self):
    self.prepare()
    for _ in range(10): self.tick()
    self.model.position.y = [0.]*4
    self.model.orientation.z = [0.]*4
    self.model.action.desiredCurvature = 0.
    for _ in range(15): self.tick()
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertTrue(self.helper.manual_entry.blocked)

  def test_disabled_and_invalid_inputs_never_request_turn(self):
    for disabled in (True, False):
      self.setUp(); self.prepare()
      self.helper.params = N(get=lambda _: not disabled)
      for _ in range(20): self.assertEqual(self.tick(valid=False), log.Desire.none)

  def test_disabled_city_turns_keep_ordinary_lane_change_threshold(self):
    self.helper.params=N(get=lambda _: False)
    self.cs.vEgo=21*CV.MPH_TO_MS
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.lane_change_state, LaneChangeState.preLaneChange)
    self.cs.steeringPressed=True
    self.cs.steeringTorque=1.
    self.helper.update(self.cs, True, 1.)
    self.assertEqual(self.helper.desire, log.Desire.laneChangeLeft)

  def test_repeated_frame_never_confirms_and_stale_frame_revokes(self):
    self.prepare()
    for _ in range(20):
      self.model.frameId -= 1
      self.assertEqual(self.tick(), log.Desire.none)
    self.setUp(); self.prepare()
    for _ in range(10): self.tick()
    self.assertEqual(self.helper.desire, log.Desire.turnLeft)
    self.now += .2
    self.assertEqual(self.tick(), log.Desire.none)

  def test_missing_nonfinite_and_straight_paths_revoke(self):
    for case in ('missing', 'nan', 'straight'):
      self.setUp(); self.prepare()
      for _ in range(10): self.tick()

      if case == 'missing':
        self.model.orientation.z = []
      elif case == 'nan':
        self.model.position.x[1] = float('nan')
      else:
        self.model.position.y = [0., 0., 0., 0.]
        self.model.orientation.z = [0., 0., 0., 0.]
        self.model.action.desiredCurvature = 0.

      self.assertEqual(self.tick(), log.Desire.none)
      self.assertTrue(self.helper.manual_entry.blocked)

  def test_obvious_near_corner_can_start_with_zero_instantaneous_curvature(self):
    """Physical blinker owns side when future path clearly shows a nearby corner.

    Reproduces the road failure where the path already bends strongly into the
    intersection while action.desiredCurvature is still approximately zero.
    """
    for direction in (1, 2):
      self.setUp()
      self.prepare()

      if direction == 2:
        self.cs.leftBlinker = False
        self.cs.rightBlinker = True
        self.model.position.y = [-v for v in self.model.position.y]
        self.model.orientation.z = [-v for v in self.model.orientation.z]

      # Important regression condition: future path is an obvious nearby
      # corner, but instantaneous planner curvature has not started turning.
      self.model.action.desiredCurvature = 0.0

      # Still require sustained confirmation; blinker alone cannot turn.
      for _ in range(4):
        self.assertEqual(
          self.tick(direction=direction),
          log.Desire.none,
        )

      for _ in range(4):
        self.tick(direction=direction)

      expected = (
        log.Desire.turnLeft
        if direction == 1
        else log.Desire.turnRight
      )

      self.assertEqual(self.helper.desire, expected)
      self.assertTrue(self.helper.manual_turn_command_active)

  def test_blinker_owns_side_when_model_draws_opposite_corner(self):
    self.prepare()
    for _ in range(10):
      self.tick()

    self.assertEqual(self.helper.desire, log.Desire.turnLeft)

    # Still a genuine corner, but the model currently draws it to the
    # opposite side. Physical blinker remains authoritative.
    self.model.position.y = [0., .5, 2., 4.]
    self.model.orientation.z = [0., .1, .3, .6]
    self.model.action.desiredCurvature = .03

    self.assertEqual(self.tick(), log.Desire.turnLeft)
    self.assertFalse(self.helper.manual_entry.blocked)

  def test_opposite_instantaneous_curvature_does_not_cancel_matching_path(self):
    self.prepare()

    # First establish a real left City Turn.
    for _ in range(10):
      self.tick()

    self.assertEqual(self.helper.desire, log.Desire.turnLeft)

    # Future path still clearly bends LEFT, but instantaneous model curvature
    # briefly points RIGHT. The blinker + matching future path should retain
    # the left City Turn.
    self.model.action.desiredCurvature = .03

    self.assertEqual(self.tick(), log.Desire.turnLeft)
    self.assertFalse(self.helper.manual_entry.blocked)

  def test_maximum_duration_releases_even_with_credible_path(self):
    self.prepare()
    for _ in range(180): self.tick()
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.assertTrue(self.helper.manual_entry.blocked)

  def test_opposite_full_signal_and_signal_cancellation_revoke(self):
    self.prepare()
    for _ in range(10): self.tick()
    self.cs.leftBlinker, self.cs.rightBlinker = False, True
    self.assertEqual(self.tick(direction=2), log.Desire.none)
    self.cs.rightBlinker = False
    self.assertEqual(self.tick(direction=0), log.Desire.none)
    self.assertFalse(self.helper.manual_entry.blocked)

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
      self.cs.vEgo=26*.44704
      self.assertEqual(tick(), 'turnLeft')
      self.cs.vEgo=36*.44704
      self.assertEqual(tick(), 'none')
      self.assertIn('35 mph', nav.decision['reason'])
      self.cs.vEgo=10*.44704
      state['maneuver']['distance_m']=25.
      self.assertEqual(tick(), 'none')
      state['maneuver']['distance_m']=10.
      # Driver effort cancels the acquired route turn; it cannot fight
      # the driver again on release during the same maneuver.
      self.cs.steeringPressed=True
      self.assertEqual(tick(), 'none')
      self.assertTrue(nav.blocked)
      self.assertEqual(nav.indicator_request, 'none')
      self.cs.steeringPressed=False
      self.assertEqual(tick(), 'none')
      self.assertTrue(nav.blocked)
      state['maneuver_id']='turn2'
      self.assertEqual(tick(), 'turnLeft')
      self.assertFalse(nav.blocked)

      # A maneuver clamped at zero must never start/restart guidance.
      state['maneuver']['distance_m']=0.
      self.assertEqual(tick(), 'none')


if __name__ == '__main__': unittest.main(verbosity=2)
