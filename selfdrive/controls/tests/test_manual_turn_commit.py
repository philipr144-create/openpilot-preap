"""Offline manual commitment, ownership, and navigation regression tests; no CAN I/O."""
import copy
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnCommit, NavigationTurnCommit
from openpilot.selfdrive.controls.lib.tap_lane_change import TapModel
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire
from opendbc.car.tesla.preap.tap_lane_change import PhysicalStalk, TapController, crc8


class ManualTests(unittest.TestCase):
  def setUp(self):
    self.helper = ManualTurnCommit()
    self.now = 100.0
    self.cs = N(vEgo=4., canValid=True, leftBlinker=True, rightBlinker=False,
                steeringPressed=False, steeringDisengage=False, brakePressed=False,
                leftBlindspot=False, rightBlindspot=False, gearShifter='drive')
    self.d = dict(manual_turn_direction=1, city_turns_enabled=True, source='driver_desire_helper',
                  final_desire='turnLeft', model_evaluated=True, signal_owner='driver',
                  tap_signal_active=False, synthetic_indicator_request='none')
    self.model = N(frameId=0, position=N(t=[0., 1., 2., 3.], x=[0., 4., 8., 10.], y=[0., -.5, -2., -4.]),
                   orientation=N(z=[0., -.1, -.3, -.6]), action=N(desiredCurvature=-.03))
    self.curvature = -.005
    self.actual = -.03
    self.healthy = True
    self.lat = True

  def tick(self, dt=.05, new_frame=True):
    self.now += dt
    if new_frame:
      self.model.frameId += 1
    self.d.update(received_mono=self.now, frame_id=self.model.frameId)
    return self.helper.update(self.curvature, self.cs, self.lat, self.d, self.model, self.healthy, self.actual, self.now)

  def start(self):
    for _ in range(27):
      result = self.tick()
    self.assertTrue(self.helper.active)
    return result

  def straight(self):
    self.model.position.y = [0.] * 4
    self.model.orientation.z = [0.] * 4
    self.model.action.desiredCurvature = 0.
    self.curvature = 0.

  def test_left_and_right_sign_progressive_bounded_floor(self):
    for direction in (1, 2):
      self.setUp()
      sign = -1 if direction == 1 else 1
      if direction == 2:
        self.cs.leftBlinker, self.cs.rightBlinker = False, True
        self.d.update(manual_turn_direction=2, final_desire='turnRight')
        self.model.position.y = [-v for v in self.model.position.y]
        self.model.orientation.z = [-v for v in self.model.orientation.z]
        self.model.action.desiredCurvature *= -1
        self.curvature *= -1
        self.actual *= -1
      outputs = [sign * self.tick() for _ in range(30)]
      self.assertEqual(outputs[0], .005)
      self.assertAlmostEqual(outputs[-1], .005)  # Matching model path is never tightened.
      self.assertTrue(self.helper.model_acquired)
      self.assertTrue(all(b >= a for a, b in zip(outputs, outputs[1:])))
      self.curvature = sign * .04
      self.assertEqual(self.tick(), self.curvature)  # Never reduce model authority.

  def test_early_model_supported_turn_gets_small_curvature_floor(self):
    # Future path clearly shows a matching turn, but the immediate model
    # curvature is still weak. After sustained confirmation we should begin
    # a small arc instead of waiting for desiredCurvature to jump sharply.
    self.model.action.desiredCurvature = -.006
    self.curvature = -.002

    out = self.start()

    self.assertTrue(self.helper.active)
    self.assertEqual(self.helper.state, 'MANUAL TURN')
    self.assertLessEqual(self.helper.floor, self.helper.SEED_MAX_CURVATURE)
    self.assertGreaterEqual(self.helper.floor, self.helper.EARLY_CURVATURE_FLOOR)
    self.assertLessEqual(out, -.008)

    self.model.action.desiredCurvature = -.030
    self.curvature = -.030
    self.assertEqual(self.tick(), -.030)
    # As the model unwinds, never put the seed floor back underneath it.
    self.curvature = -.002
    self.assertEqual(self.tick(), -.002)

  def test_shrinking_remaining_heading_is_continuation_not_completion(self):
    self.start()
    self.model.orientation.z = [0., -.05, -.15, -.26]
    self.model.position.y = [0., -.2, -.5, -1.]
    self.model.action.desiredCurvature = -.013
    self.curvature = -.013
    for _ in range(30):
      self.assertLessEqual(self.tick(), -.013)
      self.assertTrue(self.helper.active)
    self.assertFalse(self.helper.blocked)

  def test_straight_near_path_releases_floor_despite_far_residual_bend(self):
    # Reproduce an on-road late release: the vehicle has already rotated
    # through the corner and the road directly ahead is straight, but the
    # far horizon still contains enough residual bend to pass continuation
    # geometry. The committed .045 floor must return to the live planner.
    self.start()
    while self.helper.heading < self.helper.EXIT_MIN_HEADING:
      self.tick()

    self.model.orientation.z = [0., -.02, -.12, -.25]
    self.model.position.y = [0., -.1, -.5, -1.2]
    self.model.action.desiredCurvature = -.006
    self.curvature = -.010

    evidence = self.helper.turn_shape_evidence(
      self.model,
      -1.0,
      continuing=True,
    )
    self.assertIsNotNone(evidence)
    self.assertTrue(evidence[0])

    self.assertEqual(self.tick(), self.curvature)
    self.assertFalse(self.helper.active)
    self.assertTrue(self.helper.blocked)

  def test_preentry_health_interruption_requires_fresh_dwell_not_new_signal(self):
    for _ in range(3): self.tick()
    self.healthy = False
    self.tick()
    self.assertFalse(self.helper.blocked)
    self.healthy = True
    for _ in range(5):
      self.tick(); self.assertFalse(self.helper.active)
    self.start()

  def test_straight_blinker_never_invents_turn(self):
    self.straight()
    for _ in range(60):
      self.assertEqual(self.tick(), 0.)
      self.assertFalse(self.helper.active)

  def test_distant_path_cannot_start_commitment(self):
    self.model.orientation.z = [0., 0., -.3, -.6]
    for _ in range(40):
      self.assertEqual(self.tick(), self.curvature)
      self.assertFalse(self.helper.active)

  def test_straight_and_shallow_shapes_do_not_arm(self):
    # Blinker owns direction, but a genuine corner still has to exist.
    for field in ('straight', 'shallow'):
      self.setUp()
      if field == 'straight':
        self.model.orientation.z = [0., 0., 0., 0.]
        self.model.position.y = [0., 0., 0., 0.]
      else:
        self.model.position.y = [0., -.05, -.15, -.25]
        self.model.orientation.z = [0., -.01, -.02, -.03]

      for _ in range(30):
        self.tick()

      self.assertFalse(self.helper.active)

  def test_matching_geometry_and_blinker_override_opposite_immediate_curvature(self):
    # The future path clearly bends left and the driver signals left, but
    # immediate desiredCurvature briefly points right. City Turns should
    # choose the signaled/path-supported direction.
    self.model.action.desiredCurvature = .020
    self.curvature = .002

    out = self.start()

    self.assertTrue(self.helper.active)
    self.assertEqual(self.helper.state, 'MANUAL TURN')
    self.assertLess(out, 0.0)
    self.assertGreaterEqual(
      self.helper.floor,
      self.helper.EARLY_CURVATURE_FLOOR,
    )

  def test_obvious_near_corner_with_zero_instantaneous_curvature_commits(self):
    """The control-side floor covers the same zero-curvature road regression."""
    for direction in (1, 2):
      self.setUp()
      sign = -1.0 if direction == 1 else 1.0

      if direction == 2:
        self.cs.leftBlinker, self.cs.rightBlinker = False, True
        self.d.update(manual_turn_direction=2, final_desire='turnRight')
        self.model.position.y = [-v for v in self.model.position.y]
        self.model.orientation.z = [-v for v in self.model.orientation.z]
        self.actual *= -1

      self.model.action.desiredCurvature = 0.0
      self.curvature = 0.0

      out = self.start()

      self.assertTrue(self.helper.active)
      self.assertEqual(self.helper.state, 'MANUAL TURN')
      self.assertGreater(sign * out, 0.0)
      self.assertGreaterEqual(
        self.helper.floor,
        self.helper.EARLY_CURVATURE_FLOOR,
      )

  def test_single_frame_cannot_qualify(self):
    for _ in range(30): self.assertEqual(self.tick(new_frame=False), self.curvature)
    self.assertFalse(self.helper.active)

  def test_instantaneous_curvature_noise_does_not_restart_geometry_dwell(self):
    self.curvature = 0.
    for _ in range(5): self.tick()
    self.model.action.desiredCurvature = 0.
    self.tick()
    self.model.action.desiredCurvature = -.03
    for _ in range(5): self.tick()
    self.assertTrue(self.helper.active)

  def test_missing_corner_geometry_restarts_dwell(self):
    for _ in range(5): self.tick()
    saved = self.model.orientation.z
    self.model.orientation.z = [0.]*4
    self.tick()
    self.model.orientation.z = saved
    for _ in range(5):
      self.tick()
      self.assertFalse(self.helper.active)

  def test_straight_path_releases_without_synthetic_retention(self):
    self.start()
    self.straight()
    self.assertEqual(self.tick(), 0.)
    self.assertTrue(self.helper.blocked)

  def test_no_retention_before_measured_turn(self):
    self.actual = 0.
    self.start()
    self.straight()
    self.assertEqual(self.tick(), 0.)
    self.assertTrue(self.helper.blocked)

  def test_completion_and_signal_still_on_no_retrigger(self):
    self.start()
    for _ in range(60): self.tick()
    self.assertGreater(self.helper.heading, math.radians(25))
    self.straight()
    for _ in range(7): self.tick()
    self.assertFalse(self.helper.active)
    self.assertTrue(self.helper.blocked)
    self.model.action.desiredCurvature = -.03
    for _ in range(30): self.assertEqual(self.tick(), 0.)

  def test_override_immediate_latched_until_neutral(self):
    for override in ('steeringPressed', 'steeringDisengage'):
      self.setUp()
      self.start()
      setattr(self.cs, override, True)
      self.assertEqual(self.tick(), self.curvature)
      setattr(self.cs, override, False)
      if override == "steeringPressed":
        for _ in range(30): self.tick()
        self.assertTrue(self.helper.active)
      else:
        for _ in range(30): self.assertEqual(self.tick(), self.curvature)
      self.cs.leftBlinker = False
      self.tick()
      self.cs.leftBlinker = True
      self.assertEqual(self.start(), self.curvature)  # Reacquire model without tightening it.

  def test_opposite_intent_and_opposite_model_cancel(self):
    self.start()
    self.cs.leftBlinker, self.cs.rightBlinker = False, True
    self.d.update(manual_turn_direction=2, final_desire='turnRight')
    self.assertEqual(self.tick(), self.curvature)
    self.assertTrue(self.helper.blocked)
    self.setUp()
    self.start()
    self.model.action.desiredCurvature = .02

    # Immediate desiredCurvature may briefly oppose the blinker, but matching
    # future path geometry keeps the already-qualified City Turn on its
    # signaled side.
    out = self.tick()
    self.assertLess(out, 0.0)
    self.assertTrue(self.helper.active)

  def test_gates_cancel_without_replay(self):
    cases = [('cs', 'brakePressed', True), ('cs', 'canValid', False), ('cs', 'vEgo', 12.0),
             ('cs', 'vEgo', math.nan), ('cs', 'vEgo', 0.), ('cs', 'leftBlindspot', True),
             ('cs', 'rightBlinker', True), ('cs', 'gearShifter', 'reverse'),
             ('d', 'city_turns_enabled', False), ('d', 'manual_turn_direction', 0),
             ('d', 'source', 'navigation'), ('d', 'source', 'tap_lane_change'),
             ('d', 'source', 'navigation_signal_cleanup'), ('d', 'signal_owner', 'navigation'),
             ('d', 'tap_signal_active', True), ('d', 'synthetic_indicator_request', 'left'),
             ('d', 'model_evaluated', False), ('d', 'manual_turn_blocked', True), ('d', 'final_desire', 'turnRight')]
    for obj, key, value in cases:
      with self.subTest(key=key, value=value):
        self.setUp()
        self.start()
        if obj == 'cs': setattr(self.cs, key, value)
        else: self.d[key] = value
        self.assertEqual(self.tick(), self.curvature)
        self.assertFalse(self.helper.active)
        self.assertTrue(self.helper.blocked)

  def test_health_disengagement_staleness_and_invalid_model(self):
    for gate in ('health', 'lat', 'stale', 'future', 'unpaired', 'nan', 'missing', 'gap'):
      with self.subTest(gate=gate):
        self.setUp()
        self.start()
        if gate == 'health': self.healthy = False
        if gate == 'lat': self.lat = False
        if gate == 'stale': self.d['received_mono'] = self.now - 1.
        if gate == 'future': self.d['received_mono'] = self.now + 1.
        if gate == 'unpaired': self.d['frame_id'] += 10
        if gate == 'nan': self.model.position.y[2] = math.nan
        if gate == 'missing': self.model.orientation.z = []
        if gate == 'gap': self.now += 1.
        result = self.helper.update(self.curvature, self.cs, self.lat, self.d, self.model, self.healthy, self.actual, self.now)
        self.assertEqual(result, self.curvature)
        self.assertFalse(self.helper.active)

  def test_timeout_and_acceleration_cap(self):
    self.cs.vEgo = 8.
    self.actual = -.01
    self.start()
    self.assertLessEqual(self.helper.floor * self.cs.vEgo**2, 1.5)
    for _ in range(180): self.tick()
    self.assertFalse(self.helper.active)
    self.assertTrue(self.helper.blocked)

  def test_classified_physical_hold_vetoes_opposite_model_before_floor(self):
    for direction, sign in ((1, -1.), (2, 1.)):
      self.setUp()
      if direction == 2:
        self.cs.leftBlinker, self.cs.rightBlinker = False, True
      with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
        nav = NavigationTurnCommit()
      nav._refresh = lambda _: None
      nav.enabled = False
      # The full stalk is classified and fresh, but positive turn-floor
      # evidence has not qualified yet. Opposite steering is still forbidden.
      self.d.update(manual_turn_direction=direction, source='none',
                    final_desire='none', received_mono=self.now,
                    frame_id=self.model.frameId)
      nav.decision = copy.deepcopy(self.d)
      boosted = nav.update(-sign*.01, self.cs, True, self.now,
                           model=self.model, healthy=True,
                           actual_curvature=0.)
      self.assertEqual(boosted, 0.)
      self.assertEqual(nav.state, 'MANUAL VETO')
      # Matching curvature is never weakened by the early boost.
      nav.decision = copy.deepcopy(self.d)
      self.assertEqual(nav.update(sign*.01, self.cs, True, self.now+.01,
                                  model=self.model, healthy=True,
                                  actual_curvature=0.), sign*.01)

      # Waiting intent remains a continuous direction veto, without a
      # positive blinker-side seed until sustained corner geometry qualifies.
      self.model.position.y = [0.] * 4
      self.model.orientation.z = [0.] * 4
      self.model.action.desiredCurvature = 0.
      nav.decision = copy.deepcopy(self.d)
      self.assertEqual(nav.update(-sign*.01, self.cs, True, self.now+.02,
                                  model=self.model, healthy=True,
                                  actual_curvature=0.), 0.)
      self.assertEqual(nav.state, 'MANUAL VETO')

  def test_navigation_wrapper_manual_independent_of_navigation_toggle(self):
    with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
      nav = NavigationTurnCommit()
    nav._refresh = lambda _: None
    nav.enabled = False
    for _ in range(30):
      self.tick()
      nav.decision = copy.deepcopy(self.d)
      out = nav.update(self.curvature, self.cs, True, self.now, model=self.model, healthy=True, actual_curvature=self.actual)
    self.assertEqual(out, self.curvature)
    self.assertEqual(nav.state, 'MANUAL MODEL PATH')

  def test_seed_does_not_wait_for_model_desire_selection(self):
    self.d.update(source='none', final_desire='none')
    self.curvature = .02
    self.model.action.desiredCurvature = .02
    self.assertLess(self.start(), 0.)

  def test_completed_turn_cannot_reboost_while_signal_remains_on(self):
    with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
      nav = NavigationTurnCommit()
    nav._refresh = lambda _: None
    nav.manual.blocked = True
    for i in range(20):
      self.d.update(received_mono=self.now, frame_id=self.model.frameId)
      nav.decision = copy.deepcopy(self.d)
      self.assertEqual(nav.update(0., self.cs, True, self.now, model=self.model,
                                  healthy=True, actual_curvature=0.), 0.)
      self.now += .05
      self.model.frameId += 1
      self.assertFalse(nav.active)

  def test_navigation_floor_and_driver_handoff(self):
    with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
      nav = NavigationTurnCommit()
    nav._refresh = lambda _: None
    nav.enabled = True
    nav.decision = dict(desire='turnLeft', confirmed=True, distance_m=6., modifier='left')
    self.assertEqual(nav.update(0., self.cs, True, 100.), -.055)
    self.cs.steeringPressed = True
    self.assertEqual(nav.update(0., self.cs, True, 100.1), 0.)
    self.cs.steeringPressed = False
    self.assertEqual(nav.update(0., self.cs, True, 100.2), 0.)
    self.assertLess(nav.update(0., self.cs, True, 100.6), 0.)


class StalkAndDiagnostics(unittest.TestCase):
  def test_physical_hold_not_tap_synthetic_stale_or_startup(self):
    with tempfile.TemporaryDirectory() as tmp:
      producer = TapController(Path(tmp)/'request', Path(tmp)/'ack')
      producer.params = N(get_bool=lambda _: False)
      with patch('openpilot.selfdrive.controls.lib.desire_helper.Params'):
        consumer = TapModel(DesireHelper(), Path(tmp)/'request', Path(tmp)/'ack')
      stalk = PhysicalStalk()
      cs = N(vEgo=4., canValid=True, leftBlinker=True, rightBlinker=False, gearShifter='drive')
      now = 100.
      def tick(direction, bus=0, physical=True):
        nonlocal now
        now += .05
        raw = bytearray(8)
        raw[2] = direction
        raw[7] = crc8(raw[:7])
        if physical: stalk.feed([(int((now - 1e-6)*1e9), [(0x45, bytes(raw), bus)])])
        producer.update(stalk, cs, enabled=False, lateral_active=True, overriding=False, now=now)
        consumer.update(cs, N(latActive=True), True, False, 1., now=now)
        return consumer.manual_turn_direction
      for _ in range(20): self.assertEqual(tick(1), 0)  # startup held
      tick(0)
      for _ in range(5): self.assertEqual(tick(1), 0)  # tap
      self.assertEqual(tick(0), 0)
      for _ in range(14): held = tick(1)
      self.assertEqual(held, 1)
      for _ in range(5): held = tick(1, physical=False)
      self.assertEqual(held, 0)
      tick(0)
      for _ in range(14): self.assertEqual(tick(2, bus=128), 0)
      tick(0)
      for _ in range(14): held = tick(2)
      self.assertEqual(held, 2)
      self.assertEqual(tick(0), 0)

  def test_armed_waiting_intent_is_published_at_control_freshness(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      nav = NavigationDesire(str(root/'nav'), str(root/'diag'), str(root/'signal'))
      nav.decision = dict(received_mono=100.)
      nav.record_model_selection('none', 'none', True, 42, signal_owner='driver',
                                 manual_turn_direction=1, city_turns_enabled=True,
                                 manual_turn_blocked=False)
      for i in range(10):
        now = 100. + i*.1
        nav.decision['received_mono'] = now
        nav.publish_diagnostics(now)
        import json
        written = json.loads((root/'diag').read_text())
        self.assertEqual(written['received_mono'], now)
        self.assertFalse(written['manual_turn_blocked'])

  def test_diagnostics_cadence_and_selection(self):
    with tempfile.TemporaryDirectory() as tmp:
      root = Path(tmp)
      nav = NavigationDesire(str(root/'nav'), str(root/'diag'), str(root/'signal'))
      nav.decision = dict(received_mono=100.)
      nav.record_model_selection('turnLeft', 'driver_desire_helper', True, 42,
                                 signal_owner='driver', manual_turn_direction=1, city_turns_enabled=True)
      nav.publish_diagnostics(100.)
      nav.decision['received_mono'] = 100.1
      nav.publish_diagnostics(100.1)
      self.assertIn('100.1', (root/'diag').read_text())
      self.assertTrue(nav.decision['city_turns_enabled'])
      self.assertEqual(nav.decision['manual_turn_direction'], 1)

      nav.decision.update(source='navigation', manual_turn_direction=0, received_mono=100.2)
      nav.publish_diagnostics(100.2)  # Promptly revoke the preceding manual request.
      self.assertIn('100.2', (root/'diag').read_text())
      nav.decision['received_mono'] = 100.3
      nav.publish_diagnostics(100.3)
      self.assertIn('100.2', (root/'diag').read_text())  # Existing nav cadence preserved.
