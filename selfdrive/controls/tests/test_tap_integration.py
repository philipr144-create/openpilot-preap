"""Isolated tests of the installed tap producer/consumer; never transmit CAN."""
import math
import os
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

from cereal import log
from opendbc.car.tesla.preap import tap_lane_change as producer
from openpilot.selfdrive.controls.lib import tap_lane_change as consumer
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper

MPH = .44704
EXPECTED = float(os.environ.get('EXPECTED_MPH', '30'))


class TapIntegration(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    root = Path(self.tmp.name)
    self.car = producer.TapController(root/'request', root/'ack', session='test')
    self.car.params = N(get_bool=lambda _: True)
    with patch('openpilot.selfdrive.controls.lib.desire_helper.Params'):
      self.helper = DesireHelper()
    self.model = consumer.TapModel(self.helper, root/'request', root/'ack')
    self.cs = N(vEgo=(EXPECTED+1)*MPH, canValid=True, leftBlinker=False,
                rightBlinker=False, leftBlindspot=False, rightBlindspot=False,
                gearShifter='drive', steeringPressed=False, steeringDisengage=False)
    self.cc = N(latActive=True)
    self.stalk = producer.PhysicalStalk()
    self.now = 100.
    self.enabled, self.override, self.valid, self.nav = True, False, True, False
    self.tick()
    self.tick()

  def tick(self, prob=1., events=()):
    self.now += .05
    raw = bytearray(8)
    raw[7] = producer.crc8(raw[:7])
    self.stalk.raw, self.stalk.direction, self.stalk.time = bytes(raw), 0, self.now
    for kind, direction in events:
      if kind == 'press':
        self.stalk.gesture += 1
      self.stalk.events.append((kind, self.stalk.gesture, direction, self.now))
    sends = self.car.update(self.stalk, self.cs, enabled=self.enabled,
                            lateral_active=self.cc.latActive, overriding=self.override, now=self.now)
    self.model.update(self.cs, self.cc, self.valid, self.nav, prob, now=self.now)
    return sends

  def tap(self, direction=1):
    self.tick(events=(('press', direction), ('tap', direction)))

  def test_shared_threshold(self):
    self.assertEqual(producer.SPEED_MIN, EXPECTED*MPH)
    self.assertEqual(consumer.SPEED_MIN, producer.SPEED_MIN)

  def test_speed_boundaries(self):
    for mph in (EXPECTED-.01, EXPECTED, EXPECTED+.01, EXPECTED+5, 45):
      with self.subTest(mph=mph):
        self.cs.vEgo = mph*MPH
        self.tap()
        self.assertEqual(self.model.status == 'starting', mph > EXPECTED)
        self.override = True
        self.tick()
        self.override = False
        for _ in range(5): self.tick()

  def test_both_directions_finish_once(self):
    for direction, desire in ((1, log.Desire.laneChangeLeft), (2, log.Desire.laneChangeRight)):
      with self.subTest(direction=direction):
        self.tap(direction)
        self.assertEqual(self.helper.desire, desire)
        for _ in range(45): self.tick(prob=0.)
        self.assertEqual(self.car.phase, 'complete')
        self.assertEqual(self.helper.desire, log.Desire.none)
        old_id = self.car.request_id
        for _ in range(10): self.tick(prob=0.)
        self.assertEqual(self.car.request_id, old_id)
        self.assertEqual(self.car.phase, 'complete')

  def test_blindspot_waits_before_start(self):
    self.cs.leftBlindspot = True
    self.tap()
    self.assertEqual(self.model.status, 'waiting')
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.cs.leftBlindspot = False
    self.tick()
    self.assertEqual(self.model.status, 'starting')

  def test_takeover_cancels_without_replay(self):
    self.tap()
    self.cs.steeringPressed = True
    self.tick()
    self.assertEqual(self.model.status, 'cancelled')
    self.assertEqual(self.helper.desire, log.Desire.none)
    self.cs.steeringPressed = False
    for _ in range(10): self.tick()
    self.assertNotIn(self.model.status, ('waiting', 'starting', 'finishing'))

  def test_gates_cancel_active_maneuver(self):
    cases = [('override', True), ('enabled', False), ('valid', False), ('nav', True),
             ('speed', EXPECTED*MPH), ('nan', math.nan), ('lat', False),
             ('can', False), ('gear', 'park'), ('disengage', True)]
    for name, value in cases:
      with self.subTest(gate=name):
        self.setUp()
        self.tap()
        self.assertEqual(self.model.status, 'starting')
        if name in ('override', 'enabled', 'valid', 'nav'): setattr(self, name, value)
        elif name in ('speed', 'nan'): self.cs.vEgo = value
        elif name == 'lat': self.cc.latActive = value
        elif name == 'can': self.cs.canValid = value
        elif name == 'gear': self.cs.gearShifter = value
        else: self.cs.steeringDisengage = value
        self.tick()
        self.assertEqual(self.helper.desire, log.Desire.none)
        self.assertNotIn(self.model.status, ('waiting', 'starting', 'finishing'))

  def test_navigation_blocks_new_tap(self):
    self.nav = True
    self.tap()
    self.assertEqual(self.model.status, 'blocked')
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_stale_model_ack_cancels(self):
    self.tap()
    self.now += 1.
    self.tick()
    self.assertEqual(self.car.phase, 'cancelled')
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_stale_request_cancels(self):
    self.tap()
    self.model.update(self.cs, self.cc, True, False, 1., now=self.now+1.)
    self.assertEqual(self.model.status, 'cancelled')
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_model_restart_does_not_replay(self):
    self.tap()
    restarted = consumer.TapModel(self.helper, self.car.request.path, self.car.ack.path)
    restarted.update(self.cs, self.cc, True, False, 1., now=self.now)
    self.assertEqual(restarted.status, 'blocked')
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_timeout(self):
    self.tap()
    for _ in range(210): self.tick()
    self.assertEqual(self.car.phase, 'cancelled')
    self.assertEqual(self.helper.desire, log.Desire.none)

  def test_physical_stalk_cancels_active_tap(self):
    self.tap()
    self.tick(events=(('press', 2),))
    self.assertEqual(self.car.phase, 'cancelled')
    self.assertEqual(self.helper.desire, log.Desire.none)


if __name__ == '__main__': unittest.main(verbosity=2)
