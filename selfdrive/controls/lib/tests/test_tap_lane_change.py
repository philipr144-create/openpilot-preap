"""Offline integration tests using real lane-change enums/helper, no CAN publisher."""

import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

from cereal import log
from opendbc.car.tesla.preap import tap_lane_change as core
from openpilot.selfdrive.controls.lib import desire_helper as helper
from openpilot.selfdrive.controls.lib import tap_lane_change as model
from openpilot.selfdrive.modeld import navigation_desire as nav


def raw(direction=0, counter=0, cruise=0):
  # Exercise otherwise unmodelled bit 51 and unrelated switch preservation.
  out = bytearray(
    [0x40 | cruise, 66, direction | 0x10, 0, 0x80, 1, 0x0B | (counter << 4), 0]
  )
  out[7] = core.crc8(out[:7])
  return bytes(out)


def cs():
  return NS(
    vEgo=25.0,
    canValid=True,
    leftBlinker=True,
    rightBlinker=False,
    steeringPressed=False,
    steeringDisengage=False,
    steeringTorque=0.0,
    leftBlindspot=False,
    rightBlindspot=False,
    gearShifter="drive",
    brakePressed=False,
  )


class Simulation(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    req = Path(self.temp.name) / "request"
    ack = Path(self.temp.name) / "ack"
    self.ctrl = core.TapController(req, ack)
    self.stalk = core.PhysicalStalk()
    self.dh = helper.DesireHelper()
    self.model = model.TapModel(self.dh, req, ack)
    self.cs = cs()
    self.cc = NS(latActive=True)
    self.now = 1.0
    self.counter = 0
    self.enabled = True
    self.overriding = False
    self.claim = False
    self.tick()
    self.tick()

  def tick(
    self, direction=0, dt=0.02, feedback=True, physical=True, existing=(), prob=1.0
  ):
    self.now += dt
    if physical:
      self.stalk.feed(
        [(round(self.now * 1e9), [(0x45, raw(direction, self.counter), 0)])]
      )
      self.counter = (self.counter + 1) % 16
    sends = self.ctrl.update(
      self.stalk,
      self.cs,
      enabled=self.enabled,
      lateral_active=self.cc.latActive,
      overriding=self.overriding,
      existing=existing,
      now=self.now,
    )
    if feedback:
      self.model.update(self.cs, self.cc, True, self.claim, prob, now=self.now)
    return sends

  def tap(self, direction=1):
    for _ in range(8):
      self.tick(direction)
    self.tick()

  def active(self):
    self.tap()
    sends = self.tick()
    self.assertEqual(self.ctrl.phase, "active")
    self.assertEqual(self.model.status, "starting")
    self.assertEqual(sends[0][1][2] & 3, 1)

  def test_no_nudge_or_post_release_delay(self):
    self.tap()
    self.assertEqual(self.model.status, "starting")
    self.assertEqual(self.dh.desire, log.Desire.laneChangeLeft)

  def test_right(self):
    self.tap(2)
    self.assertEqual(self.dh.desire, log.Desire.laneChangeRight)

  def test_default_off(self):
    self.enabled = False
    self.tap()
    self.assertEqual(self.ctrl.request_id, 0)
    self.assertFalse(self.ctrl.suppress)
    self.assertEqual(self.tick(), [])

  def test_enabled_idle_is_not_signal_owner(self):
    self.tick()
    self.assertTrue(self.model.enabled)
    self.assertFalse(self.model.signal_active)
    self.assertFalse(self.model.locked)

  def test_exactly_40_blocked(self):
    self.cs.vEgo = core.SPEED_MIN
    self.tap()
    self.assertEqual(self.ctrl.request_id, 0)

  def test_below_speed_blocked_not_deferred(self):
    self.cs.vEgo = 10
    self.tap()
    self.cs.vEgo = 25
    for _ in range(20):
      self.tick()
    self.assertEqual(self.ctrl.request_id, 0)

  def test_hold_unchanged(self):
    for _ in range(60):
      self.tick(1)
    self.tick()
    self.assertEqual(self.ctrl.request_id, 0)
    self.assertFalse(self.ctrl.suppress)

  def test_blindspot_wait_and_release(self):
    self.cs.leftBlindspot = True
    self.tap()
    self.tick()
    self.assertEqual(self.model.status, "waiting")
    self.assertEqual(self.dh.desire, log.Desire.none)
    self.cs.leftBlindspot = False
    self.tick()
    self.assertEqual(self.model.status, "starting")

  def test_wait_timeout_consumed(self):
    self.cs.leftBlindspot = True
    self.tap()
    for _ in range(520):
      self.tick()
    self.cs.leftBlindspot = False
    self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")
    self.assertEqual(self.dh.desire, log.Desire.none)

  def test_complete_cancels_signal_once_and_no_repeat(self):
    self.active()
    sent = []
    for _ in range(90):
      sent.extend(self.tick(prob=0))
    self.assertEqual(self.ctrl.phase, "complete")
    self.assertEqual([m[1][2] & 3 for m in sent][-3:], [0, 1, 0])
    for _ in range(100):
      self.assertEqual(self.tick(prob=0), [])
    self.assertEqual(self.dh.desire, log.Desire.none)
    self.assertTrue(self.ctrl.suppress)

    # The replay latch may remain set, but completed tap signaling must release
    # model ownership after cleanup.
    handled = self.model.update(self.cs, self.cc, True, False, 0, now=self.now)
    self.assertFalse(handled)
    self.assertFalse(self.model.signal_active)

  def test_second_tap_cancels_no_restart(self):
    self.active()
    self.tap()
    for _ in range(50):
      self.assertEqual(self.tick(), [])
    self.assertEqual(self.ctrl.phase, "cancelled")
    self.assertEqual(self.ctrl.request_id, 1)
    self.assertEqual(self.dh.desire, log.Desire.none)
    self.tap()
    self.assertEqual(self.ctrl.request_id, 3)
    self.assertEqual(self.model.status, "starting")

  def test_opposite_cancels_without_commands(self):
    self.active()
    for _ in range(8):
      self.assertEqual(self.tick(2), [])
    self.tick()
    self.assertEqual(self.ctrl.request_id, 1)

  def test_hso_cancel_even_when_lat_active(self):
    self.active()
    self.overriding = True
    self.tick()
    self.overriding = False
    for _ in range(10):
      self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")
    self.assertEqual(self.dh.desire, log.Desire.none)

  def test_disengage(self):
    self.active()
    self.cc.latActive = False
    self.tick()
    self.cc.latActive = True
    self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_toggle_off_active_cleanup(self):
    self.active()
    self.enabled = False
    sends = []
    for _ in range(8):
      sends.extend(self.tick())
    self.assertEqual([msg[1][2] & 3 for msg in sends], [0, 1, 0])
    self.assertEqual(self.tick(), [])

  def test_stale_model(self):
    self.active()
    for _ in range(20):
      self.tick(feedback=False)
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_model_restart_no_replay(self):
    self.active()
    self.model = model.TapModel(self.dh, self.ctrl.request.path, self.ctrl.ack.path)
    self.tick()
    self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")
    self.assertEqual(self.dh.desire, log.Desire.none)

  def test_controller_restart_no_replay(self):
    self.active()
    self.ctrl = core.TapController(self.ctrl.request.path, self.ctrl.ack.path)
    self.tick()
    self.assertEqual(self.ctrl.request_id, 0)
    self.assertEqual(self.dh.desire, log.Desire.none)

  def test_invalid_can_no_transmit(self):
    self.active()
    self.cs.canValid = False
    self.assertEqual(self.tick(), [])
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_stale_physical_no_transmit(self):
    self.active()
    self.assertEqual(self.tick(dt=0.2, physical=False), [])
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_stale_gesture_not_tap(self):
    self.tick(1)
    self.tick(dt=0.3)
    self.assertEqual(self.ctrl.request_id, 0)

  def test_navigation_claim_consumes_tap(self):
    self.claim = True
    self.tap()
    self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")
    self.claim = False
    self.tick()
    self.assertEqual(self.dh.desire, log.Desire.none)

  def test_navigation_new_claim_cancels_active(self):
    self.active()
    self.claim = True
    self.tick()
    self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_cruise_transmission_priority(self):
    self.active()
    self.assertEqual(self.tick(existing=[(0x45, raw(), 0)]), [])
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_speed_drop_no_restart(self):
    self.active()
    self.cs.vEgo = core.SPEED_MIN
    self.tick()
    self.cs.vEgo = 25
    self.tick()
    self.assertEqual(self.ctrl.phase, "cancelled")

  def test_hazards(self):
    self.cs.rightBlinker = True
    self.tap()
    self.assertEqual(self.ctrl.request_id, 0)

  def test_nan(self):
    self.cs.vEgo = math.nan
    self.tap()
    self.assertEqual(self.ctrl.request_id, 0)


class PhysicalTests(unittest.TestCase):
  def test_all_edges_in_one_batch(self):
    s = core.PhysicalStalk()
    s.feed(
      [
        (1000000000, [(0x45, raw(), 0)]),
        (1100000000, [(0x45, raw(1, 1), 0)]),
        (1200000000, [(0x45, raw(0, 2), 0)]),
      ]
    )
    self.assertEqual([e[0] for e in s.events], ["invalid", "press", "tap"])

  def test_startup_held_never_tap(self):
    s = core.PhysicalStalk()
    s.feed([(1000000000, [(0x45, raw(1), 0)]), (1100000000, [(0x45, raw(), 0)])])
    self.assertNotIn("tap", [e[0] for e in s.events])

  def test_tx_receipts_no_events_or_freshness(self):
    s = core.PhysicalStalk()
    s.feed([(1000000000, [(0x45, raw(1), 128), (0x45, raw(), 192)])])
    self.assertIsNone(s.raw)
    self.assertFalse(s.events)

  def test_rx_reflection_filtered(self):
    s = core.PhysicalStalk()
    frame = (0x45, raw(1), 0)
    s.remember_tx(frame, 1)
    s.feed([(1010000000, [frame])])
    self.assertIsNone(s.raw)

  def test_bad_crc_no_tap(self):
    s = core.PhysicalStalk()
    s.feed([(1000000000, [(0x45, raw(), 0)])])
    bad = bytearray(raw(1))
    bad[7] ^= 1
    s.feed([(1100000000, [(0x45, bad, 0)])])
    self.assertIsNone(s.raw)

  def test_signal_bit_preservation(self):
    source = raw(counter=15)
    for direction in (0, 1, 2):
      _, data, bus = core.signal_frame(source, direction)
      self.assertEqual(bus, 0)
      self.assertEqual(data[2] & 3, direction)
      self.assertEqual(data[:2], source[:2])
      self.assertEqual(data[3:6], source[3:6])
      self.assertEqual(data[2] & 252, source[2] & 252)
      self.assertEqual(data[6] & 15, source[6] & 15)
      self.assertEqual(data[6] >> 4, 0)
      self.assertEqual(data[7], core.crc8(data[:7]))

  def test_physical_cruise_and_signal_not_overwritten(self):
    for source in (raw(1), raw(cruise=1)):
      with self.assertRaises(ValueError):
        core.signal_frame(source, 2)


class NavigationTests(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.path = Path(self.temp.name) / "nav"
    self.signal_path = Path(self.temp.name) / "signal"
    self.nav = nav.NavigationDesire(self.path, Path(self.temp.name) / "decision", self.signal_path)
    self.nav.params = NS(get_bool=lambda _: True)

  def state(self, kind="fork", distance=100, enabled=True, active=True):
    self.path.write_text(
      json.dumps(
        {
          "enabled": enabled,
          "route_active": active,
          "received_mono": 1.0,
          "expires_mono": 3.0,
          "route_state": "active",
          "route_id": "r",
          "maneuver_id": "m",
          "maneuver": {"type": kind, "modifier": "left", "distance_m": distance},
        }
      )
    )

  def test_windows(self):
    for kind, distance, claim in [
      ("fork", 100, True),
      ("off ramp", 300, True),
      ("fork", 301, False),
      ("turn", 80, True),
      ("turn", 81, False),
      ("end of road", -8, True),
    ]:
      self.state(kind, distance)
      self.assertEqual(self.nav.claims_tap(now=2), claim)

  def test_disabled_or_no_route(self):
    self.state(enabled=False)
    self.assertFalse(self.nav.claims_tap(now=2))
    self.state(active=False, enabled=True)
    self.assertFalse(self.nav.claims_tap(now=2))

  def test_navigation_toggle_off_never_claims_tap(self):
    self.state(kind="fork", distance=100)
    self.nav.params = NS(get_bool=lambda _: False)
    self.assertFalse(self.nav.claims_tap(now=2, prepare=True))
    self.assertFalse(self.nav.owns_blinker)
    self.assertEqual(self.nav.update(cs(), NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertEqual(self.nav.decision["reason"], "Disabled in NAP settings")

  def test_stale_active_route_blocks(self):
    self.state()
    self.assertTrue(self.nav.claims_tap(now=10))

  def test_unknown_blocks(self):
    self.assertTrue(self.nav.claims_tap(now=2))

  def test_owned_tap_does_not_confirm_or_block_route(self):
    self.state(distance=500)
    c = cs()
    c.leftBlinker = False
    c.rightBlinker = True
    self.assertEqual(
      self.nav.update(c, NS(latActive=True), True, now=2, signal_owned_by_tap=True),
      "none",
    )
    self.assertFalse(self.nav.blocked)
    self.assertFalse(self.nav.confirmed)

  def test_navigation_authorizes_and_signals_turn_without_physical_stalk(self):
    self.state(kind="turn", distance=10)
    c = cs()
    c.vEgo = 5
    # The route authorizes the maneuver; the neutral physical stalk leaves
    # signaling to the synthetic navigation controller.
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "turnLeft")
    self.assertTrue(self.nav.confirmed)
    self.assertEqual(self.nav.indicator_request, "left")

  def test_turn_signal_starts_on_approach_before_low_speed_desire(self):
    self.state(kind="turn", distance=45)
    c = cs()
    c.vEgo = 12.0  # Above the 25 mph model-turn limit.
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertEqual(self.nav.indicator_request, "left")

  def test_turn_never_begins_signaling_at_zero_distance(self):
    self.state(kind="turn", distance=0)
    c = cs()
    c.vEgo = 5
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertEqual(self.nav.indicator_request, "none")

  def test_steering_override_pauses_turn_desire_not_announcement(self):
    self.state(kind="turn", distance=25)
    c = cs()
    c.vEgo = 5
    c.steeringPressed = True
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertEqual(self.nav.indicator_request, "left")

  def test_city_confirmation_latches_but_signals_near_turn(self):
    self.state(kind="turn", distance=70)
    c = cs()
    c.vEgo = 5
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertTrue(self.nav.confirmed)
    self.assertEqual(self.nav.indicator_request, "none")

    self.state(kind="turn", distance=10)
    c.leftBlinker = False
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2.05,
                                     physical_direction=0), "turnLeft")
    self.assertEqual(self.nav.indicator_request, "left")
    signal = json.loads(self.signal_path.read_text())
    self.assertTrue(signal["active"])
    self.assertEqual(signal["direction"], 1)

  def test_exit_guidance_starts_early_but_signal_is_just_in_time(self):
    self.state(kind="fork", distance=200)
    c = cs()
    c.vEgo = 30
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "keepLeft")
    self.assertTrue(self.nav.confirmed)
    self.assertEqual(self.nav.indicator_request, "none")

    self.state(kind="fork", distance=110)
    c.leftBlinker = False
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2.05,
                                     physical_direction=0), "keepLeft")
    self.assertEqual(self.nav.indicator_request, "left")

  def test_steering_before_exit_pauses_without_poisoning_maneuver(self):
    self.state(kind="fork", distance=200)
    c = cs()
    c.vEgo = 20
    c.steeringPressed = True
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertFalse(self.nav.blocked)
    self.assertEqual(self.nav.indicator_request, "none")

    c.steeringPressed = False
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2.05,
                                     physical_direction=0), "keepLeft")
    self.assertTrue(self.nav.confirmed)

  def test_steering_override_does_not_delay_exit_signal(self):
    self.state(kind="off ramp", distance=80)
    c = cs()
    c.vEgo = 20
    c.steeringPressed = True
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertEqual(self.nav.indicator_request, "left")

  def test_braking_does_not_delay_exit_signal(self):
    self.state(kind="off ramp", distance=80)
    c = cs()
    c.vEgo = 20
    c.brakePressed = True
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "none")
    self.assertEqual(self.nav.indicator_request, "left")

  def test_exit_control_gate_is_a_pause_not_permanent_cancel(self):
    self.state(kind="fork", distance=70)
    c = cs()
    c.vEgo = 20
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=0), "keepLeft")
    self.assertEqual(self.nav.indicator_request, "left")

    self.assertEqual(self.nav.update(c, NS(latActive=False), True, now=2.05,
                                     physical_direction=0), "none")
    self.assertFalse(self.nav.blocked)
    self.assertTrue(self.nav.confirmed)

    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2.1,
                                     physical_direction=0), "keepLeft")
    self.assertEqual(self.nav.indicator_request, "left")

  def test_opposite_blinker_is_temporary_priority_not_route_cancellation(self):
    self.state(kind="fork", distance=70)
    c = cs()
    c.vEgo = 20
    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2,
                                     physical_direction=2), "none")
    self.assertFalse(self.nav.blocked)
    self.assertEqual(self.nav.indicator_request, "none")

    self.assertEqual(self.nav.update(c, NS(latActive=True), True, now=2.05,
                                     physical_direction=0), "keepLeft")
    self.assertEqual(self.nav.indicator_request, "left")


class NavigationSignalTests(unittest.TestCase):
  def setUp(self):
    self.temp = tempfile.TemporaryDirectory()
    self.addCleanup(self.temp.cleanup)
    self.request_path = Path(self.temp.name) / "request"
    self.status_path = Path(self.temp.name) / "status"
    self.request = core.Snapshot(self.request_path)
    self.ctrl = core.NavigationSignalController(self.request_path, self.status_path)
    self.stalk = core.PhysicalStalk()
    self.cs = NS(canValid=True, vEgo=25.0, leftBlinker=False,
                 rightBlinker=False, gearShifter="drive")
    self.now = 1.0
    self.counter = 0

  def feed(self, direction=0):
    self.stalk.feed([(round(self.now * 1e9), [(0x45, raw(direction, self.counter), 0)])])
    self.counter = (self.counter + 1) % 16

  def publish(self, active=True, direction=1, maneuver_id="m"):
    self.request.write({"active": active, "direction": direction,
                        "maneuver_id": maneuver_id}, self.now)

  def tick(self, direction=0, active=True, request_direction=1, existing=()):
    self.now += .02
    self.feed(direction)
    self.publish(active, request_direction)
    return self.ctrl.update(self.stalk, self.cs, lateral_active=True,
                            overriding=False, existing=existing, now=self.now)

  def test_navigation_signal_uses_existing_frame_builder(self):
    sends = self.tick(request_direction=2)
    self.assertEqual([msg[1][2] & 3 for msg in sends], [2])
    self.assertTrue(self.ctrl.held_sent)

  def test_physical_stalk_has_priority(self):
    self.tick(request_direction=2)
    sends = self.tick(direction=1, request_direction=2)
    self.assertEqual(sends, [])
    self.assertEqual(self.ctrl.direction, 0)
    self.assertFalse(self.ctrl.cleanup)

  def test_hands_on_steering_does_not_delay_navigation_signal(self):
    self.now += .02
    self.feed(0)
    self.publish(active=True, direction=2)
    sends = self.ctrl.update(self.stalk, self.cs, lateral_active=True,
                             overriding=True, now=self.now)
    self.assertEqual([msg[1][2] & 3 for msg in sends], [2])

  def test_navigation_release_uses_bounded_cleanup(self):
    self.tick(request_direction=1)
    directions = []
    for _ in range(8):
      directions.extend(msg[1][2] & 3 for msg in self.tick(active=False))
      if not self.ctrl.cleanup:
        break
    self.assertEqual(directions, [0, 1, 0])

  def test_existing_stw_sender_wins_slot(self):
    existing = [(0x45, raw(), 0)]
    self.assertEqual(self.tick(existing=existing), [])

  def test_navigation_preempts_active_tap_signal(self):
    tap_request = Path(self.temp.name) / "tap-request"
    tap_ack = Path(self.temp.name) / "tap-ack"
    tap = core.TapController(tap_request, tap_ack, session="tap-controller")
    tap.phase = "active"
    tap.direction = 1
    tap.started = self.now
    tap.model = "tap-model"
    tap.request_id = 7
    core.Snapshot(tap_ack).write({"controller": tap.session, "model": tap.model,
                                 "id": tap.request_id, "status": "starting"}, self.now)

    nav_sends = self.tick(request_direction=2)
    tap_sends = tap.update(self.stalk, self.cs, enabled=True, lateral_active=True,
                           overriding=False, existing=nav_sends, now=self.now)
    self.assertEqual([msg[1][2] & 3 for msg in nav_sends], [2])
    self.assertEqual(tap_sends, [])
    self.assertEqual(tap.phase, "cancelled")


if __name__ == "__main__":
  unittest.main(verbosity=2)
