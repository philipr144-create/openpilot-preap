"""Model-side consumer of one-shot physical Pre-AP tap requests."""

import math
import time
import uuid

from opendbc.car.tesla.preap.tap_lane_change import (
  ACK_PATH,
  REQUEST_PATH,
  SPEED_MIN,
  Snapshot,
)
from openpilot.selfdrive.controls.lib.desire_helper import LaneChangeState


class TapModel:
  def __init__(self, helper, request_path=REQUEST_PATH, ack_path=ACK_PATH):
    self.helper = helper
    self.request = Snapshot(request_path)
    self.ack = Snapshot(ack_path)
    self.model = uuid.uuid4().hex
    self.controller = None
    self.seen = 0
    self.status = "idle"
    self.locked = False
    self.enabled = False
    self.phase = "idle"
    self.signal_active = False
    self.physical_direction = 0
    self.request_valid = False
    self.rearm_gesture = None

  def update(self, cs, cc, valid, nav_claim, lane_change_prob, now=None):
    now = time.monotonic() if now is None else now
    maneuver_was_active = self.status in ("waiting", "starting", "finishing")
    request = self.request.read(now)
    if request is None:
      if self.status in ("waiting", "starting", "finishing"):
        self.locked = True
      if self.locked:
        self.helper.suspend_for_navigation(cs)
      self.status, self.enabled = "cancelled", False
      self.phase, self.signal_active = "cancelled", False
      self.physical_direction, self.request_valid = 0, False
      self._publish(now)
      return self.locked
    controller, seq = request.get("controller"), request.get("id")
    if (
      not isinstance(controller, str)
      or not controller
      or type(seq) is not int
      or seq < 0
    ):
      self.helper.suspend_for_navigation(cs)
      self.locked, self.status = True, "cancelled"
      self.enabled, self.signal_active = False, False
      self.phase, self.physical_direction = "invalid", 0
      self.request_valid = False
      self._publish(now)
      return True
    self.request_valid = True
    self.enabled = request.get("enabled") is True
    self.phase = request.get("phase") if isinstance(request.get("phase"), str) else "invalid"
    self.physical_direction = request.get("physical_direction") if request.get("physical_direction") in (0, 1, 2) else 0
    self.signal_active = request.get("signal_active") is True
    active = self.phase in ("pending", "active")
    if controller != self.controller:
      # Baseline on attachment: never replay a request surviving a process restart.
      self.controller, self.seen = controller, seq
      self.status = "blocked" if active else "idle"
      if self.locked:
        self.rearm_gesture = request.get("gesture")
      self.locked = self.locked or active or self.signal_active
      if self.locked:
        self.helper.suspend_for_navigation(cs)
      self._publish(now)
      return self.locked
    if seq != self.seen:
      self.seen = seq  # Consume even a blocked request; never defer authorization.
      self.status = "blocked"
      if (
        active
        and self.enabled
        and valid
        and cc.latActive
        and not nav_claim
        and math.isfinite(cs.vEgo)
        and cs.vEgo > SPEED_MIN
        and not cs.steeringPressed
        and not getattr(cs, "steeringDisengage", False)
        and request.get("direction") in (1, 2)
        and self.helper.lane_change_state
        in (LaneChangeState.off, LaneChangeState.preLaneChange)
      ):
        self.helper.suspend_for_navigation(cs)
        self.status = "waiting"
    if self.rearm_gesture is not None and self.rearm_gesture != request.get("gesture"):
      self.rearm_gesture = None
    # `suppress` remains latched after a consumed gesture to prevent replay;
    # it is not proof that tap still owns the signal or DesireHelper. Ownership
    # lasts only while the request/cleanup is active (or restart rearm is live).
    handled = active or self.signal_active or self.rearm_gesture is not None
    if self.status in ("waiting", "starting", "finishing"):
      if (
        not active
        or not self.enabled
        or not valid
        or not cc.latActive
        or nav_claim
        or not math.isfinite(cs.vEgo)
        or cs.vEgo <= SPEED_MIN
        or cs.steeringPressed
        or getattr(cs, "steeringDisengage", False)
      ):
        self.status = "cancelled"
      else:
        self.status = self.helper.update_tap(cs, lane_change_prob, request["direction"])
    ended_this_frame = maneuver_was_active and self.status in ("complete", "cancelled", "blocked")
    if (handled or ended_this_frame) and self.status not in ("waiting", "starting", "finishing"):
      self.helper.suspend_for_navigation(cs)
    self.locked = handled
    self._publish(now)
    return handled or ended_this_frame

  def _publish(self, now):
    self.ack.write(
      {
        "controller": self.controller,
        "model": self.model,
        "id": self.seen,
        "status": self.status,
      },
      now,
    )
