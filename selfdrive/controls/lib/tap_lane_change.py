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
    self.rearm_gesture = None

  def update(self, cs, cc, valid, nav_claim, lane_change_prob, now=None):
    now = time.monotonic() if now is None else now
    request = self.request.read(now)
    if request is None:
      if self.status in ("waiting", "starting", "finishing"):
        self.locked = True
      if self.locked:
        self.helper.suspend_for_navigation(cs)
      self.status, self.enabled = "cancelled", False
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
      self._publish(now)
      return True
    self.enabled = request.get("enabled") is True
    active = request.get("phase") in ("pending", "active")
    if controller != self.controller:
      # Baseline on attachment: never replay a request surviving a process restart.
      self.controller, self.seen = controller, seq
      self.status = "blocked" if active else "idle"
      if self.locked:
        self.rearm_gesture = request.get("gesture")
      self.locked = self.locked or active or request.get("suppress") is True
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
    handled = (
      request.get("suppress") is True or active or self.rearm_gesture is not None
    )
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
    if handled and self.status not in ("waiting", "starting", "finishing"):
      self.helper.suspend_for_navigation(cs)
    self.locked = handled
    self._publish(now)
    return handled

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
