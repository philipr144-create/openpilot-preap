"""Bounded navigation turn commitment for NAP Pre-AP.

This does NOT directly command steering torque or steering angle.

When NavigationDesire has already authorized an intersection turn, this
helper prevents a clearly straight/opposite model curvature from silently
winning. The resulting curvature still passes through controlsd's normal
clip_curvature() and lateral controller.
"""

import json
import math
import time

from openpilot.common.constants import CV
from openpilot.common.params import Params


DECISION_PATH = "/dev/shm/nap_navigation_decision.json"

# Only intervene very near an already-authorized intersection turn.
COMMIT_DISTANCE_M = 22.0
COMMIT_SPEED_MPH = 20.0

# Curvature floor ramps up as the intersection gets closer.
# The model is always free to request MORE curvature in the correct direction.
CURVATURE_FLOOR_FAR = 0.010
CURVATURE_FLOOR_NEAR = 0.055
NEAR_DISTANCE_M = 6.0

# Read modeld's decision at 20 Hz rather than doing file I/O at controlsd's 100 Hz.
READ_INTERVAL_S = 0.05
MAX_DECISION_AGE_S = 0.35

# After the driver helps the steering, do not snap the navigation curvature
# floor straight back in. Blend from the live model curvature to the bounded
# navigation floor over this interval.
REACQUIRE_SECONDS = 0.75


class NavigationTurnCommit:
  def __init__(self):
    self.params = Params()
    self.enabled = False
    self.last_param_read = -1e9
    self.last_read = -1e9
    self.decision = None

    self.active = False
    self.state = "OFF"
    self.side = "none"
    self.distance = None
    self.floor = 0.0
    self.model_curvature = 0.0
    self.output_curvature = 0.0

    # Seamless driver-assist handoff state.
    self.driver_assisting = False
    self.reacquire_start = None

  def _refresh(self, now):
    if now - self.last_param_read >= 1.0:
      self.enabled = self.params.get_bool("NAPNavigationManeuvers")
      self.last_param_read = now

    if now - self.last_read < READ_INTERVAL_S:
      return

    self.last_read = now
    self.decision = None

    try:
      with open(DECISION_PATH) as f:
        raw = f.read(4097)

      if len(raw) > 4096:
        return

      d = json.loads(raw)
      received = d.get("received_mono")

      if type(received) not in (int, float) or not math.isfinite(received):
        return

      age = now - received
      if not 0.0 <= age <= MAX_DECISION_AGE_S:
        return

      self.decision = d

    except (OSError, ValueError, TypeError, AttributeError):
      pass

  @staticmethod
  def _floor_for_distance(distance):
    # 22 m -> 0.010
    #  6 m -> 0.055
    # <=6 m -> 0.055
    if distance <= NEAR_DISTANCE_M:
      return CURVATURE_FLOOR_NEAR

    span = COMMIT_DISTANCE_M - NEAR_DISTANCE_M
    progress = (COMMIT_DISTANCE_M - distance) / span
    progress = max(0.0, min(1.0, progress))

    return (
      CURVATURE_FLOOR_FAR
      + progress * (CURVATURE_FLOOR_NEAR - CURVATURE_FLOOR_FAR)
    )

  def update(self, curvature, CS, lat_active, now=None):
    now = time.monotonic() if now is None else now
    self._refresh(now)

    self.active = False
    self.state = "OFF"
    self.side = "none"
    self.distance = None
    self.floor = 0.0
    self.model_curvature = float(curvature)
    self.output_curvature = float(curvature)

    if not self.enabled:
      self.driver_assisting = False
      self.reacquire_start = None
      return curvature

    d = self.decision
    if not isinstance(d, dict):
      self.driver_assisting = False
      self.reacquire_start = None
      self.state = "WAITING"
      return curvature

    desire = d.get("desire")
    confirmed = d.get("confirmed") is True
    distance = d.get("distance_m")
    modifier = d.get("modifier")

    # ----------------------------------------------------------
    # Seamless driver steering assistance.
    #
    # The driver always wins immediately. Do not inject the navigation
    # curvature floor while steering override is present, but remember that
    # the driver was helping an already-confirmed navigation maneuver.
    #
    # When the driver releases steering, begin a short bounded reacquisition
    # ramp instead of snapping navigation curvature straight back in.
    # ----------------------------------------------------------
    steering_override = bool(
      CS.steeringPressed or getattr(CS, "steeringDisengage", False)
    )

    if steering_override:
      self.reacquire_start = None

      if confirmed:
        self.driver_assisting = True
        self.state = "DRIVER ASSIST"

        if modifier in ("left", "slight left"):
          self.side = "left"
        elif modifier in ("right", "slight right"):
          self.side = "right"

        if type(distance) in (int, float) and math.isfinite(distance):
          self.distance = float(distance)
      else:
        self.driver_assisting = False
        self.state = "WAITING"

      return curvature

    if self.driver_assisting:
      # First frame after the driver releases the wheel.
      self.driver_assisting = False

      if confirmed and lat_active:
        self.reacquire_start = now
      else:
        self.reacquire_start = None

    if desire == "turnLeft":
      side = "left"
      sign = 1.0
    elif desire == "turnRight":
      side = "right"
      sign = -1.0
    else:
      # Navigation may already have confirmed the turn but still be waiting
      # for the model-guidance distance/speed window.
      if confirmed:
        self.state = "ARMED"
        modifier = d.get("modifier")
        if modifier in ("left", "slight left"):
          self.side = "left"
        elif modifier in ("right", "slight right"):
          self.side = "right"
        if type(distance) in (int, float) and math.isfinite(distance):
          self.distance = float(distance)
      else:
        self.state = "WAITING"
      return curvature

    self.side = side

    if type(distance) not in (int, float) or not math.isfinite(distance):
      self.state = "INVALID"
      return curvature

    distance = float(distance)
    self.distance = distance

    speed_mph = max(0.0, float(CS.vEgo)) * CV.MS_TO_MPH

    # NavigationDesire has already checked GPS quality, route identity,
    # driver takeover, brake state, lateral engagement, etc. Repeat the
    # important actuator-side gates here so stale state cannot keep assisting.
    if (
      not confirmed
      or not lat_active
      or not 0.0 < distance <= COMMIT_DISTANCE_M
      or speed_mph > COMMIT_SPEED_MPH
    ):
      self.state = "GUIDING"
      return curvature

    floor = self._floor_for_distance(distance)
    self.floor = floor

    signed = sign * float(curvature)

    # Model already agrees with navigation and is turning strongly enough.
    # No synthetic handoff is necessary if the model itself is doing the job.
    if signed >= floor:
      self.reacquire_start = None
      self.active = True
      self.state = "MODEL TURN"
      return curvature

    # Model is too straight or pointing the wrong direction.
    # Establish only a minimum turn request. Never reduce a stronger
    # correct-direction model request.
    target = sign * floor
    output = target

    # If the driver just released steering assistance, blend from the current
    # live model curvature toward the navigation floor. At blend=0 the output
    # is exactly the model request; at blend=1 normal turn commitment is back.
    if self.reacquire_start is not None:
      elapsed = max(0.0, now - self.reacquire_start)
      blend = min(1.0, elapsed / REACQUIRE_SECONDS)

      output = float(curvature) + blend * (target - float(curvature))

      if blend < 1.0:
        self.state = "REACQUIRE"
      else:
        self.reacquire_start = None
        self.state = "TURN ASSIST"
    else:
      self.state = "TURN ASSIST"

    self.active = True
    self.output_curvature = output
    return output
