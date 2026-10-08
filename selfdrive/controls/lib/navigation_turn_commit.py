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
from openpilot.selfdrive.controls.lib.turn_steering_gate import matching_signal
from openpilot.selfdrive.controls.lib.turn_context import driver_braking, paired_decision, manual_turn_intent
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


DECISION_PATH = "/dev/shm/nap_navigation_decision.json"

# Only intervene very near an already-authorized intersection turn.
COMMIT_DISTANCE_M = 22.0
COMMIT_SPEED_MPH = 25.0

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
    self.manual = ManualTurnCommit()
    self.enabled = False
    self.last_param_read = -1e9
    self.last_read = -1e9
    self.last_diagnostic = -1e9
    self.decision = None
    self.manual_veto_direction = 0
    self.manual_veto_until = -math.inf

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

  def update(self, curvature, CS, lat_active, now=None, *, model=None, healthy=False, actual_curvature=0.0):
    now = time.monotonic() if now is None else now
    output = self._update(curvature, CS, lat_active, now, model=model,
                          healthy=healthy, actual_curvature=actual_curvature)
    if lat_active and (CS.leftBlinker != CS.rightBlinker) and now - self.last_diagnostic >= 1.0:
      self.last_diagnostic = now
      decision = self.decision or {}
      stamp = decision.get('received_mono')
      age = now - stamp if type(stamp) in (int, float) else -1.0
      cloudlog.info('NAP turn state=%s side=%s speed=%.2f model=%.5f request=%.5f age=%.3f source=%s blocked=%s',
                    self.state, self.side, CS.vEgo, curvature, output, age,
                    decision.get('source'), decision.get('manual_turn_blocked'))
    return output

  def _update(self, curvature, CS, lat_active, now, *, model, healthy, actual_curvature):
    self._refresh(now)

    self.active = False
    self.state = "OFF"
    self.side = "none"
    self.distance = None
    self.floor = 0.0
    self.model_curvature = float(curvature)
    self.output_curvature = float(curvature)

    manual_output = self.manual.update(curvature, CS, lat_active, self.decision,
                                       model, healthy, actual_curvature, now)
    if self.manual.active:
      self.active = True
      self.state = self.manual.state
      self.side = self.manual.side
      self.floor = self.manual.floor
      self.output_curvature = manual_output
      self.driver_assisting = False
      self.reacquire_start = None
      return manual_output

    # ----------------------------------------------------------
    # City Turns directional veto.
    #
    # If City Turns has identified a genuine driver-owned manual turn,
    # the blinker chooses the allowed direction. While positive turn
    # commitment is still accumulating, contradictory model curvature
    # is suppressed to straight rather than steering the wrong way.
    # ----------------------------------------------------------
    d = self.decision
    if isinstance(d, dict) and lat_active:
      direction = d.get("manual_turn_direction")
      one_blinker = bool(CS.leftBlinker) != bool(CS.rightBlinker)

      paired_intent = manual_turn_intent(CS, d, model, now)
      if paired_intent:
        self.manual_veto_direction = direction
        self.manual_veto_until = now + .15
      # Retain only a previously frame-validated directional veto across
      # brief asynchronous publication gaps. This cannot authorize a turn floor.
      carried_veto = (direction == self.manual_veto_direction and now <= self.manual_veto_until
                      and manual_turn_intent(CS, d, model, now, require_pair=False))
      manual_valid = (
        (paired_intent or carried_veto)
        and
        one_blinker
        and healthy
        and d.get("city_turns_enabled") is True
        and d.get("signal_owner") == "driver"
        and d.get("tap_signal_active") is False
        and d.get("synthetic_indicator_request") == "none"
        and d.get("model_evaluated") is True
        and -0.1 <= CS.vEgo <= COMMIT_SPEED_MPH * CV.MPH_TO_MS
        and str(CS.gearShifter) in ("drive", "low", "sport", "eco")
        and not CS.steeringPressed
        and not getattr(CS, "steeringDisengage", False)
        and not driver_braking(CS)
        and CS.canValid
      )

      if manual_valid and direction in (1, 2):
        requested_sign = -1.0 if direction == 1 else 1.0
        expected = "turnLeft" if direction == 1 else "turnRight"

        matching_lamp = (
          (direction == 1 and bool(CS.leftBlinker))
          or (direction == 2 and bool(CS.rightBlinker))
        )

        if matching_lamp and not (CS.leftBlindspot if direction == 1 else CS.rightBlindspot):
          signed_curvature = requested_sign * float(curvature)
          # A fresh physical hold owns direction even while the model is
          # still waiting to choose a corner. This is only a veto: no positive
          # blinker-side boost. Once the maneuver is handed back/cancelled,
          # leave the model free to make its exit alignment corrections.
          guard_allowed = (d.get('manual_turn_blocked') is not True and not self.manual.blocked)
          if guard_allowed and signed_curvature < 0.0:
            self.active = True
            self.state = "MANUAL VETO"
            self.side = "left" if direction == 1 else "right"
            self.output_curvature = 0.0
            return 0.0

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

    # This is model/control curvature: positive right, negative left.
    # LatControlAngle converts to steering angle with -desired_curvature;
    # the Tesla CAN adapter then converts that angle to Tesla's wire sign.
    if desire == "turnLeft":
      side = "left"
      sign = -1.0
    elif desire == "turnRight":
      side = "right"
      sign = 1.0
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
      or not matching_signal(CS, 1 if side == "left" else 2)
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


class ManualTurnCommit:
  """Retain only curvature supported by a sustained, matching model turn.

  The model/control coordinate system is positive right (left is negative).
  This is deliberately separate from navigation's distance-derived floor:
  manual intent supplies no intersection location and cannot invent a radius.
  """

  # Manual blinker intent still requires matching model geometry, but
  # react to the beginning of the turn rather than waiting until the
  # model has already committed deeply into the intersection.
  EVIDENCE_SECONDS = 0.20
  RAMP_SECONDS = 0.25
  HOLD_SECONDS = 0.75
  MAX_SECONDS = 8.0
  MAX_ACCEL = 1.5
  MIN_CURVATURE = 0.004

  # Once a matching manual/blinker turn has passed all geometry and
  # sustained-evidence gates, allow a small early arc even while the
  # model's immediate desiredCurvature is still nearly straight.
  #
  # This is NOT a blinker-only steering command: entry_evidence(),
  # direction, ownership, freshness, health and the 0.20 s dwell must
  # all succeed first.
  EARLY_CURVATURE_FLOOR = 0.010

  # A short, geometry-derived seed assists initial selection only. Never
  # impose the old fixed .045/.055 curvature after the model picks a path.
  SEED_MAX_CURVATURE = 0.032
  MODEL_AUTHORITY_CURVATURE = MIN_CURVATURE
  STOP_RELEASE_SPEED = 1.0
  STOP_EVIDENCE_SECONDS = 0.10
  STOP_EVIDENCE_FRAMES = 3
  STOP_COMMIT_RAMP_SECONDS = 0.15

  # Intent-first stopped-intersection fallback.
  #
  # A classified physical blinker may pre-arm while fully stopped only when
  # matching near-path geometry establishes a turn. Steering remains untouched
  # at standstill. On rollout, provide only a small bounded seed and require
  # the model/path geometry to acquire the maneuver shortly afterward.
  STOP_INTENT_GRACE_SECONDS = 0.80
  STOP_INTENT_MAX_DISTANCE_M = 4.0

  # Once the vehicle has made meaningful progress through the corner, stop
  # forcing the committed floor as soon as the near path and both curvature
  # requests agree that the road directly ahead is straight. Far-horizon
  # residual bend must not keep steering into the completed turn.
  EXIT_MIN_HEADING = math.radians(12)
  EXIT_NEAR_HEADING = math.radians(3)
  EXIT_NEAR_OFFSET_M = 0.35
  EXIT_MODEL_CURVATURE = 0.012
  EXIT_PLANNER_CURVATURE = 0.018

  def __init__(self):
    self.blocked = False
    self.last_time = None
    self.reset()

  def reset(self):
    self.active = False
    self.state = "MANUAL WAITING"
    self.side = "none"
    self.direction = 0
    self.floor = 0.0
    self.evidence_since = None
    self.started = None
    self.last_evidence = None
    self.last_frame = None
    self.last_model_time = None
    self.evidence_frames = 0

    # Entry evidence has two valid modes:
    #   signed    = instantaneous curvature is meaningfully nonzero
    #   ambiguous = instantaneous curvature is near zero, but future
    #               path geometry clearly shows the intersection
    #
    # Either mode may qualify if sustained. Switching modes restarts
    # the dwell so one ambiguous frame cannot bridge two evidence runs.
    self.entry_evidence_mode = None

    self.heading = 0.0
    self.distance = 0.0
    self.straight_since = None
    self.started_from_stop = False
    self.model_acquired = False

    # Intent-first stopped-intersection state.
    self.intent_frame = None
    self.stop_intent_since = None
    self.stop_intent_frames = 0
    self.stop_intent_armed = False
    self.intent_seed_start = None

  def cancel(self):
    self.blocked = self.blocked or self.started is not None
    self.reset()

  @staticmethod
  def path_evidence(model, sign, *, continuing=False, require_raw_direction=True):
    try:
      ts = list(model.position.t)
      xs, ys = list(model.position.x), list(model.position.y)
      yaw = list(model.orientation.z)
      raw = float(model.action.desiredCurvature)
      if (len(ts) < 3 or not len(ts) == len(xs) == len(ys) == len(yaw)
          or not all(math.isfinite(v) for v in ts + xs + ys + yaw + [raw])
          or any(b <= a for a, b in zip(ts, ts[1:]))):
        return None
      indices = [i for i, t in enumerate(ts) if 0.5 <= t <= 3.0]
      if len(indices) < 2:
        return None
      i = indices[-1]
      heading = sign * (yaw[i] - yaw[0])
      offset = sign * (ys[i] - ys[0])
      # Starting requires a distinct nearby turn. Once underway, the remaining
      # heading and offset naturally shrink; do not reapply the entry threshold.
      minimum_forward = 0.5 if continuing else 3.0
      # Entry thresholds intentionally look for the *beginning* of a
      # real intersection turn. A straight path still cannot arm the
      # helper, and opposite/lane-change-shaped paths remain rejected.
      minimum_offset = 0.25 if continuing else 0.60
      minimum_heading = math.radians(4 if continuing else 10)
      minimum_curvature = 0.003 if continuing else ManualTurnCommit.MIN_CURVATURE
      raw_direction_ok = (
        sign * raw >= minimum_curvature
        if require_raw_direction
        else True
      )

      credible = (
        xs[i] - xs[0] >= minimum_forward
        and offset >= minimum_offset
        and minimum_heading <= heading <= math.radians(120)
        and raw_direction_ok
        and all(sign * (yaw[j] - yaw[0]) >= -0.05 for j in indices)
      )
      return credible, heading, sign * raw
    except (AttributeError, TypeError, ValueError, OverflowError):
      return None

  @staticmethod
  def turn_shape_evidence(model, sign, *, continuing=False):
    """Detect a genuine nearby corner without letting the model choose its side."""
    try:
      ts = list(model.position.t)
      xs = list(model.position.x)
      ys = list(model.position.y)
      yaw = list(model.orientation.z)
      model_raw = float(model.action.desiredCurvature)

      if (len(ts) < 3
          or not len(ts) == len(xs) == len(ys) == len(yaw)
          or not all(math.isfinite(v) for v in ts + xs + ys + yaw + [model_raw])
          or any(b <= a for a, b in zip(ts, ts[1:]))):
        return None

      indices = [i for i, t in enumerate(ts) if 0.5 <= t <= 3.0]
      if len(indices) < 2:
        return None

      i = indices[-1]

      forward = xs[i] - xs[0]
      heading = abs(yaw[i] - yaw[0])
      offset = abs(ys[i] - ys[0])

      minimum_forward = 0.5 if continuing else 3.0
      minimum_offset = 0.25 if continuing else 0.60
      minimum_heading = math.radians(4 if continuing else 10)

      credible = (
        forward >= minimum_forward
        and offset >= minimum_offset
        and minimum_heading <= heading <= math.radians(120)
      )

      # Positive means the immediate model agrees with the blinker.
      # Negative means it currently predicts the other direction.
      raw = sign * model_raw

      return credible, heading, raw

    except (AttributeError, TypeError, ValueError, OverflowError):
      return None

  @staticmethod
  def deep_turn_evidence(model, speed):
    """True when the actual corner is getting very near the vehicle."""
    try:
      start_x = float(model.position.x[0])
      start_y = float(model.position.y[0])
      start_yaw = float(model.orientation.z[0])

      near = min(COMMIT_DISTANCE_M, max(2.5, speed * 1.1))

      near_heading = any(
        0 < float(t) <= 0.7
        and 0 < float(x) - start_x <= near
        and abs(float(yaw) - start_yaw) >= math.radians(6)
        for t, x, yaw in zip(
          model.position.t,
          model.position.x,
          model.orientation.z,
        )
      )

      near_offset = any(
        0 < float(t) <= 1.0
        and abs(float(y) - start_y) >= 0.55
        for t, y in zip(model.position.t, model.position.y)
      )

      return near_heading and near_offset

    except (AttributeError, TypeError, ValueError, OverflowError):
      return False

  @staticmethod
  def turn_exit_evidence(model, curvature):
    """Detect a straight near path after a corner despite residual far bend."""
    try:
      start_y = float(model.position.y[0])
      start_yaw = float(model.orientation.z[0])
      raw = abs(float(model.action.desiredCurvature))

      near_indices = [
        i for i, t in enumerate(model.position.t)
        if 0 < float(t) <= 1.0
      ]
      if not near_indices:
        return False

      near_heading = max(
        abs(float(model.orientation.z[i]) - start_yaw)
        for i in near_indices
      )
      near_offset = max(
        abs(float(model.position.y[i]) - start_y)
        for i in near_indices
      )

      return (
        near_heading <= ManualTurnCommit.EXIT_NEAR_HEADING
        and near_offset <= ManualTurnCommit.EXIT_NEAR_OFFSET_M
        and raw <= ManualTurnCommit.EXIT_MODEL_CURVATURE
        and abs(float(curvature)) <= ManualTurnCommit.EXIT_PLANNER_CURVATURE
      )
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
      return False

  @staticmethod
  def directional_entry_evidence(model, sign, speed):
    """Blinker selects direction; turn geometry proves a real corner exists.

    Correct-direction model curvature qualifies normally.

    Near-zero instantaneous curvature can qualify from matching future geometry.

    Strong opposite instantaneous curvature may still be overridden when the
    future scene contains substantial real corner geometry. This is the case
    City Turns is specifically intended to repair.
    """
    evidence = ManualTurnCommit.path_evidence(
      model,
      sign,
      require_raw_direction=False,
    )
    if evidence is None or not evidence[0]:
      return False

    credible, heading, raw = evidence

    try:
      near = min(COMMIT_DISTANCE_M, max(4.0, speed * 2.0))

      start_x = float(model.position.x[0])
      start_y = float(model.position.y[0])
      start_yaw = float(model.orientation.z[0])

      offsets = [
        sign * (float(y) - start_y)
        for t, y in zip(model.position.t, model.position.y)
        if 0.5 <= float(t) <= 3.0
      ]
      max_offset = max(offsets) if offsets else 0.0

      developing_turn = max_offset >= 0.8

      near_heading = any(
        0 < float(t) <= 1.4
        and 0 < float(x) - start_x <= near
        and sign * (float(yaw) - start_yaw) >= math.radians(1.7)
        for t, x, yaw in zip(
          model.position.t,
          model.position.x,
          model.orientation.z,
        )
      )

      if not (developing_turn and near_heading):
        return False

      # Normal case: instantaneous model already agrees with blinker.
      if raw >= ManualTurnCommit.MIN_CURVATURE:
        return True


    # Weak/near-zero instantaneous curvature must not veto an obvious
    # nearby intersection. Physical blinker owns direction; strong future
    # turn geometry below remains the qualification requirement.
      # Strongly opposite instantaneous prediction can be overridden, but
      # only when the future scene contains substantial corner geometry.
      return (
        heading >= math.radians(12)
        and max_offset >= 1.0
      )

    except (AttributeError, TypeError, ValueError, OverflowError):
      return False

  @staticmethod
  def geometry_floor(model, sign):
    """Small signed-side curvature floor derived from proven future path geometry."""
    try:
      ts = list(model.position.t)
      xs = list(model.position.x)
      yaw = list(model.orientation.z)

      indices = [
        i for i, t in enumerate(ts)
        if 0.5 <= t <= 3.0
      ]
      if not indices:
        return ManualTurnCommit.EARLY_CURVATURE_FLOOR

      i = indices[-1]
      forward = max(0.5, xs[i] - xs[0])
      heading = max(0.0, sign * (yaw[i] - yaw[0]))

      # Rough path curvature is heading / forward distance.
      # Use only half of it as an early commitment floor.
      path_floor = 0.5 * heading / forward

      return max(
        ManualTurnCommit.EARLY_CURVATURE_FLOOR,
        min(0.035, path_floor),
      )
    except (AttributeError, TypeError, ValueError, OverflowError):
      return ManualTurnCommit.EARLY_CURVATURE_FLOOR

  @staticmethod
  def entry_evidence(model, sign, speed):
    """Require the credible turn to start in the near path, not at its horizon."""
    evidence = ManualTurnCommit.path_evidence(
      model,
      sign,
      require_raw_direction=False,
    )
    if evidence is None or not evidence[0]:
      return False
    near = min(COMMIT_DISTANCE_M, max(4.0, speed * 2.0))

    # Starting a NEW manual turn needs more than a small residual bend.
    # A turn that is already unwinding can still have a few degrees of nearby
    # heading, so require meaningful matching lateral displacement in the
    # 3-second path as evidence that an actual corner is developing ahead.
    start_y = model.position.y[0]
    developing_turn = any(
      0.5 <= t <= 3.0 and sign * (y - start_y) >= 0.8
      for t, y in zip(model.position.t, model.position.y)
    )

    near_heading = any(
      0 < t <= 1.4
      and 0 < x - model.position.x[0] <= near
      and sign * (yaw - model.orientation.z[0]) >= math.radians(1.7)
      for t, x, yaw in zip(model.position.t, model.position.x, model.orientation.z)
    )

    return developing_turn and near_heading

  def update(self, curvature, cs, lat_active, decision, model, healthy, actual_curvature, now):
    dt = 0.0 if self.last_time is None else now - self.last_time
    self.last_time = now
    self.active = False
    one = bool(cs.leftBlinker) != bool(cs.rightBlinker)
    if not cs.leftBlinker and not cs.rightBlinker:
      self.blocked = False
      self.reset()
      return curvature
    if cs.steeringPressed and not getattr(cs, "steeringDisengage", False):
      self.cancel()
      self.state = "DRIVER ASSIST CANCELLED"
      return curvature
    if not isinstance(decision, dict):
      self.cancel()
      return curvature
    direction = decision.get('manual_turn_direction')
    sign = -1.0 if direction == 1 else 1.0
    expected = 'turnLeft' if direction == 1 else 'turnRight'
    stamp = decision.get('received_mono')
    frame = decision.get('frame_id')
    fresh = (type(stamp) in (int, float) and math.isfinite(stamp)
             and 0 <= now - stamp <= MAX_DECISION_AGE_S)
    model_frame = getattr(model, 'frameId', None)
    paired = type(frame) is int and type(model_frame) is int and abs(frame - model_frame) <= 2
    if (not healthy or not lat_active or not one or not fresh or not paired
        or not 0 <= dt <= 0.1 or self.blocked
        or type(direction) is not int or direction not in (1, 2)
        or (direction == 1) != bool(cs.leftBlinker)
        or decision.get('city_turns_enabled') is not True
        # Physical ownership plus the independent near-corner dwell authorizes
        # the seed; model selection must not be a prerequisite for correcting it.
        or decision.get('source') not in ('none', 'driver_desire_helper')
        or decision.get('final_desire') not in ('none', expected)
        or decision.get('manual_turn_blocked') is True
        or decision.get('model_evaluated') is not True
        or decision.get('signal_owner') != 'driver'
        or decision.get('tap_signal_active') is not False
        or decision.get('synthetic_indicator_request') != 'none'
        or cs.steeringPressed or getattr(cs, 'steeringDisengage', False)
        or driver_braking(cs) or not cs.canValid
        or str(cs.gearShifter) not in ('drive', 'low', 'sport', 'eco')
        or (cs.leftBlindspot if direction == 1 else cs.rightBlindspot)
        or not all(math.isfinite(v) for v in (cs.vEgo, curvature, actual_curvature))
        or not -0.1 <= cs.vEgo <= COMMIT_SPEED_MPH * CV.MPH_TO_MS
        # Standstill is valid only for pre-arming a new intersection turn.
        # If an already-started maneuver falls back to essentially stopped,
        # cancel it and latch blocked until the physical signal is cleared.
        or (self.started is not None and cs.vEgo <= 0.3)):
      self.cancel()
      return curvature
    if self.direction and self.direction != direction:
      self.cancel()  # Opposite intent cancels; require neutral before rearming.
      return curvature

    # ----------------------------------------------------------
    # Intent-first stopped intersection.
    #
    # Direction comes from the classified physical full stalk. While stopped
    # matching near-path geometry is required; arming adds no curvature.
    # Model conditioning may establish intent earlier, but actuator seeding
    # always needs independent geometry on the physical blinker side.
    # ----------------------------------------------------------
    if model_frame != self.intent_frame:
      self.intent_frame = model_frame

      if (cs.vEgo <= 0.3 and self.started is None
          and self.directional_entry_evidence(model, sign, cs.vEgo)):
        if self.stop_intent_since is None:
          self.stop_intent_since = now
        self.stop_intent_frames += 1

        if (
          self.stop_intent_frames >= 6
          and now - self.stop_intent_since >= self.EVIDENCE_SECONDS
        ):
          self.stop_intent_armed = True

      elif self.started is None:
        self.stop_intent_since = None
        self.stop_intent_frames = 0
        self.stop_intent_armed = False

    # Absolutely no synthetic steering while sitting still.
    if cs.vEgo <= 0.3:
      self.state = (
        'MANUAL STOP ARMED'
        if self.stop_intent_armed
        else 'MANUAL WAITING'
      )
      return curvature

    # First movement after a valid stopped hold. Start with only the existing
    # small early floor. Model geometry gets a short grace period to acquire.
    if (self.stop_intent_armed and self.started is None
        and self.directional_entry_evidence(model, sign, cs.vEgo)):
      self.started = now
      self.started_from_stop = True
      self.intent_seed_start = now
      self.stop_intent_armed = False
      self.last_evidence = now
      self.floor = min(
        self.EARLY_CURVATURE_FLOOR,
        self.MAX_ACCEL / max(cs.vEgo, 1.0) ** 2,
      )

    evidence = self.path_evidence(
      model,
      sign,
      require_raw_direction=False,
      continuing=self.started is not None,
    )
    if evidence is None:
      self.cancel()
      return curvature
    credible, remaining_heading, raw = evidence

    if self.intent_seed_start is not None and credible:
      self.floor = min(
        self.SEED_MAX_CURVATURE,
        self.MAX_ACCEL / cs.vEgo**2,
        self.geometry_floor(model, sign),
      )
      self.last_evidence = now
      self.intent_seed_start = None

    self.direction = direction
    self.side = 'left' if direction == 1 else 'right'
    if self.last_model_time is not None and now - self.last_model_time > 0.15:
      self.cancel()
      return curvature
    if model_frame != self.last_frame:
      self.last_frame = model_frame
      self.last_model_time = now
      entry_ok = (
        self.started is not None
        or self.directional_entry_evidence(model, sign, cs.vEgo)
      )

      if credible and entry_ok:
        if self.evidence_since is None:
          self.evidence_since = now
        self.last_evidence = now
        self.evidence_frames += 1
        starting_from_stop = cs.vEgo <= self.STOP_RELEASE_SPEED
        required_frames = self.STOP_EVIDENCE_FRAMES if starting_from_stop else 5
        required_seconds = self.STOP_EVIDENCE_SECONDS if starting_from_stop else self.EVIDENCE_SECONDS

        if self.evidence_frames >= required_frames and now - self.evidence_since >= required_seconds:
          if self.started is None:
            self.started = now
            self.started_from_stop = starting_from_stop

          self.floor = min(
            self.SEED_MAX_CURVATURE,
            self.MAX_ACCEL / cs.vEgo**2,
            self.geometry_floor(model, sign),
          )
      elif self.started is None:
        self.evidence_since = None
        self.evidence_frames = 0
        self.entry_evidence_mode = None
      if self.started is not None:
        if abs(remaining_heading) < math.radians(8) and abs(raw) < self.MIN_CURVATURE:
          if self.straight_since is None:
            self.straight_since = now
        else:
          self.straight_since = None
    if self.started is None:
      return curvature
    self.heading += sign * actual_curvature * cs.vEgo * dt
    self.distance += cs.vEgo * dt
    if (
      self.heading >= self.EXIT_MIN_HEADING
      and self.turn_exit_evidence(model, curvature)
    ):
      self.cancel()
      return curvature
    if (now - self.started >= self.MAX_SECONDS or self.distance >= 60.0
        or self.heading >= math.radians(120) or self.heading < -math.radians(5)
        or (self.heading >= math.radians(25) and self.straight_since is not None
            and now - self.straight_since >= 0.25)):
      self.cancel()
      return curvature
    intent_seed_grace = (
      self.intent_seed_start is not None
      and not credible
      and now - self.intent_seed_start <= self.STOP_INTENT_GRACE_SECONDS
      and self.distance <= self.STOP_INTENT_MAX_DISTANCE_M
    )

    if intent_seed_grace:
      ramp = min(
        1.0,
        (now - self.intent_seed_start) / self.STOP_COMMIT_RAMP_SECONDS,
      )
      seed = min(
        self.EARLY_CURVATURE_FLOOR,
        self.MAX_ACCEL / cs.vEgo**2,
      ) * ramp

      self.active = True
      self.state = 'MANUAL STOP SEED'

      signed_curvature = sign * curvature
      return sign * max(0.0, signed_curvature, seed)

    # The model never found the corner. Do not keep inventing a path.
    if self.intent_seed_start is not None and not credible:
      self.cancel()
      return curvature

    age = now - self.last_evidence
    if age > self.HOLD_SECONDS:
      self.cancel()
      return curvature
    # Loss of turn geometry releases the seed immediately. Final curvature
    # limiting handles unwind; retaining a synthetic arc here delayed exits.
    if not credible:
      self.cancel()
      return curvature
    ramp_seconds = (
      self.STOP_COMMIT_RAMP_SECONDS
      if self.started_from_stop
      else self.RAMP_SECONDS
    )
    ramp = min(1.0, (now - self.started) / ramp_seconds)
    retain = max(0.0, 1.0 - age / self.HOLD_SECONDS) if not credible else 1.0
    floor = min(self.floor, self.MAX_ACCEL / cs.vEgo**2) * ramp * retain
    self.active = True
    self.state = 'MANUAL TURN' if credible else 'MANUAL UNWIND'

    signed_curvature = sign * curvature

    # Once the model supplies usable matching curvature, follow it through
    # the whole unwind. Do not reapply a floor as its curvature falls at exit.
    self.model_acquired = self.model_acquired or signed_curvature >= self.MODEL_AUTHORITY_CURVATURE
    if self.model_acquired:
      self.state = 'MANUAL MODEL PATH'
      return sign * max(0.0, signed_curvature)

    return sign * max(signed_curvature, floor)



class ManualTurnEntry:
  """Pre-inference authorization; signaling alone never creates a desire.

  Fresh consecutive model frames must show either near turn entry or an
  unmistakable future intersection corner before a physical blinker can
  condition the model. Steering authority keeps its stricter near-path gate.
  """

  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.blocked = False
    self.frame = None
    self.last_frame_time = None
    self.evidence_since = None
    self.frames = 0
    self.started = None
    self.last_evidence = None
    self.stop_armed = False

    # Intent-first stopped-intersection state.
    self.stop_intent_since = None
    self.stop_intent_frames = 0
    self.started_from_stop = False

  def cancel(self):
    # An interrupted approach is not a completed/cancelled turn. Discard its
    # evidence and require a full fresh dwell, but permit early signaling.
    blocked = self.blocked or self.started is not None
    self.reset()
    self.blocked = blocked

  @staticmethod
  def obvious_future_corner(model, sign):
    """Recognize a strong intersection shape before it reaches the near path.

    This authorizes only the model desire. ManualTurnCommit still requires its
    independent near-path evidence before adding any curvature floor.
    """
    evidence = ManualTurnCommit.turn_shape_evidence(
      model,
      sign,
      continuing=False,
    )
    if evidence is None or not evidence[0]:
      return False
    try:
      heading = evidence[1]
      start_y = float(model.position.y[0])
      max_offset = max(
        abs(float(y) - start_y)
        for t, y in zip(model.position.t, model.position.y)
        if 0.5 <= float(t) <= 3.0
      )
      return heading >= math.radians(15) and max_offset >= 1.5
    except (AttributeError, TypeError, ValueError, OverflowError):
      return False

  def update(self, model, direction, speed, authorized, now, driver_assisting=False):
    if driver_assisting:
      self.cancel()
      return False
    if (not authorized or self.blocked or not math.isfinite(speed)
        or not -0.1 <= speed <= COMMIT_SPEED_MPH * CV.MPH_TO_MS
        or self.direction not in (0, direction)):
      self.cancel()
      return False
    self.direction = direction
    sign = -1 if direction == 1 else 1
    evidence = ManualTurnCommit.turn_shape_evidence(
      model,
      sign,
      continuing=self.started is not None,
    )
    if evidence is None:
      self.cancel()
      return False
    frame = getattr(model, 'frameId', None)
    if type(frame) is not int:
      self.cancel()
      return False
    if self.last_frame_time is not None and not 0 <= now - self.last_frame_time <= 0.15:
      self.cancel()
      return False
    if frame != self.frame:
      if self.frame is not None and frame < self.frame:
        self.cancel()
        return False
      self.frame, self.last_frame_time = frame, now
      credible, _, raw = evidence

      # A classified full-stalk hold while stopped may establish intent
      # before the model has drawn the corner. This authorizes model
      # conditioning only; actuator steering still remains zero at standstill.
      if speed <= 0.3 and self.started is None:
        if self.stop_intent_since is None:
          self.stop_intent_since = now
        self.stop_intent_frames += 1

        if (
          self.stop_intent_frames >= 6
          and now - self.stop_intent_since >= ManualTurnCommit.EVIDENCE_SECONDS
        ):
          self.stop_armed = True

      elif self.started is None and not self.stop_armed:
        self.stop_intent_since = None
        self.stop_intent_frames = 0

      # Normally loss of turn shape revokes City Turns immediately. A turn
      # started from a complete stop receives a short acquisition grace period
      # so modeld can react to the newly supplied turn desire.
      if self.started is not None and not credible:
        stop_grace = (
          self.started_from_stop
          and now - self.started <= ManualTurnCommit.STOP_INTENT_GRACE_SECONDS
        )
        if not stop_grace:
          self.cancel()
          return False

        self.last_evidence = now

      entry_ok = (
        self.started is not None
        or ManualTurnCommit.directional_entry_evidence(model, sign, speed)
        or self.obvious_future_corner(model, sign)
      )
      if credible and entry_ok:
        self.last_evidence = now
        if self.evidence_since is None:
          self.evidence_since = now
        self.frames += 1
        if self.frames >= 5 and now - self.evidence_since >= ManualTurnCommit.EVIDENCE_SECONDS:
          if speed <= 0.3:
            self.stop_armed = True
          elif self.started is None:
            self.started = now
      elif self.started is None:
        self.evidence_since = None
        self.frames = 0
    if self.stop_armed and speed > 0.3:
      self.started = now
      self.started_from_stop = True
      self.last_evidence = now
      self.stop_armed = False

    # A genuine corner that has completed the normal evidence dwell may
    # condition the model while stopped. This does NOT grant actuator-side
    # steering authority: ManualTurnCommit still requires vehicle motion
    # before applying its curvature floor.
    #
    # Conditioning the model before rollout prevents the old behavior where
    # the car rolled several feet into the intersection before the model was
    # finally told which direction the physical blinker intended.
    if speed <= 0.3:
      return self.stop_armed

    if self.started is None:
      return False
    if now - self.started >= ManualTurnCommit.MAX_SECONDS or now - self.last_evidence > ManualTurnCommit.HOLD_SECONDS:
      self.cancel()
      return False
    return True
