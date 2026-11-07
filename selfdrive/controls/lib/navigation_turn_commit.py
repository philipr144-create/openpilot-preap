"""Bounded navigation turn commitment for NAP Pre-AP.

This does NOT directly command steering torque or steering angle.

When NavigationDesire has already authorized an intersection turn, this
helper requires a matching visible turn before providing a bounded entry
seed. The model owns curvature after acquisition. All output still passes
through normal clip_curvature() and the lateral controller.
"""

import json
import math
import time

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.turn_steering_gate import matching_signal
from openpilot.selfdrive.controls.lib.turn_context import driver_braking, paired_decision, manual_turn_intent, city_turns_enabled, navigation_turn_intent
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog


DECISION_PATH = "/dev/shm/nap_navigation_decision.json"

# Only intervene very near an already-authorized intersection turn.
COMMIT_DISTANCE_M = 22.0
COMMIT_SPEED_MPH = 25.0
CONTINUE_SPEED_MPH = 35.0

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
    self.navigation_geometry = NavigationGeometryCommit()
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
    self.city_enabled = city_turns_enabled(self.params)
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

  def update(self, curvature, CS, lat_active, now=None, *, model=None, healthy=False, actual_curvature=0.0):
    now = time.monotonic() if now is None else now
    output = self._update(curvature, CS, lat_active, now, model=model,
                          healthy=healthy, actual_curvature=actual_curvature)
    if lat_active and (CS.leftBlinker != CS.rightBlinker) and now - self.last_diagnostic >= 1.0:
      self.last_diagnostic = now
      decision = self.decision or {}
      stamp = decision.get('received_mono')
      age = now - stamp if type(stamp) in (int, float) else -1.0
      cloudlog.info('NAP turn state=%s side=%s speed=%.2f model=%.5f request=%.5f age=%.3f source=%s blocked=%s phase=%s reason=%s',
                    self.state, self.side, CS.vEgo, curvature, output, age,
                    decision.get('source'), decision.get('manual_turn_blocked'),
                    decision.get('manual_turn_phase'), decision.get('manual_turn_reason'))
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

    d = self.decision if isinstance(self.decision, dict) else {}
    nav_key = (d.get('route_id'), d.get('maneuver_id'))
    if not hasattr(self, '_cancelled_navigation_keys'):
      self._cancelled_navigation_keys = set()
    if all(isinstance(v, str) and v for v in nav_key):
      if CS.steeringPressed or getattr(CS, 'steeringDisengage', False):
        self._cancelled_navigation_keys.add(nav_key)
      if nav_key in self._cancelled_navigation_keys:
        self.manual.cancel()
        self.navigation_geometry.suspend()
        self.driver_assisting = False
        self.reacquire_start = None
        self.state = 'NAVIGATION CANCELLED BY DRIVER'
        return curvature

    if not getattr(self, 'city_enabled', False):
      self.manual.cancel()
      self.navigation_geometry.suspend()
      self.manual_veto_direction = 0
      self.manual_veto_until = -math.inf
      self.driver_assisting = False
      self.reacquire_start = None
      self.state = 'CITY TURNS OFF'
      return curvature

    nav_turn=self.decision.get('navigation_turn') if isinstance(self.decision,dict) else None
    nav_request=nav_turn.get('request') if isinstance(nav_turn,dict) else None
    identity=('navigation',nav_request.get('key') if isinstance(nav_request,dict) else None) if isinstance(nav_turn,dict) else ('driver',None)
    if identity!=getattr(self,'_city_identity',None):
      self.manual.reset();self.manual.blocked=False;self._city_identity=identity
    if isinstance(nav_turn,dict):
      # Navigation uses the same acquired-path executor, with explicit owner
      # validation below. It never pretends to be a physical driver request.
      self.navigation_geometry.suspend()
      if not self.enabled:
        self.manual.cancel()
        self.state='NAVIGATION OFF'
        return curvature
      output=self.manual.update(curvature,CS,lat_active,self.decision,model,healthy,actual_curvature,now)
      self.active=self.manual.active;self.state='NAV SHARED '+self.manual.state
      self.side=self.manual.side;self.floor=self.manual.floor;self.output_curvature=output
      return output

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

      # Direction veto adds no turn curvature. Accept a bounded publication
      # skew for fresh, classified driver intent; positive turn seeding above
      # still requires its original <=2-frame pairing and geometry checks.
      decision_frame = d.get('frame_id')
      model_frame = getattr(model, 'frameId', None)
      bounded_skew = (type(decision_frame) is int and type(model_frame) is int
                      and abs(decision_frame - model_frame) <= 6)
      fresh_direction = (bounded_skew
                         and manual_turn_intent(CS, d, model, now, require_pair=False))
      manual_valid = (
        fresh_direction
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
            if ManualTurnCommit.alignment_correction(model, curvature):
              self.state = "MANUAL ALIGNMENT"
              self.side = "left" if direction == 1 else "right"
              return curvature
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
      # Discard cached entry evidence throughout handover. Releasing the
      # wheel must collect the existing full geometry dwell/ramp again;
      # completed or otherwise blocked seeds remain blocked.
      self.navigation_geometry.suspend()

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

    output = self.navigation_geometry.update(curvature, CS, d, model, healthy,
                                             actual_curvature, now)
    self.active = self.navigation_geometry.active
    self.state = self.navigation_geometry.state
    self.floor = self.navigation_geometry.floor
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
  HOLD_SECONDS = 1.75
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

  def suspend(self):
    """Yield all added authority; require fresh evidence without a signal reset."""
    blocked = self.blocked
    self.reset()
    self.blocked = blocked

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
  def alignment_correction(model, curvature):
    """Allow a small road-alignment correction in either signal direction.

    A blinker identifies maneuver intent, not the sign of every steering
    correction. Use a straight near path and bounded three-second heading
    to distinguish countersteering from an opposite corner. This only lets
    the existing model request through; it never adds steering authority.
    Invalid/incomplete geometry cannot exempt a request from the veto.
    """
    try:
      ts = list(model.position.t)
      xs, ys = list(model.position.x), list(model.position.y)
      yaw = list(model.orientation.z)
      raw = float(model.action.desiredCurvature)
      if (len(ts) < 3 or not len(ts) == len(xs) == len(ys) == len(yaw)
          or not all(math.isfinite(v) for v in ts + xs + ys + yaw + [raw, curvature])
          or any(b <= a for a, b in zip(ts, ts[1:]))
          or ts[0] > 0.0 or ts[-1] < 3.0
          or max(abs(raw), abs(curvature)) > ManualTurnCommit.EXIT_PLANNER_CURVATURE):
        return False
      near = [i for i, t in enumerate(ts) if 0.0 < t <= 1.0]
      horizon = [i for i, t in enumerate(ts) if 0.0 < t <= 3.0]
      if not near or len(horizon) < 2 or xs[horizon[-1]] <= xs[0]:
        return False
      return (
        max(abs(yaw[i] - yaw[0]) for i in near) <= ManualTurnCommit.EXIT_NEAR_HEADING
        and max(abs(ys[i] - ys[0]) for i in near) <= ManualTurnCommit.EXIT_NEAR_OFFSET_M
        # Below the existing 10-degree corner-entry heading threshold.
        and max(abs(yaw[i] - yaw[0]) for i in horizon) < math.radians(10)
      )
    except (AttributeError, TypeError, ValueError, OverflowError):
      return False

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
    nav_ok = navigation_turn_intent(cs,decision,model,now) if isinstance(decision,dict) else False
    turn_in_progress = self.started is not None
    if not cs.leftBlinker and not cs.rightBlinker and not nav_ok and not turn_in_progress:
      self.blocked = False
      self.reset()
      return curvature
    if turn_in_progress and ((self.direction == 1 and cs.rightBlinker) or (self.direction == 2 and cs.leftBlinker)):
      self.cancel()
      return curvature
    if cs.steeringPressed and not getattr(cs, "steeringDisengage", False):
      self.cancel()
      self.state = "DRIVER ASSIST CANCELLED"
      return curvature
    if not isinstance(decision, dict):
      self.suspend()
      self.state = "MANUAL WAIT DECISION"
      return curvature
    nav=decision.get('navigation_turn')
    nav_request=nav.get('request') if isinstance(nav,dict) else None
    life=nav.get('lifecycle',{}) if isinstance(nav,dict) else {}
    nav_ok=navigation_turn_intent(cs,decision,model,now)
    direction=nav_request.get('direction') if isinstance(nav_request,dict) else decision.get('manual_turn_direction')
    sign = -1.0 if direction == 1 else 1.0
    expected = 'turnLeft' if direction == 1 else 'turnRight'
    stamp = decision.get('received_mono')
    frame = decision.get('frame_id')
    fresh = (type(stamp) in (int, float) and math.isfinite(stamp)
             and 0 <= now - stamp <= MAX_DECISION_AGE_S)
    model_frame = getattr(model, 'frameId', None)
    paired = type(frame) is int and type(model_frame) is int and abs(frame - model_frame) <= 2
    if not fresh or not paired or not 0 <= dt <= 0.1:
      self.suspend()
      self.state = "MANUAL WAIT FRESH DATA"
      return curvature
    phase=life.get('phase') if isinstance(nav,dict) else decision.get('manual_turn_phase')
    if phase is not None and phase not in ('TURN', 'EXIT'):
      if phase in ('COMPLETE', 'CANCELLED'):
        self.cancel()
      else:
        self.suspend()
      self.state = 'MANUAL LIFECYCLE ' + str(phase)
      return curvature
    if self.started is not None and cs.vEgo <= 0.3:
      self.suspend()
      self.state = "MANUAL STOP WAIT"
      return curvature
    if (not healthy or not lat_active or (not one and not nav_ok and not turn_in_progress) or not fresh or not paired
        or not 0 <= dt <= 0.1 or self.blocked
        or type(direction) is not int or direction not in (1, 2)
        or (not nav_ok and not turn_in_progress and (direction == 1) != bool(cs.leftBlinker))
        or decision.get('city_turns_enabled') is not True
        # Physical ownership plus the independent near-corner dwell authorizes
        # the seed; model selection must not be a prerequisite for correcting it.
        or not (nav_ok if isinstance(nav,dict) else decision.get('source') in ('none', 'driver_desire_helper')
                and decision.get('signal_owner')=='driver'
                and decision.get('synthetic_indicator_request')=='none')
        or decision.get('final_desire') not in ('none', expected)
        or (life.get('blocked') if isinstance(nav,dict) else decision.get('manual_turn_blocked')) is True
        or decision.get('model_evaluated') is not True
        or decision.get('tap_signal_active') is not False
        or cs.steeringPressed or getattr(cs, 'steeringDisengage', False)
        or not cs.canValid
        or str(cs.gearShifter) not in ('drive', 'low', 'sport', 'eco')
        or (cs.leftBlindspot if direction == 1 else cs.rightBlindspot)
        or not all(math.isfinite(v) for v in (cs.vEgo, curvature, actual_curvature))
        or not -0.1 <= cs.vEgo <= 40.0 * CV.MPH_TO_MS
):
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
      self.suspend()
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
      self.suspend()
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

    # Above city speed, preserve established maneuver context without adding
    # a turn seed/floor or changing the model's requested direction.
    if cs.vEgo > COMMIT_SPEED_MPH * CV.MPH_TO_MS:
      self.active = True
      self.state = 'MANUAL MODEL CONTINUATION'
      return curvature

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



class CityTurnIntent:
  """Motion-bounded model conditioning from qualified physical driver intent.

  A sustained classified hold may condition the model before matching geometry
  exists below 5 mph, or on a qualified slowing approach at or below 25 mph.
  Positive added steering retains ManualTurnCommit's independent near-path
  checks; conditioning can still change ordinary model-following steering.
  Fresh low-speed waiting preserves pending intent without spending the moving
  search budget. It is a motion heuristic, not traffic-light recognition.
  """

  LOW_INTENT_START_SPEED = 5.0 * CV.MPH_TO_MS
  LOW_INTENT_MAX_DISTANCE = 100.0
  # Offline candidate admission limits; not validated for deployment.
  ROLLING_INTENT_START_SPEED = 25.0 * CV.MPH_TO_MS
  ROLLING_APPROACH_SECONDS = 1.5
  ROLLING_APPROACH_SPEED_DROP = 0.5
  ROLLING_APPROACH_HOLD_SECONDS = 1.0
  ROLLING_APPROACH_REBOUND = 0.25
  QUEUE_ENTER_SPEED = 5.0 * CV.MPH_TO_MS
  QUEUE_EXIT_SPEED = 6.0 * CV.MPH_TO_MS
  QUEUE_TURN_HEADING = math.radians(3)
  QUEUE_TURN_CURVATURE = .008

  def __init__(self):
    self.reset()

  def reset(self):
    self.direction = 0
    self.phase = 'IDLE'
    self.reason = 'NO_SIGNAL'
    self.heading = 0.0
    self.motion_time = None
    self.exit_since = None
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
    self.intent_distance = 0.0
    self.intent_path_acquired = False
    self.turn_locked = False
    self.approach_samples = []
    self.last_slowing = None
    self.last_slowing_speed = None
    self.authorization_trace = {}
    self.queue_waiting = False
    self.physical_turn_started = False
    self.search_elapsed = 0.
    self.search_distance = 0.
    self.active_elapsed = 0.
    self.waiting_elapsed = 0.
    self.intent_origin = 'none'
    self.turn_started = None
    self.last_gates = {}
    self.last_event = None
    self.peak_actual_curvature = 0.
    self.exit_mode = None

  def suspend(self, reason="FRESH_EVIDENCE_REQUIRED"):
    """Yield all added authority; require fresh evidence without a signal reset."""
    blocked = self.blocked
    event = self.event(reason)
    self.reset()
    self.blocked = blocked
    self.last_event = event
    self.phase = 'CANCELLED' if blocked else 'WAIT'
    self.reason = reason

  def cancel(self, reason="AUTHORIZATION_LOST"):
    # Unaccepted approach evidence can qualify afresh. Revocation of an accepted
    # stopped or moving request remains latched until the signal is neutral.
    blocked = self.blocked or self.started is not None or self.stop_armed
    event = self.event(reason)
    self.reset()
    self.blocked = blocked
    self.last_event = event
    self.phase = 'CANCELLED' if blocked else 'WAIT'
    self.reason = reason

  def complete(self, reason='NEAR_PATH_STRAIGHT'):
    event = self.event(reason)
    self.reset()
    self.last_event = event
    self.blocked = True
    self.phase = 'COMPLETE'
    self.reason = reason

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

  def update(self, model, direction, speed, authorized, now, driver_assisting=False, transient_unavailable=False, actual_curvature=None, stationary_wait=False):
    self.current_frame = getattr(model, 'frameId', None)
    self.current_time = now
    if self.blocked:
      return False
    if driver_assisting:
      self.cancel('DRIVER_OVERRIDE')
      return False
    if transient_unavailable:
      self.suspend('INPUTS_UNAVAILABLE')
      return False
    # Stopping after physical turn entry still relinquishes the old motion
    # commitment. Before entry, a normal queue stop retains pending intent.
    if self.started is not None and self.physical_turn_started and math.isfinite(speed) and speed <= 0.3:
      self.suspend()
    stationary_wait = bool(stationary_wait and math.isfinite(speed) and speed <= .3
                           and not self.physical_turn_started)
    if ((not authorized and not stationary_wait) or self.blocked or not math.isfinite(speed)
        or not -0.1 <= speed <= (CONTINUE_SPEED_MPH if self.started is not None else 40.0) * CV.MPH_TO_MS
        or self.direction not in (0, direction)):
      self.cancel('AUTHORIZATION_LOST' if not authorized else 'SPEED_OR_DIRECTION_INVALID')
      return False
    self.direction = direction
    sign = -1 if direction == 1 else 1
    evidence = ManualTurnCommit.path_evidence(
      model,
      sign,
      require_raw_direction=False,
      continuing=self.started is not None,
    )
    if evidence is None:
      self.suspend('MODEL_GEOMETRY_UNAVAILABLE')
      return False
    self.last_gates = {'path_credible': bool(evidence[0]), 'signed_heading_deg': math.degrees(evidence[1]),
                       'signed_model_curvature': evidence[2]}
    frame = getattr(model, 'frameId', None)
    if type(frame) is not int:
      self.cancel('MODEL_FRAME_INVALID')
      return False
    if self.last_frame_time is not None and not 0 <= now - self.last_frame_time <= 0.15:
      self.suspend('MODEL_FRAME_STALE')
      return False
    if frame != self.frame:
      if self.frame is not None and frame < self.frame:
        self.cancel('MODEL_FRAME_REGRESSED')
        return False
      dt = 0.0 if self.motion_time is None else now - self.motion_time
      self.motion_time = now
      was_waiting = self.queue_waiting
      measured_curvature = actual_curvature is not None and math.isfinite(actual_curvature)
      turning_curvature = measured_curvature and abs(actual_curvature) >= self.QUEUE_TURN_CURVATURE
      if self.started is not None and 0 <= dt <= .15:
        self.intent_distance += max(0.0, speed) * dt
        if measured_curvature and (not was_waiting or speed > self.QUEUE_EXIT_SPEED or turning_curvature):
          self.heading += sign * actual_curvature * max(0.0, speed) * dt
          self.peak_actual_curvature = max(self.peak_actual_curvature, sign*actual_curvature)
        # A brief steering correction in a queue is not proof of turn entry.
        # Require observed heading change before permanently ending waiting.
        if abs(self.heading) >= self.QUEUE_TURN_HEADING:
          self.physical_turn_started = True
      queue_speed = self.QUEUE_EXIT_SPEED if was_waiting else self.QUEUE_ENTER_SPEED
      self.queue_waiting = (not self.physical_turn_started and self.turn_started is None
                            and speed <= queue_speed and (speed <= .3 or measured_curvature))
      if 0 <= dt <= .15:
        if self.queue_waiting and was_waiting:
          if self.started is not None or self.stop_armed:
            self.waiting_elapsed += dt
        elif self.started is not None:
          self.active_elapsed += dt
          if self.turn_started is None:
            self.search_elapsed += dt
            self.search_distance += max(0.0, speed) * dt
      if self.started_from_stop and self.heading < -math.radians(5):
        self.cancel('WRONG_WAY_MOTION')
        return False
      if self.started is not None and self.turn_started is None and self.search_distance > self.LOW_INTENT_MAX_DISTANCE:
        self.cancel('SEARCH_DISTANCE_EXPIRED')
        return False
      if stationary_wait:
        self.frame, self.last_frame_time = frame, now
        self.phase, self.reason = 'ARMED', 'WAITING_FOR_ENGAGEMENT'
        return False
      # Model conditioning can precede physical turn entry. Keep separate
      # bounded clocks so early conditioning does not consume turn completion.
      if (self.started is not None and self.turn_started is None
          and self.heading >= ManualTurnCommit.EXIT_MIN_HEADING and evidence[0]):
        self.turn_started = now
      # Clear model conditioning as well as added steering when the near path
      # straightens after observed turning. A distant next bend must not retain
      # the completed city-turn desire while the stalk remains held.
      straight_exit = (self.started is not None and self.heading >= ManualTurnCommit.EXIT_MIN_HEADING
                       and ManualTurnCommit.turn_exit_evidence(model, getattr(getattr(model, 'action', None), 'desiredCurvature', float('nan'))))
      curved_exit = self.curved_exit_evidence(model, actual_curvature, sign)
      exit_mode = 'NEAR_PATH_STRAIGHT' if straight_exit else 'UNWOUND_ALIGNED_EXIT' if curved_exit else None
      if exit_mode is not None:
        if self.exit_since is None or self.exit_mode != exit_mode:
          self.exit_since = now
        self.exit_mode = exit_mode
        self.phase = 'EXIT'
        if now - self.exit_since >= (.20 if straight_exit else .50):
          self.complete(exit_mode)
          return False
      else:
        self.exit_since = None
        self.exit_mode = None
        self.phase = 'TURN' if self.started is not None else 'ARMED'
      if self.phase != 'EXIT':
        self.phase = 'TURN' if self.started is not None else 'ARMED'
      self.reason = 'QUALIFYING_GEOMETRY' if self.started is None else 'TURN_IN_PROGRESS'
      self.frame, self.last_frame_time = frame, now
      credible, _, raw = evidence

      # Physical intent admission is distinct from predicted path acquisition.
      # A rolling request additionally needs a measured slowing approach; an
      # early signal at steady speed must not start a repeated search window.
      self.approach_samples.append((now, speed))
      self.approach_samples = [(t, v) for t, v in self.approach_samples
                               if 0 <= now-t <= self.ROLLING_APPROACH_SECONDS + .5][-50:]
      older = [(t, v) for t, v in self.approach_samples if now-t >= self.ROLLING_APPROACH_SECONDS]
      recent = [v for t, v in self.approach_samples if now-t <= .5]
      rebound = bool(recent) and speed > min(recent) + self.ROLLING_APPROACH_REBOUND
      speed_drop = older[-1][1] - speed if older else None
      if speed_drop is not None and speed_drop >= self.ROLLING_APPROACH_SPEED_DROP and not rebound:
        self.last_slowing, self.last_slowing_speed = now, speed
      slowing_age = None if self.last_slowing is None else now-self.last_slowing
      approach_held = (slowing_age is not None and 0 <= slowing_age <= self.ROLLING_APPROACH_HOLD_SECONDS
                       and speed <= self.last_slowing_speed + self.ROLLING_APPROACH_REBOUND and not rebound)
      # The newly admitted 18–25 mph band needs a matching corner, so a
      # slowing manual lane change does not gain automatic turn conditioning.
      rolling_ready = (speed <= self.ROLLING_INTENT_START_SPEED and approach_held
                       and (speed <= 18.0 * CV.MPH_TO_MS
                            or self.obvious_future_corner(model, sign) and raw >= 0))
      self.last_gates = {'path_credible': bool(credible), 'signed_heading_deg': math.degrees(evidence[1]),
                         'signed_model_curvature': evidence[2], 'low_speed': speed <= self.LOW_INTENT_START_SPEED,
                         'rolling_approach': bool(rolling_ready), 'rolling_speed_limit_mps': self.ROLLING_INTENT_START_SPEED,
                         'approach_speed_drop_mps': speed_drop, 'slowing_evidence_age_s': slowing_age,
                         'approach_evidence_held': bool(approach_held), 'speed_rebound': rebound}
      # High-speed approach gathers history and a geometry lock only. Never
      # condition the model above the existing steering admission ceiling.
      if speed > self.ROLLING_INTENT_START_SPEED and self.started is None:
        corner = credible and raw >= 0 and self.obvious_future_corner(model, sign)
        if corner:
          if self.evidence_since is None:
            self.evidence_since = now
          self.frames += 1
        else:
          self.evidence_since, self.frames = None, 0
        self.turn_locked = bool(corner and self.frames >= 5
                                and now-self.evidence_since >= ManualTurnCommit.EVIDENCE_SECONDS)
        self.phase, self.reason = 'CANDIDATE', 'EARLY_GEOMETRY_LOCK' if self.turn_locked else 'EARLY_INTENT'
        self.last_gates['turn_locked'] = self.turn_locked
        return False
      # Intent can begin while stopped/creeping, after six fresh frames.
      # Requesting a model direction never arms a blind actuator turn floor.
      if (speed <= self.LOW_INTENT_START_SPEED or rolling_ready) and self.started is None:
        if self.stop_intent_since is None:
          self.stop_intent_since = now
        self.stop_intent_frames += 1

        if (
          self.stop_intent_frames >= 6
          and now - self.stop_intent_since >= ManualTurnCommit.EVIDENCE_SECONDS
        ):
          self.stop_armed = True
          self.intent_origin = 'physical_low_speed' if speed <= self.LOW_INTENT_START_SPEED else 'physical_rolling_approach'

      elif self.started is None and not self.stop_armed:
        self.stop_intent_since = None
        self.stop_intent_frames = 0

      # Low-speed model intent may search within its time/distance budget.
      # An acquired turn retains input briefly across an opposite prediction,
      # but positive steering still requires its independent matching path.
      if self.started is not None and not credible and self.exit_since is None:
        stop_grace = (
          (self.started_from_stop or self.queue_waiting)
          and (not self.intent_path_acquired or self.heading < ManualTurnCommit.EXIT_MIN_HEADING)
          and self.search_elapsed < ManualTurnCommit.MAX_SECONDS
          and self.search_distance <= self.LOW_INTENT_MAX_DISTANCE
        )
        brief_geometry_gap = (self.last_evidence is not None
                              and now - self.last_evidence <= ManualTurnCommit.HOLD_SECONDS)
        if not (stop_grace or brief_geometry_gap):
          self.cancel('PATH_CREDIBILITY_LOST')
          return False
        if stop_grace:
          self.last_evidence = now

      entry_ok = (
        self.started is not None
        or ManualTurnCommit.directional_entry_evidence(model, sign, speed)
        or self.obvious_future_corner(model, sign)
      )
      self.last_gates['path_entry'] = bool(entry_ok)
      self.turn_locked = bool(credible and raw >= 0 and entry_ok
                              and (self.intent_path_acquired or self.frames >= 5))
      self.last_gates['turn_locked'] = self.turn_locked
      if self.started is None:
        self.reason = 'PATH_DWELL' if credible and entry_ok else 'PHYSICAL_INTENT_DWELL' if (speed <= self.LOW_INTENT_START_SPEED or rolling_ready) else 'WAIT_PATH_OR_SLOWING_APPROACH'
      if credible and entry_ok:
        self.last_evidence = now
        if self.evidence_since is None:
          self.evidence_since = now
        self.frames += 1
        if self.frames >= 5 and now - self.evidence_since >= ManualTurnCommit.EVIDENCE_SECONDS:
          if self.started is not None:
            self.intent_path_acquired = True
          if speed <= 0.3:
            self.stop_armed = True
          elif self.started is None:
            self.started = now
            self.intent_origin = 'qualified_geometry' 
      else:
        self.evidence_since = None
        self.frames = 0
    if self.stop_armed and speed > 0.3 and self.started is None:
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
      if self.stop_armed or self.queue_waiting and self.started is not None:
        self.phase, self.reason = 'ARMED', 'WAITING_FOR_TURN_ENTRY'
        return True
      return False

    if self.started is None:
      return False
    time_expired = ((self.turn_started is None and self.search_elapsed >= ManualTurnCommit.MAX_SECONDS)
                    or (self.turn_started is not None and now-self.turn_started >= ManualTurnCommit.MAX_SECONDS)
                    or self.active_elapsed >= 2*ManualTurnCommit.MAX_SECONDS)
    if time_expired or now - self.last_evidence > ManualTurnCommit.HOLD_SECONDS:
      self.cancel('TURN_TIME_EXPIRED' if time_expired else 'PATH_HOLD_EXPIRED')
      return False
    if self.phase != 'EXIT':
      self.phase = 'ARMED' if self.queue_waiting else 'TURN'
      if self.queue_waiting:
        self.reason = 'WAITING_FOR_TURN_ENTRY'
    return True


  def curved_exit_evidence(self, model, actual_curvature, sign):
    """Conservative offline exit proxy; not proof of intersection topology."""
    if (self.started is None or actual_curvature is None or not math.isfinite(actual_curvature)
        or self.heading < math.radians(60) or self.peak_actual_curvature < .025):
      return False
    actual = sign*actual_curvature
    if not 0 <= actual <= min(.015, .4*self.peak_actual_curvature):
      return False
    try:
      if abs(float(model.action.desiredCurvature)-actual_curvature) > .003:
        return False
      indices=[i for i,t in enumerate(model.position.t) if .5 <= t <= 1.5]
      if len(indices) < 2:
        return False
      # Small-angle arc heading k*s, using measured steering curvature and
      # model forward distance. Agreement must persist for the exit dwell.
      return all(abs((model.orientation.z[i]-model.orientation.z[0])
                     - actual_curvature*(model.position.x[i]-model.position.x[0])) <= math.radians(2)
                 for i in indices)
    except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
      return False

  def event(self, reason):
    return {'reason': reason, 'from_phase': self.phase, 'origin': self.intent_origin,
            'direction': self.direction, 'distance_m': self.intent_distance,
            'measured_heading_deg': math.degrees(self.heading), 'gates': dict(self.last_gates),
            'queue_waiting': self.queue_waiting, 'search_elapsed_s': self.search_elapsed,
            'search_distance_m': self.search_distance, 'active_elapsed_s': self.active_elapsed,
            'waiting_elapsed_s': self.waiting_elapsed,
            'model_input_frame': getattr(self, 'current_frame', None),
            'producer_mono': getattr(self, 'current_time', None)}

  def diagnostics(self, now, authorization=None):
    return {'version': 4, 'implementation': 'unified-city-turn-v4', 'producer_mono': now, 'phase': self.phase, 'reason': self.reason,
            'direction': self.direction, 'blocked': self.blocked, 'origin': self.intent_origin,
            'path_acquired': self.intent_path_acquired, 'turn_locked': self.turn_locked,
            'stage': 'TURNING' if self.turn_started is not None else 'WAITING' if self.queue_waiting else 'CONDITIONING' if self.started is not None else self.phase, 'conditioning_active': self.started is not None or self.stop_armed,
            'model_input_frame': self.frame, 'model_input_age_s': None if self.last_frame_time is None else now-self.last_frame_time,
            'request_age_s': None if self.started is None else now-self.started,
            'distance_m': self.intent_distance, 'measured_heading_deg': math.degrees(self.heading),
            'queue_waiting': self.queue_waiting, 'physical_turn_started': self.physical_turn_started,
            'search_elapsed_s': self.search_elapsed, 'search_distance_m': self.search_distance,
            'active_elapsed_s': self.active_elapsed, 'waiting_elapsed_s': self.waiting_elapsed,
            'gates': dict(self.last_gates), 'last_event': self.last_event, 'eligibility': dict(authorization or {}),
            'authorization_trace': dict(self.authorization_trace), 'limits': {'search_time_s': ManualTurnCommit.MAX_SECONDS,
            'search_distance_m': self.LOW_INTENT_MAX_DISTANCE,
            'rolling_admission_mph': self.ROLLING_INTENT_START_SPEED / CV.MPH_TO_MS,
            'recognition_mph': 40.0,
            'approach_window_s': self.ROLLING_APPROACH_SECONDS,
            'approach_speed_drop_mps': self.ROLLING_APPROACH_SPEED_DROP,
            'approach_hold_s': self.ROLLING_APPROACH_HOLD_SECONDS,
            'approach_rebound_mps': self.ROLLING_APPROACH_REBOUND,
            'queue_enter_mph': self.QUEUE_ENTER_SPEED / CV.MPH_TO_MS,
            'queue_exit_mph': self.QUEUE_EXIT_SPEED / CV.MPH_TO_MS,
            'queue_turn_heading_deg': math.degrees(self.QUEUE_TURN_HEADING),
            'queue_turn_curvature': self.QUEUE_TURN_CURVATURE,
            'lifetime_clock': 'non_waiting_time',
            'turn_time_s': ManualTurnCommit.MAX_SECONDS, 'total_lifetime_s': 2*ManualTurnCommit.MAX_SECONDS}}


# Compatibility name; there is one manual city-turn lifecycle implementation.
ManualTurnEntry = CityTurnIntent

class NavigationGeometryCommit:
  """Reuse proven turn-shape checks instead of inventing a radius from GPS.

  Navigation authorizes direction; visible near-path geometry authorizes a
  short seed. Once the model supplies the turn, it owns curvature and unwind.
  """
  def __init__(self):
    self.key = None
    self.reset()

  def reset(self):
    self.active = self.acquired = self.blocked = False
    self.started = self.evidence_since = self.last_time = self.frame = self.frame_time = None
    self.last_evidence = None
    self.frames = 0
    self.heading = self.distance = self.floor = 0.0
    self.state = 'NAV GEOMETRY WAIT'

  def suspend(self):
    blocked = self.blocked
    self.reset()
    self.blocked = blocked
    self.state = 'NAV DRIVER HANDOVER'

  def update(self, curvature, cs, decision, model, healthy, actual_curvature, now):
    key = (decision.get('route_id'), decision.get('maneuver_id'), decision.get('desire'))
    if key != self.key:
      self.reset()
      self.key = key
    self.active = False
    self.floor = 0.0
    if self.blocked:
      return curvature
    if (not healthy or not paired_decision(decision, model, now)
        or decision.get('source') != 'navigation'
        or decision.get('final_desire') != decision.get('desire')
        or not all(math.isfinite(v) for v in (curvature, actual_curvature, cs.vEgo, now))
        or cs.vEgo <= 0.3 or cs.steeringPressed or getattr(cs, 'steeringDisengage', False)):
      self.reset()
      self.state = 'NAV GEOMETRY INVALID'
      return curvature
    dt = 0.0 if self.last_time is None else now - self.last_time
    if not 0 <= dt <= .15:
      self.evidence_since = None
      self.frames = 0
      self.started = None
      self.acquired = False
      self.heading = self.distance = 0.0
      self.last_time = now
      self.state = 'NAV GEOMETRY GAP'
      return curvature
    sign = -1 if decision['desire'] == 'turnLeft' else 1
    evidence = ManualTurnCommit.path_evidence(model, sign, require_raw_direction=False,
                                             continuing=self.started is not None)
    if evidence is not None and evidence[0]:
      self.last_evidence = now
    else:
      self.evidence_since = None
      self.frames = 0
      self.last_time = None
      self.state = 'NAV NO MATCHING PATH'
      if self.started is not None and self.last_evidence is not None and (now - self.last_evidence) <= 1.5:
        self.active = True
        self.state = 'NAV HOLD GRACE'
        return curvature
      self.blocked = self.started is not None
      return curvature
    frame = getattr(model, 'frameId', None)
    if self.frame is not None and frame < self.frame:
      self.blocked = True
      self.state = 'NAV MODEL ORDER'
      return curvature
    fresh = frame != self.frame
    if not fresh and self.frame_time is not None and now - self.frame_time > .15:
      self.reset()
      self.state = 'NAV STALE MODEL'
      return curvature
    if fresh:
      self.frame_time = now
    self.last_time = now
    if self.started is not None:
      self.heading += sign * actual_curvature * cs.vEgo * dt
      self.distance += cs.vEgo * dt
      if (self.heading >= ManualTurnCommit.EXIT_MIN_HEADING
          and ManualTurnCommit.turn_exit_evidence(model, curvature)):
        self.blocked = True
        self.state = 'NAV COMPLETE'
        return curvature
      if (now - self.started > ManualTurnCommit.MAX_SECONDS or self.distance > 60.
          or self.heading < -math.radians(5) or self.heading > math.radians(120)):
        self.blocked = True
        self.state = 'NAV TURN LIMIT'
        return curvature
    if self.started is None:
      if not ManualTurnCommit.entry_evidence(model, sign, cs.vEgo):
        self.evidence_since = None
        self.frames = 0
        self.state = 'NAV ENTRY WAIT'
        return curvature
      if fresh:
        if self.evidence_since is None:
          self.evidence_since = now
        self.frames += 1
      if self.frames < 4 or now - self.evidence_since < ManualTurnCommit.EVIDENCE_SECONDS:
        self.frame = frame
        self.state = 'NAV ENTRY QUALIFY'
        return curvature
      self.started = now
    self.frame = frame
    self.active = True
    signed = sign * curvature
    self.acquired = self.acquired or signed >= ManualTurnCommit.MODEL_AUTHORITY_CURVATURE
    if self.acquired:
      self.state = 'NAV MODEL PATH'
      return curvature
    # Opposite instantaneous steering is not a reason to force a blind arc.
    if signed < -ManualTurnCommit.MIN_CURVATURE:
      self.state = 'NAV MODEL DISAGREES'
      self.active = False
      return curvature
    ramp = min(1., max(0., (now - self.started) / ManualTurnCommit.RAMP_SECONDS))
    self.floor = min(ManualTurnCommit.SEED_MAX_CURVATURE,
                     ManualTurnCommit.MAX_ACCEL / cs.vEgo**2,
                     ManualTurnCommit.geometry_floor(model, sign)) * ramp
    self.state = 'NAV GEOMETRY SEED'
    return sign * max(signed, self.floor)
