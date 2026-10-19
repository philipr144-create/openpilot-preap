#!/usr/bin/env python3
import json
import math
import time
import numpy as np
from openpilot.selfdrive.controls.lib.predictive_grade_assist import PredictiveGradeAssist, FileGradeToggle

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from opendbc.car.tesla.preap.accel_policy import accel_ceiling, limit_positive_accel
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc, LongitudinalPlanSource
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist
from openpilot.selfdrive.controls.lib.cruise_refinement import CruiseRefinement, model_support_weight
from openpilot.selfdrive.controls.lib.map_stop import MapStop, read_snapshot
from openpilot.selfdrive.controls.lib.normal_model_stop import NormalModelStop
from openpilot.selfdrive.controls.lib.normal_propulsion import NormalPropulsion
from openpilot.selfdrive.controls.lib.model_forecast import assess_model_forecast, assess_model_intent
from openpilot.selfdrive.controls.lib.acceleration_handoff import braking_priority, UnifiedCruiseSmoother, curve_recovery_accel_limit, PersonalityAccelerationLimit

A_CRUISE_MAX_VALS = [1.2, 1.0, 1.0, 0.8]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]

# Pre-AP follow-mode accel cap: imported lazily to avoid circular deps
_preap_follow_cache = None
def _get_preap_follow_limit(v_ego):
  global _preap_follow_cache
  if _preap_follow_cache is None:
    try:
      from opendbc.car.tesla.preap.constants import ACCEL_PREAP_BP, ACCEL_PREAP_FOLLOW
      _preap_follow_cache = (ACCEL_PREAP_BP, ACCEL_PREAP_FOLLOW)
    except ImportError:
      _preap_follow_cache = (None, None)
  bp, v = _preap_follow_cache
  if bp is None:
    return None
  return float(np.interp(v_ego, bp, v))
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5
PREAP_EXP_THROTTLE_OFF = 0.35
PREAP_EXP_THROTTLE_ON = 0.45

# Pre-AP Experimental longitudinal V2.
#
# gasPressProb is useful model context, but it is not the same thing as an
# acceleration limit. Keep the raw model permission separate from the actual
# geometry/profile MPC envelope.
PREAP_EXP_RECOVERY_MIN_ERROR = 0.50       # m/s below effective cruise
PREAP_EXP_MODEL_DRIVE_MIN = 0.05          # explicit positive E2E request
PREAP_EXP_MPC_DRIVE_MIN = 0.05            # cruise/MPC must also want propulsion
PREAP_EXP_DISAGREEMENT_CAP = 0.25         # bounded recovery when signals disagree
PREAP_EXP_COAST_ENTRY_RATE = 0.40         # m/s^3, gentle neutral -> coast entry
PREAP_EXP_NEUTRAL_TARGET = 0.05            # suppress tiny +/- accel chatter
PREAP_EXP_NEUTRAL_PREVIOUS = 0.12
MAPPED_STOP_APPROACH_SPEED = 5.0 * CV.MPH_TO_MS
NAV_SNAPSHOT = '/dev/shm/nap_navigation_desire.json'
STOP_SETTINGS = '/data/nap_turn_settings.json'
ROAD_STOP_SNAPSHOT = '/dev/shm/nap_road_stop.json'
MAP_CONTEXT_SNAPSHOT = '/dev/shm/nap_map_context.json'


def normal_model_stop_enabled(path=STOP_SETTINGS):
  try:
    with open(path) as stream:
      raw = stream.read(2049)
    return len(raw) <= 2048 and json.loads(raw).get("normal_model_stops") is True
  except (OSError, ValueError, TypeError, AttributeError):
    return False


def mapped_stop_setting_enabled(path=STOP_SETTINGS):
  try:
    with open(path) as stream:
      raw = stream.read(2049)
    return len(raw) <= 2048 and json.loads(raw).get('mapped_stop_slowdown') is True
  except (OSError, ValueError, TypeError, AttributeError):
    return False


def road_stop_speed_cap(path=ROAD_STOP_SNAPSHOT, settings=STOP_SETTINGS, now=None, v_ego=None):
  """Experimental road-only hint. Reject stale, ambiguous or inaccurate map matches."""
  now = time.monotonic() if now is None else now
  try:
    with open(settings) as stream:
      raw_setting = stream.read(2049)
    if len(raw_setting) > 2048 or json.loads(raw_setting).get('road_stop_slowdown') is not True:
      return None
    with open(path) as stream:
      raw = stream.read(2049)
    if len(raw) > 2048:
      return None
    state = json.loads(raw)
    received, expires = state['received_mono'], state['expires_mono']
    if (type(received) not in (int, float) or type(expires) not in (int, float)
        or not math.isfinite(received+expires) or not received <= now < expires
        or expires-received > 3):
      return None
    quality = state['position_quality']
    for name, limit in (('match_error_m', 8), ('gps_accuracy_m', 8),
                        ('gps_age_s', 1.5), ('heading_error_deg', 20)):
      value = quality[name]
      if (type(value) not in (int, float) or not math.isfinite(value)
          or not 0 <= value <= limit or name == 'gps_accuracy_m' and value == 0):
        return None
    stop = state['mapped_stop_sign']
    distance = stop['distance_m']
    if (not isinstance(stop.get('id'), str) or not stop['id']
        or type(distance) not in (int, float) or not math.isfinite(distance)
        or not -8 <= distance <= 200 or type(v_ego) not in (int, float)
        or not math.isfinite(v_ego) or not 0 <= v_ego <= 25):
      return None
    return math.sqrt(MAPPED_STOP_APPROACH_SPEED**2 + 2*.8*max(0.0, distance-15))
  except (OSError, ValueError, TypeError, KeyError, AttributeError, RecursionError):
    return None



def map_curve_speed_cap(path=MAP_CONTEXT_SNAPSHOT, now=None, v_ego=None, consumed_stops=(), include_stops=True, preview_distance=0.0, diagnostic=None):
  """Fail-open map-context cruise ceiling.

  Uses the complete mapped curve corridor rather than one curve.
  Map information can only lower cruise. It never raises cruise,
  commands steering, or authorizes proceeding through an intersection.
  """
  def reject(reason):
    if diagnostic is not None:diagnostic['reason'] = reason
    return None
  if diagnostic is not None:diagnostic.clear()
  now = time.monotonic() if now is None else now
  if type(preview_distance) not in (int, float) or not math.isfinite(preview_distance) or not 0. <= preview_distance <= 200.:
    return reject('INVALID_PREVIEW')

  try:
    with open(path) as stream:
      raw = stream.read(65537)

    if len(raw) > 65536:
      return reject('SNAPSHOT_OVERSIZE')

    state = json.loads(raw)
    if diagnostic is not None and isinstance(state, dict):
      diagnostic['snapshot_status'] = state.get('status')
      diagnostic['source'] = state.get('source')

    if (
      state.get('version') not in (2, 3)
      or state.get('enabled') is not True
      or state.get('valid') is not True
      or state.get('control_valid') is not True
      or state.get('status') != 'ok'
    ):
      return reject('SNAPSHOT_DISABLED_OR_INVALID')

    received = state['mono_time']
    expires = state['expires_mono']

    if (
      type(received) not in (int, float)
      or type(expires) not in (int, float)
      or not math.isfinite(received + expires)
      or not received <= now < expires
      or expires - received > 3.0
    ):
      return reject('SNAPSHOT_STALE')

    if (
      type(v_ego) not in (int, float)
      or not math.isfinite(v_ego)
      or v_ego < 0.0
      or v_ego > 45.0
    ):
      return reject('INVALID_SPEED')

    match = state['match']

    match_error = match['distance_m']
    heading_error = match['heading_error_deg']
    confidence = match['confidence']

    for value in (
      match_error,
      heading_error,
      confidence,
    ):
      if (
        type(value) not in (int, float)
        or not math.isfinite(value)
      ):
        return reject('INVALID_MATCH')

    if (
      match_error < 0.0
      or match_error > 20.0
      or heading_error < 0.0
      or heading_error > 35.0
      or confidence < 0.55
      or confidence > 1.0
    ):
      return reject('MATCH_REJECTED')

    lookahead = state['lookahead_m']

    if (
      type(lookahead) not in (int, float)
      or not math.isfinite(lookahead)
      or lookahead < 80.0
      or lookahead > 550.0
    ):
      return reject('INVALID_LOOKAHEAD')

    # V3 publishes every relevant curve.
    # Fall back to the V2 single curve during transition.
    curves = state.get(
      'control_curves'
    )

    if not isinstance(curves, list):
      curve = state.get(
        'control_curve'
      )

      curves = (
        [curve]
        if isinstance(curve, dict)
        else []
      )

    caps = []
    # Map geometry is published at 2 Hz. Propagate its brief age at the
    # measured speed so approach/exit distances evolve at planner rate.
    # This does not extend the snapshot lifetime or lower its quality gates.
    traveled = v_ego * (now - received)

    # ------------------------------------------------------------
    # CURVE CORRIDOR
    #
    # Work backwards from every mapped curve target.
    # The minimum envelope naturally chains consecutive bends and
    # avoids accelerating between closely spaced curves.
    # ------------------------------------------------------------

    for curve in curves[:12]:
      if not isinstance(curve, dict):
        continue

      start = curve.get(
        'start_distance_m'
      )

      end = curve.get(
        'end_distance_m'
      )

      length = curve.get(
        'length_m'
      )

      target = curve.get(
        'target_speed_mps'
      )

      curvature = curve.get(
        'curvature'
      )

      values = (
        start,
        end,
        length,
        target,
        curvature,
      )

      if any(
        type(v) not in (int, float)
        or not math.isfinite(v)
        for v in values
      ):
        continue

      if (
        # Keep an active mapped curve authoritative through its exit.
        # start <= 0 < end means the vehicle is inside the curve, so its
        # independently calculated target remains the longitudinal ceiling.
        end - traveled <= 0.0
        or start > 450.0
        or end <= start
        or end > 500.0
        or length < 20.0
        or curvature < 0.0018
        or curvature > 0.12
        or target < 6.7
        or target > 31.0
      ):
        continue

      # Keep only a modest entry buffer. The previous 12 m buffer
      # completed the slowdown too early and made Map Driving Assist
      # feel unnecessarily slow before ordinary road curves.
      distance = max(
        0.0,
        start - traveled - preview_distance - 6.0,
      )

      comfort_decel = 1.2

      cap = math.sqrt(
        target * target
        + 2.0
        * comfort_decel
        * distance
      )

      caps.append(cap)

    # ------------------------------------------------------------
    # SUPERVISED CITY PREPARATION
    #
    # A mapped stop sign may prepare speed only when the user's
    # existing mapped-stop setting is enabled.
    #
    # It deliberately does NOT command a complete stop and does
    # NOT set shouldStop. Live perception / driver supervision
    # remains responsible for the actual intersection.
    # ------------------------------------------------------------

    if include_stops and mapped_stop_setting_enabled():
      events = state.get(
        'events',
        [],
      )

      if isinstance(events, list):
        for event in events[:16]:
          if not isinstance(event, dict):
            continue

          if event.get('type') != 'stop_sign' or str(event.get('node_id')) in consumed_stops:
            continue

          distance = event.get(
            'distance_m'
          )

          if (
            type(distance) not in (int, float)
            or not math.isfinite(distance)
            or distance < 0.0
            or distance > 220.0
          ):
            continue

          approach_target = MAPPED_STOP_APPROACH_SPEED

          cap = math.sqrt(
            approach_target ** 2
            + 2.0
            * 0.8
            * max(
              0.0,
              distance - 15.0,
            )
          )

          caps.append(cap)

    if not caps:
      return reject('NO_APPLICABLE_CURVE')

    # IMPORTANT:
    # Do NOT compare this to current v_ego here.
    #
    # The map ceiling must constrain the future cruise target even
    # if the vehicle is presently slower and accelerating toward it.
    if diagnostic is not None:diagnostic['reason'] = 'AVAILABLE'
    return min(caps)

  except (
    OSError,
    ValueError,
    TypeError,
    KeyError,
    AttributeError,
    RecursionError,
  ):
    return reject('SNAPSHOT_READ_ERROR')

def navigation_speed_cap(path=NAV_SNAPSHOT, now=None, v_ego=None, consumed_stops=()):
  """Conservative cruise cap from fresh, well-matched route geometry."""
  now = time.monotonic() if now is None else now
  try:
    with open(path) as stream:
      raw = stream.read(2049)
    if len(raw) > 2048:
      return None
    state = json.loads(raw)
    if state.get('enabled') is not True or state.get('route_state') not in ('active', 'arrived'):
      return None
    received, expires = state['received_mono'], state['expires_mono']
    if (type(received) not in (int, float) or type(expires) not in (int, float)
        or not math.isfinite(received + expires) or not received <= now < expires
        or expires-received > 3):
      return None
    quality = state['position_quality']
    if state.get('version') == 4 and (quality.get('projection_valid') is not True
        or quality.get('path_ambiguous') is not False
        or type(quality.get('heading_error_deg')) not in (int, float)
        or not 0 <= quality['heading_error_deg'] <= 20
        or quality.get('gps_age_s', 99) + now-received > 1.5):
      return None
    for name, limit in (('match_error_m', 15), ('gps_accuracy_m', 20), ('gps_age_s', 1.5)):
      value = quality[name]
      if (type(value) not in (int, float) or not math.isfinite(value)
          or not 0 <= value <= limit or name == 'gps_accuracy_m' and value == 0):
        return None
    caps = []
    stop = state.get('mapped_stop_sign')
    if isinstance(stop, dict) and stop.get('id') not in consumed_stops and mapped_stop_setting_enabled():
      stop_distance = stop.get('distance_m')
      if (isinstance(stop.get('id'), str) and stop['id'] and
          type(stop_distance) in (int, float) and math.isfinite(stop_distance) and
          -8 <= stop_distance <= 250 and
          type(v_ego) in (int, float) and math.isfinite(v_ego) and 0 <= v_ego <= 25):
        # Advance hint only: roll toward the mapped point, never command a
        # stop or set shouldStop from map data alone.
        caps.append(math.sqrt(MAPPED_STOP_APPROACH_SPEED**2 + 2 * .8 * max(0.0, stop_distance-15)))
    maneuver = state['maneuver']
    kind, distance = maneuver['type'], maneuver['distance_m']
    if type(distance) not in (int, float) or not math.isfinite(distance) or distance < -8 or distance > 300:
      return min(caps) if caps else None
    # Map guidance does not identify stop signs. Only turn, end-of-road and
    # destination geometry contribute; the model still handles actual stops.
    target = {'turn': 7.0, 'end of road': 5.0, 'arrive': 3.0}.get(kind)
    if target is None:
      return min(caps) if caps else None
    if kind != 'arrive' and maneuver.get('modifier') not in ('left', 'right', 'slight left', 'slight right'):
      return min(caps) if caps else None
    buffer = 20.0 if kind != 'arrive' else 10.0
    caps.append(math.sqrt(target * target + 2 * 1.0 * max(0.0, distance - buffer)))
    return min(caps)
  except (OSError, ValueError, TypeError, KeyError, AttributeError, RecursionError):
    return None

# Lookup table for turns
_A_TOTAL_MAX_V = [2.0, 2.5]
_A_TOTAL_MAX_BP = [20., 30.]

def get_max_accel(v_ego):
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    # Raw model gas/throttle probability permission.
    self.allow_throttle = True

    # Effective Pre-AP propulsion decision. In Experimental V2 this may remain
    # True during a narrowly validated recovery even if raw gasPressProb is low.
    self.propulsion_allowed = True
    self._exp_allow_throttle = None
    # Independent Normal-mode hysteresis. Keep the existing personality
    # acceleration envelopes and all lead/stop braking constraints unchanged.
    self._normal_allow_throttle = None

    self.corner_assist = CornerAssist()
    self.map_stop = MapStop()
    self.normal_model_stop = NormalModelStop(dt)
    self.normal_propulsion = NormalPropulsion(dt)
    self.normal_propulsion_recovery = False
    self.cruise_refinement = CruiseRefinement(dt)
    self.cruise_smoother = UnifiedCruiseSmoother()
    self.personality_accel_limit = PersonalityAccelerationLimit()

    self._is_preap = (CP.brand == "tesla" and CP.carFingerprint == "TESLA_MODEL_S_PREAP"
                       and CP.openpilotLongitudinalControl and not CP.pcmCruise)
    self._params = Params()
    self.nap_follow_dist = self._params.get("NAPFollowDistance", return_default=True) if self._is_preap else None
    self.nap_adaptive_accel = self._params.get_bool("NAPAdaptiveAccel") if self._is_preap else False
    self.nap_navigation_maneuvers = self._params.get_bool("NAPNavigationManeuvers") if self._is_preap else False
    self.nap_map_driving_assist = self._params.get_bool("NAPMapDrivingAssist") if self._is_preap else False
    self._frame = 0

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.prev_turn_accel_max = ACCEL_MAX
    self.output_a_target = 0.0
    # NAP_PREDICTIVE_GRADE_V9E: disabled by default.
    self.nap_v9e_grade = PredictiveGradeAssist(dt=self.dt)
    self.nap_v9e_toggle = FileGradeToggle(dt=self.dt)
    self.phantom_brake_counter = 0
    self.output_should_stop = False

    # NAP Experimental longitudinal arbitration.
    #
    # The Tesla drivetrain responds very clearly to transitions across
    # zero acceleration: positive torque -> lift -> regen can feel like
    # three separate pedal inputs. Keep model-only hesitation from
    # repeatedly crossing that boundary while preserving immediate
    # authority for verified leads, stops, speed ceilings and hard braking.
    self.exp_model_brake_frames = 0
    self.exp_long_state = "OFF"

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

  def _experimental_longitudinal_arbiter(
      self,
      model_accel,
      mpc_accel,
      v_cruise,
      driver_cruise_speed,
      v_ego,
      has_lead,
      should_stop_e2e,
      should_stop_mpc,
      map_should_stop,
      force_slow_decel,
      allow_throttle,
      gas_pressed,
      reset_state,
      model_braking_required=False,
      coast_accel=-0.30,
      recovery_weight=1.0,
      follow_recovery_eligible=False,
  ):
    """Tesla-aware Experimental longitudinal arbitration.

    Keep three concepts separate:

      * MPC / geometry acceleration authority
      * desired vehicle acceleration
      * model propulsion permission

    Verified braking, leads, stops and real speed ceilings retain immediate
    authority. A low gasPressProb alone is treated as a coast preference, not
    as a fake negative MPC acceleration ceiling.

    If modeld explicitly requests positive acceleration, MPC agrees, the car is
    materially below the effective cruise target, and no braking constraint is
    active, permit only a small bounded recovery even when gasPressProb is low.
    """

    model_accel = float(model_accel)
    mpc_accel = float(mpc_accel)
    coast_accel = float(coast_accel)
    raw_target = min(model_accel, mpc_accel)

    # Retained for existing diagnostics/tests. V2 intentionally has no
    # persistence timer which can turn weak hesitation into braking.
    self.exp_model_brake_frames = 0
    self.propulsion_allowed = bool(allow_throttle)

    # Preserve the existing pass-through behavior when Adaptive Accel is not
    # the owner, at low speed, on driver gas, or outside active longitudinal
    # control. Legacy no-throttle clipping remains outside this arbiter for
    # those modes.
    if (
      reset_state
      or gas_pressed
      or not self.nap_adaptive_accel
      or v_ego < 5.0
    ):
      self.exp_long_state = "PASS"
      return raw_target

    speed_error = max(
      0.0,
      float(v_cruise) - float(v_ego),
    )

    # A lowered map/navigation/curve target becomes a braking reason only once
    # the car is actually approaching that effective target.
    approaching_ceiling = (
      v_cruise < driver_cruise_speed - 0.30
      and v_ego >= v_cruise - 0.30
    )

    verified_slowdown = (
      model_braking_required
      or has_lead
      or should_stop_e2e
      or should_stop_mpc
      or map_should_stop
      or force_slow_decel
      or approaching_ceiling
      or mpc_accel < -0.20
      or model_accel <= -1.0
    )

    # Raw low throttle probability must never manufacture positive propulsion
    # while a real longitudinal constraint exists. Negative verified requests
    # retain immediate authority.
    if verified_slowdown:
      self.exp_long_state = "VERIFIED"

      # A detected lead is not itself a demand to hold near-zero model
      # acceleration. Both sources must request propulsion; the lead MPC
      # remains the hard upper bound, and all braking/stop gates still win.
      if (follow_recovery_eligible and has_lead and allow_throttle and raw_target > 0.0
          and not model_braking_required and not should_stop_e2e
          and not should_stop_mpc and not map_should_stop
          and not force_slow_decel and not approaching_ceiling
          and speed_error > .30):
        recovery = float(np.interp(speed_error,
          [0., .25, .75, 1.5, 2.5], [0., .04, .14, .28, .45]))
        support = min(.25, float(np.clip(recovery_weight, 0., 1.))
                      * max(0., recovery-model_accel))
        self.exp_long_state = "FOLLOW_RECOVER"
        return float(min(mpc_accel, model_accel + support))

      if raw_target <= 0.0 or allow_throttle:
        self.propulsion_allowed = bool(allow_throttle)
        return raw_target

      # The unusual case where a constraint is active but both accel sources
      # are still positive: remove added acceleration instead of inventing a
      # stronger brake request.
      self.propulsion_allowed = False
      target = min(raw_target, coast_accel)

      return float(min(0.0, target))

    # Existing bounded cruise-recovery envelope.
    recovery = float(np.interp(
      speed_error,
      [0.0, 0.25, 0.75, 1.50, 2.50],
      [0.0, 0.04, 0.14, 0.28, 0.45],
    ))

    # Fade optional recovery with model confidence instead of jumping from
    # positive cruise support to braking at the weak-coast threshold.
    recovery_weight = float(np.clip(recovery_weight, 0.0, 1.0))

    # --------------------------------------------------------------
    # Model-signal disagreement recovery
    # --------------------------------------------------------------
    #
    # gasPressProb can be below threshold at the same instant that the model's
    # explicit desiredAcceleration is positive. Do not let that metadata bit
    # force natural coast when:
    #
    #   * model acceleration is explicitly positive,
    #   * MPC also permits acceleration,
    #   * the car is materially below the effective target, and
    #   * no lead/stop/curve/braking constraint above is active.
    #
    # This is deliberately capped well below the normal accel envelope.
    disagreement_recovery = (
      not allow_throttle
      and speed_error >= PREAP_EXP_RECOVERY_MIN_ERROR
      and model_accel > PREAP_EXP_MODEL_DRIVE_MIN
      and mpc_accel > PREAP_EXP_MPC_DRIVE_MIN
    )

    if disagreement_recovery:
      self.propulsion_allowed = True
      self.exp_long_state = "RECOVER_LIMITED"

      target = min(
        mpc_accel,
        max(model_accel, recovery),
        PREAP_EXP_DISAGREEMENT_CAP,
      )

      # The normal Pre-AP positive-accel limiter below still controls the
      # actual rise rate. This cap is only arbitration authority.
      return float(target)

    # --------------------------------------------------------------
    # Normal raw-permission path
    # --------------------------------------------------------------

    if allow_throttle:
      self.propulsion_allowed = True
      target = min(
        mpc_accel,
        model_accel + min(0.25, recovery_weight * max(0.0, recovery - model_accel)),
      )

      self.exp_long_state = (
        "RECOVER"
        if speed_error > 0.30
        else "HOLD"
      )

      # The P85 makes tiny drive/coast/regen sign changes unusually obvious.
      # Neutralize only small unverified chatter; never mask real braking.
      if (
        abs(target) < PREAP_EXP_NEUTRAL_TARGET
        and abs(self.output_a_target) < PREAP_EXP_NEUTRAL_PREVIOUS
      ):
        target = 0.0

      return float(target)

    # --------------------------------------------------------------
    # Genuine model coast request
    # --------------------------------------------------------------
    #
    # No verified brake exists and the positive-model disagreement exception
    # did not qualify. Move toward the calibrated natural-coast acceleration,
    # but do not jump directly from positive drive torque into regen.
    self.propulsion_allowed = False
    self.exp_long_state = "COAST"

    target = min(
      raw_target,
      coast_accel,
    )

    return float(min(0.0, target))


  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm):
    self._frame += 1
    self.corner_assist.refresh()
    if self._is_preap and self._frame % 20 == 0:
      self.nap_follow_dist = self._params.get("NAPFollowDistance", return_default=True)
      self.nap_adaptive_accel = self._params.get_bool("NAPAdaptiveAccel")
      self.nap_navigation_maneuvers = self._params.get_bool("NAPNavigationManeuvers")
      self.nap_map_driving_assist = self._params.get_bool("NAPMapDrivingAssist")

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
      experimental_coast_accel = accel_coast
    else:
      # Preserve legacy behavior for Normal mode while giving Experimental V2
      # a sane no-pitch fallback instead of treating ACCEL_MAX as "coast".
      accel_coast = ACCEL_MAX
      experimental_coast_accel = get_coast_accel(0.0)

    v_ego = sm['carState'].vEgo
    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    # Always rebuild ceilings from the driver setting, never prior slowed speed.
    driver_cruise_speed = v_cruise_kph * CV.KPH_TO_MS
    v_cruise = driver_cruise_speed
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    # PCM cruise speed may be updated a few cycles later, check if initialized
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    personality = sm["selfdriveState"].personality
    accel_max = accel_ceiling(v_ego, personality) if self._is_preap else get_max_accel(v_ego)
    if self._is_preap:
      accel_max = self.personality_accel_limit.update(
        accel_max, personality, self.dt,
        reset=reset_state or sm['carState'].gasPressed or sm['carState'].standstill
        or not self.nap_adaptive_accel)
    accel_clip = [ACCEL_MIN, accel_max]
    steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
    accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    _, _, _, _, throttle_prob = self.parse_model(sm['modelV2'])
    was_effective_permission = self.propulsion_allowed

    # Raw model permission remains available for diagnostics and legacy modes.
    # Experimental V2 computes a separate effective propulsion decision later.
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    self.propulsion_allowed = self.allow_throttle

    experimental_preap = (
      self._is_preap
      and sm['selfdriveState'].experimentalMode
      and self.nap_adaptive_accel
    )

    # Keep the raw 0.4 permission for diagnostics and legacy behavior. Only
    # active Experimental V2 gets hysteresis; braking authority below remains
    # independent of this preference and bypasses the final comfort limiter.
    effective_allow_throttle = self.allow_throttle
    if experimental_preap and not reset_state and not sm['carState'].gasPressed and v_ego >= 5.0:
      if self._exp_allow_throttle is None:
        self._exp_allow_throttle = self.allow_throttle
      else:
        threshold = PREAP_EXP_THROTTLE_OFF if self._exp_allow_throttle else PREAP_EXP_THROTTLE_ON
        self._exp_allow_throttle = math.isfinite(throttle_prob) and throttle_prob > threshold
      effective_allow_throttle = self._exp_allow_throttle
    else:
      self._exp_allow_throttle = None

    # The legacy single-threshold Normal permission can repeatedly switch
    # between propulsion and a negative coast ceiling on winding roads.
    # Match Experimental's narrow hysteresis; do not change allowed accel.
    if (self._is_preap and self.nap_adaptive_accel
        and not sm['selfdriveState'].experimentalMode
        and not reset_state and not sm['carState'].gasPressed
        and v_ego >= 5.0):
      if self._normal_allow_throttle is None:
        self._normal_allow_throttle = self.allow_throttle
      else:
        threshold = PREAP_EXP_THROTTLE_OFF if self._normal_allow_throttle else PREAP_EXP_THROTTLE_ON
        self._normal_allow_throttle = math.isfinite(throttle_prob) and throttle_prob > threshold
      effective_allow_throttle = self._normal_allow_throttle
    else:
      self._normal_allow_throttle = None

    # One recovery-permission history across Chill/Experimental. This does
    # not replace either mode's model/MPC/stop selection or final handoff.
    was_normal_recovery = self.normal_propulsion_recovery
    self.normal_propulsion_recovery = self.normal_propulsion.update(
      sm['modelV2'], v_ego, driver_cruise_speed,
      enabled=self._is_preap and self.nap_adaptive_accel,
      valid=sm.all_checks(service_list=['modelV2', 'carState', 'radarState']),
      blocked=reset_state or sm['carState'].gasPressed or getattr(sm['carState'], 'brakePressed', False)
      or getattr(sm['carState'], 'brake', 0.0) > 0.0
      or force_slow_decel or self.normal_model_stop.active
      or sm['radarState'].leadOne.status
      or bool(getattr(getattr(sm['radarState'], 'leadTwo', None), 'status', False))
      or sm['modelV2'].action.shouldStop
      # The preceding map release remains a real speed restriction until
      # it unwinds; don't resurrect model-only propulsion across that handoff.
      or (self.cruise_refinement.map_cap is not None
          and self.cruise_refinement.map_cap < driver_cruise_speed - 0.30))
    effective_allow_throttle = effective_allow_throttle or self.normal_propulsion_recovery
    self.propulsion_allowed = effective_allow_throttle

    if self._is_preap:
      if self.normal_propulsion_recovery and not was_normal_recovery and not self.allow_throttle:
        self.prev_accel_clip[1] = self.prev_turn_accel_max
      elif experimental_preap and self.prev_accel_clip[1] < 0.0:
        # Entering V2 must not inherit a legacy temporary coast ceiling.
        self.prev_accel_clip[1] = self.prev_turn_accel_max
      elif not experimental_preap and effective_allow_throttle and not was_effective_permission:
        # A new effective Normal permission, not a raw 0.4 threshold crossing,
        # ends a temporary coast ceiling. The downstream positive-accel ramp
        # still controls propulsion recovery.
        self.prev_accel_clip[1] = self.prev_turn_accel_max

    self.prev_turn_accel_max = accel_clip[1]

    # Smooth curve exit: restrict how fast the acceleration cap can increase.
    # 0.025 per frame (at 20Hz) = 0.5 m/s^2 per second.
    if not reset_state and accel_clip[1] > self.prev_accel_clip[1]:
      accel_clip[1] = min(accel_clip[1], self.prev_accel_clip[1] + 0.025)
    self.prev_accel_clip = list(accel_clip)

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    if not effective_allow_throttle and not (self._is_preap and self.nap_adaptive_accel):
      # Legacy behavior with adaptive control disabled. Adaptive coast requests
      # are shaped after selection, independently of physical MPC limits.
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED*2], [accel_clip[1], clipped_accel_coast])
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    map_cap, map_should_stop = None, False
    if self._is_preap:
      map_cap, map_should_stop = self.map_stop.update(
        read_snapshot(NAV_SNAPSHOT), read_snapshot("/dev/shm/nap_brake_resume.json"),
        enabled=self.nap_map_driving_assist and mapped_stop_setting_enabled(), active=not reset_state,
        inputs_valid=sm.all_checks(service_list=["carState", "controlsState", "selfdriveState"]),
        v_ego=v_ego, gas=sm["carState"].gasPressed, now=time.monotonic())
      if map_cap is not None:
        v_cruise = min(v_cruise, map_cap)

    map_diagnostic = {'reason': 'DISABLED_OR_INACTIVE'}
    curve_cap = None
    curve_recovery_limit = None
    if self._is_preap and not reset_state:
      # Route navigation and generic map assistance are intentionally separate.
      #
      # Navigation Maneuvers:
      #   Uses the active route snapshot for maneuver-aware speed preparation.
      #
      # Map Driving Assist:
      #   Uses independent mapped-road context even when no route is active.
      #
      # Both are speed CEILINGS only. Neither path may raise the driver's
      # cruise target or override a more restrictive planner/radar/model limit.
      if self.nap_navigation_maneuvers:
        nav_cap = navigation_speed_cap(v_ego=v_ego, consumed_stops=self.map_stop.consumed)
        if nav_cap is not None:
          v_cruise = min(v_cruise, nav_cap)

      if self.nap_map_driving_assist:
        # Curves retain their existing ceiling. Direction-matched route stops
        # use the separate driver-brake / acknowledged-resume lifecycle.
        curve_cap = map_curve_speed_cap(v_ego=v_ego, include_stops=False, diagnostic=map_diagnostic)
        if self.nap_adaptive_accel and not sm['carState'].gasPressed:
          # Evaluate geometry farther along the same fresh corridor, without
          # changing its timestamp, quality gates, or current braking ceiling.
          horizon = 3.0
          preview_distance = min(200., v_ego * horizon + .5 * max(0., accel_clip[1]) * horizon**2)
          future_cap = map_curve_speed_cap(v_ego=v_ego, include_stops=False,
                                           preview_distance=preview_distance)
          curve_recovery_limit = curve_recovery_accel_limit(future_cap, v_ego, horizon)
        preparation_cap = map_curve_speed_cap(v_ego=v_ego, consumed_stops=self.map_stop.consumed)
        # The preparation snapshot includes curves and optional mapped stops.
        # Previously its independent v_cruise assignment bypassed the release
        # smoother for preparation-only caps (and duplicated the curve path).
        # Tightening stays immediate; release now uses one speed-target path.
        if preparation_cap is not None:
          curve_cap = preparation_cap if curve_cap is None else min(curve_cap, preparation_cap)

    if self._is_preap:
      curve_target = self.cruise_refinement.curve_target(
        curve_cap, driver_cruise_speed, v_ego,
        self.nap_map_driving_assist and not reset_state and not sm['carState'].gasPressed)
      v_cruise = min(v_cruise, curve_target)

    # One city-turn speed ceiling feeds the existing MPC in both modes.
    # Pedal ownership and engagement are evaluated inside prepare().
    city_turn_cap = None
    turn_decision = None
    if self._is_preap:
      turn_decision = read_snapshot("/dev/shm/nap_navigation_decision.json")
      city_turn_cap = self.corner_assist.prepare(sm, self.CP, reset_state, driver_cruise_speed, turn_decision)
      if type(city_turn_cap) in (int, float) and math.isfinite(city_turn_cap) and 0. < city_turn_cap <= driver_cruise_speed:
        v_cruise = min(v_cruise, city_turn_cap)
      else:
        city_turn_cap = None

    # Optional Pre-AP follow cap. The NAPAdaptiveAccel toggle is the master
    # gate; when disabled, personality and the normal MPC limits apply.
    if (self._is_preap and self.nap_adaptive_accel
        and (sm['radarState'].leadOne.status
             or bool(getattr(getattr(sm['radarState'], 'leadTwo', None), 'status', False)))):
      follow_limit = _get_preap_follow_limit(v_ego)
      if follow_limit is not None:
        accel_clip[1] = min(accel_clip[1], follow_limit)

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    self.mpc.update(sm['radarState'], v_cruise, personality=sm['selfdriveState'].personality, nap_follow_dist=self.nap_follow_dist, max_accel=accel_clip[1],
                    preap_accel_profile=self._is_preap)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t =  self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
                                                                        action_t=action_t, vEgoStopping=self.CP.vEgoStopping)

    if not self._is_preap:
      # Phantom braking buffer
      if output_a_target_mpc < self.output_a_target - 0.5:
        self.phantom_brake_counter += 1
        if self.phantom_brake_counter < 5:
          output_a_target_mpc = max(output_a_target_mpc, self.output_a_target - 0.5)
      else:
        self.phantom_brake_counter = 0

    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop
    normal_stop_active = self.normal_model_stop.update(
      sm['modelV2'], v_ego, enabled=self._is_preap
      and not sm['selfdriveState'].experimentalMode and normal_model_stop_enabled(),
      valid=sm.all_checks(service_list=['modelV2', 'carState']),
      reset=reset_state or sm['carState'].gasPressed)

    # Normal retains its existing support floor. Experimental recovery is
    # owned solely by the arbiter; never bias the model before arbitration.
    # Weak model coasting may receive bounded support; braking predictions
    # and either lead's constraints retain authority.
    # Normal cruise uses a floor, not another additive acceleration loop.
    if self._is_preap:
      lead_two = getattr(sm['radarState'], 'leadTwo', None)
      forecast = assess_model_forecast(sm['modelV2'], v_ego)
      model_intent = assess_model_intent(sm['modelV2'], v_ego, output_a_target_e2e, forecast)
      model_weight = model_intent.support_weight
      support = self.cruise_refinement.speed_support(
        v_cruise, v_ego, sm['carState'].aEgo,
        self.nap_adaptive_accel and not sm['selfdriveState'].experimentalMode
        and not reset_state and not sm['carState'].gasPressed
        and effective_allow_throttle and not force_slow_decel and v_ego >= 5.0
        and not sm['radarState'].leadOne.status and not (lead_two is not None and lead_two.status)
        and not output_should_stop_mpc and not output_should_stop_e2e and not normal_stop_active and not map_should_stop
        and output_a_target_mpc >= 0.0 and model_weight > 0.0)
      if not sm['selfdriveState'].experimentalMode and support > 0.0:
        output_a_target_mpc = max(output_a_target_mpc, support)

    if sm['selfdriveState'].experimentalMode:
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

      if self._is_preap:
        has_lead = (
          sm['radarState'].leadOne.status
          or (
            lead_two is not None
            and lead_two.status
          )
        )

        output_a_target = self._experimental_longitudinal_arbiter(
          output_a_target_e2e,
          output_a_target_mpc,
          v_cruise,
          driver_cruise_speed,
          v_ego,
          has_lead,
          output_should_stop_e2e,
          output_should_stop_mpc,
          map_should_stop,
          force_slow_decel,
          effective_allow_throttle,
          sm['carState'].gasPressed,
          reset_state,
          coast_accel=max(experimental_coast_accel, accel_clip[0]),
          recovery_weight=model_weight,
          follow_recovery_eligible=(forecast.valid and not forecast.near_braking
                                    and sm.all_checks(service_list=['carState', 'modelV2', 'radarState'])),
          # A missing lead is not evidence that visual braking is phantom.
          # Only weak coasting with a validated speed-holding forecast may
          # receive the existing bounded compensation.
          model_braking_required=model_intent.braking_required,
        )

        if (
          self.exp_long_state in ("VERIFIED", "PASS", "COAST")
          and output_a_target
              < output_a_target_mpc - 0.01
        ):
          self.mpc.source = LongitudinalPlanSource.e2e

      else:
        output_a_target = min(
          output_a_target_e2e,
          output_a_target_mpc,
        )

        if output_a_target < output_a_target_mpc:
          self.mpc.source = LongitudinalPlanSource.e2e

    else:
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
      if normal_stop_active:
        # Share Experimental's restrictive model/MPC selection only while
        # a model stop is acquired; Normal cruise remains MPC-owned.
        output_a_target = min(output_a_target_mpc, output_a_target_e2e, 0.0)
        self.output_should_stop = output_should_stop_mpc or output_should_stop_e2e
        if output_a_target < output_a_target_mpc:
          self.mpc.source = LongitudinalPlanSource.e2e

      if self._is_preap:
        self.exp_model_brake_frames = 0
        self.exp_long_state = "OFF"

    if (self._is_preap and self.nap_adaptive_accel
        and not sm['selfdriveState'].experimentalMode and not effective_allow_throttle):
      # A model coast preference is a request, not a hard physical ceiling.
      # Keep its existing speed dependence while using the shared handoff.
      coast_cap = np.interp(v_ego, [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
                            [accel_clip[1], max(experimental_coast_accel, accel_clip[0])])
      output_a_target = min(output_a_target, coast_cap)

    # Release removes only map authority; model/MPC stop requests remain intact.
    self.output_should_stop = self.output_should_stop or map_should_stop

    # CornerAssist is the sole gate for NAP turn-related longitudinal
    # behavior. Do not apply the old blinker-only -2.5 m/s^2 override: it
    # ignored path geometry and remained active when Corner Assist was off.
    corner_braking = False
    if self._is_preap:
      before_corner = output_a_target
      output_a_target = self.corner_assist.apply(output_a_target, sm, self.CP, reset_state, accel_coast, driver_cruise_speed,
                                                       turn_decision, True)

      corner_braking = output_a_target < min(0., before_corner)

    for idx in range(2):
      accel_clip[idx] = np.clip(accel_clip[idx], self.prev_accel_clip[idx] - 0.05, self.prev_accel_clip[idx] + 0.05)
    if self._is_preap:
      previous = 0.0 if reset_state or sm["carState"].gasPressed else self.output_a_target
      # Physical/profile envelopes remain hard bounds. Request selection and
      # CornerAssist are complete before this single source-independent handoff.
      accel_clip[1] = min(accel_clip[1], accel_max)
      self.handoff_request = float(output_a_target)
      self.handoff_reason = 'OFF'
      has_lead = (bool(sm['radarState'].leadOne.status)
                  or bool(getattr(lead_two, 'status', False)))
      handoff_active = (self.nap_adaptive_accel and not reset_state
                        and not sm['carState'].gasPressed
                        and not getattr(sm['carState'], 'brakePressed', False))
      if handoff_active:
        valid = sm.all_checks(service_list=['carState', 'modelV2', 'radarState'])
        if sm['selfdriveState'].experimentalMode:
          self.handoff_reason = braking_priority(
            sm['modelV2'], v_ego, output_a_target, output_a_target_mpc,
            has_lead, self.output_should_stop, force_slow_decel,
            v_cruise, driver_cruise_speed, self.corner_assist.target_speed,
            valid=valid, forecast=forecast)
        elif self.output_should_stop or normal_stop_active:
          self.handoff_reason = 'STOP'
        elif force_slow_decel:
          self.handoff_reason = 'FORCE_DECEL'
        elif not valid:
          self.handoff_reason = 'INVALID_INPUT'
        elif has_lead and output_a_target < 0.:
          self.handoff_reason = 'LEAD_BRAKING'
        elif output_a_target_mpc <= -1.:
          self.handoff_reason = 'STRONG_BRAKING'
        elif output_a_target_mpc < 0. and v_ego >= v_cruise - .3:
          self.handoff_reason = 'SPEED_CEILING'
        else:
          self.handoff_reason = 'ORDINARY'
        if has_lead and self.handoff_reason == 'ORDINARY':
          self.handoff_reason = 'LEAD_CONSTRAINT'
        if corner_braking:
          self.handoff_reason = 'CORNER_BRAKING'
        # Preview restricts positive recovery only. All existing braking and
        # stop selection has already occurred, and retains priority below.
        if curve_recovery_limit is not None and output_a_target > 0.:
          output_a_target = min(output_a_target, curve_recovery_limit)
        output_a_target = self.cruise_smoother.update(
          output_a_target, previous, self.dt, personality, v_ego, v_cruise,
          sm['carState'].aEgo, accel_clip[0], accel_clip[1], self.handoff_reason,
          capture=effective_allow_throttle and not has_lead and not self.output_should_stop
          and not normal_stop_active and not force_slow_decel and not corner_braking)
        self.propulsion_allowed = self.propulsion_allowed or output_a_target > 0.
      else:
        self.cruise_smoother.reset()
        output_a_target = limit_positive_accel(output_a_target, previous, v_ego, personality, self.dt)
      if experimental_preap and self.handoff_request < output_a_target_mpc - .01:
        self.mpc.source = LongitudinalPlanSource.e2e
    # NAP_PREDICTIVE_GRADE_V9E: optional, tiny uphill-only feedforward.
    # Runs after existing planner smoothing and corner arbitration, BEFORE
    # the original final accel_clip. Never moves a limit or speed target.
    # The code below is inert when toggle is OFF (default).
    if self.nap_v9e_toggle.read():
      radar = sm['radarState']
      grade_lead = next((candidate for candidate in
                         (radar.leadOne, radar.leadTwo)
                         if bool(getattr(candidate, 'status', False))), None)
      output_a_target += self.nap_v9e_grade.update(
        model=sm['modelV2'],
        enabled=True,
        engaged=bool(getattr(sm['selfdriveState'], 'enabled', False)) and not reset_state,
        v_ego=v_ego,
        a_ego=sm['carState'].aEgo,
        effective_speed=v_cruise,
        driver_speed=driver_cruise_speed,
        base_accel=output_a_target,
        mpc_accel=output_a_target_mpc,
        model_accel=output_a_target_e2e,
        brake_reason=getattr(self, 'handoff_reason', 'OFF'),
        model_stop=bool(sm['modelV2'].action.shouldStop),
        mpc_stop=bool(output_should_stop_mpc),
        corner_braking=bool(corner_braking),
        force_decel=bool(sm['controlsState'].forceDecel),
        gas_pressed=bool(sm['carState'].gasPressed),
        brake_pressed=bool(sm['carState'].brakePressed),
        lead=grade_lead,
      )
    else:
      self.nap_v9e_grade.reset()
    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip
    if self._is_preap and not reset_state and self._frame % 20 == 0:
      cloudlog.info('NAP long speed=%.2f driver=%.2f effective=%.2f map=%s curve=%.2f model=%.3f mpc=%.3f support=%.3f output=%.3f source=%s state=%s frames=%d throttle=%.3f raw_allow=%d prop=%d cap=%.3f turncap=%.3f model_stop=%d mpc_stop=%d final_stop=%d long_off=%d corner=%s request=%.3f handoff=%s long_active=%d gas=%d brake=%d signal=%d steer=%.2f corner_target=%s weight=%.2f aego=%.3f experimental=%d personality=%d forecast_stop=%d preview=%s recovery=%d recovery_reason=%s map_reason=%s map_status=%s map_source=%s turn_phase=%s turn_owner=%s turn_ceiling=%s model_intent=%s unified=9',
                    v_ego, driver_cruise_speed, v_cruise, curve_cap, curve_target,
                    sm['modelV2'].action.desiredAcceleration, output_a_target_mpc,
                    support * model_weight, self.output_a_target, self.mpc.source,
                    self.exp_long_state, self.exp_model_brake_frames, throttle_prob,
                    self.allow_throttle, self.propulsion_allowed,
                    accel_clip[1], self.prev_turn_accel_max,
                    output_should_stop_e2e, output_should_stop_mpc, self.output_should_stop,
                    reset_state, self.corner_assist.reason, self.handoff_request, self.handoff_reason,
                    bool(getattr(sm['carControl'], 'longActive', False)), sm['carState'].gasPressed,
                    bool(getattr(sm['carState'], 'brakePressed', False)),
                    int(bool(getattr(sm['carState'], 'leftBlinker', False)))
                    + 2 * int(bool(getattr(sm['carState'], 'rightBlinker', False))),
                    sm['carState'].steeringAngleDeg, self.corner_assist.target_speed,
                    model_weight, sm['carState'].aEgo, sm['selfdriveState'].experimentalMode,
                    int(getattr(personality, 'raw', personality)), forecast.planned_stop, curve_recovery_limit, self.normal_propulsion_recovery, self.normal_propulsion.reason,
                    map_diagnostic.get("reason"), map_diagnostic.get("snapshot_status"), map_diagnostic.get("source"), self.corner_assist.turn_plan.phase, self.corner_assist.turn_plan.owner, city_turn_cap, model_intent.reason)

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState', 'modelV2'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime - sm.logMonoTime['modelV2']) / 1e9
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = (sm['radarState'].leadOne.status
                                or bool(getattr(getattr(sm['radarState'], 'leadTwo', None), 'status', False)))
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.propulsion_allowed)

    pm.send('longitudinalPlan', plan_send)
