"""Experimental acceleration cap; never overrides a stronger planner slowdown."""
import json
import math
import time

from openpilot.selfdrive.controls.lib.turn_context import driver_braking, paired_decision, manual_turn_intent, city_turns_enabled
from openpilot.selfdrive.controls.lib.curve_comfort import PROFILES, DEFAULT_PROFILE, read_curve_comfort

from openpilot.selfdrive.controls.lib.turn_speed_plan import TurnSpeedPlan, path_turn_target
from openpilot.selfdrive.controls.lib.city_turn_plan import paired_turn_plan, profile_target
from openpilot.selfdrive.controls.lib.turn_context import navigation_request_valid, navigation_signal_matches

SETTINGS = '/data/nap_turn_settings.json'

LAT_ACCEL_TARGET = PROFILES[DEFAULT_PROFILE].corner_lateral_accel
MIN_CURVATURE = .008
CAP_JERK = .4


def settings(path=SETTINGS):
  defaults = {'corner_assist': False, 'blinker_braking': True}
  try:
    with open(path) as f:
      obj = json.loads(f.read(2049))
    for key in defaults:
      if type(obj.get(key)) is bool:
        defaults[key] = obj[key]
  except (OSError, ValueError, TypeError, AttributeError):
    pass
  return defaults


def path_curve(xs, ys, distance):
  """Return maximum usable three-point curvature in the near forward path."""
  if len(xs) != len(ys) or not 3 <= len(xs) <= 64:
    return None
  points = list(zip(xs, ys))
  if any(not math.isfinite(v) for p in points for v in p):
    return None
  curves = []
  for a, b, c in zip(points, points[1:], points[2:]):
    if not (0 <= a[0] < b[0] < c[0] <= distance):
      continue
    ab = math.hypot(b[0]-a[0], b[1]-a[1])
    bc = math.hypot(c[0]-b[0], c[1]-b[1])
    ac = math.hypot(c[0]-a[0], c[1]-a[1])
    if min(ab, bc, ac) < .5:
      continue
    cross = abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
    curves.append(2*cross/(ab*bc*ac))
  return max(curves) if curves else None


class CornerAssist:
  def __init__(self):
    self.config = {'corner_assist': False, 'blinker_braking': True}
    self.last_config = -math.inf
    self.cap = None
    self.last_time = None
    self.reason = 'Corner assistance off'
    self.target_speed = None
    self.comfort = PROFILES[DEFAULT_PROFILE]
    self.turn_plan = TurnSpeedPlan()

  def refresh(self, now=None):
    now = time.monotonic() if now is None else now
    if now-self.last_config >= 1 or now < self.last_config:
      self.config = settings()
      self.comfort = read_curve_comfort()
      if not hasattr(self, 'params'):
        from openpilot.common.params import Params
        self.params = Params()
      self.last_config = now
    # City Turns replaces the old city-corner/propulsion enable split.
    self.config['corner_assist'] = city_turns_enabled(self.params)

  def update(self, base, speed, curvature, coast, active, now, cruise_speed=None, target_override=None):
    """City-speed experiment, with limited additional deceleration and jerk.

    This caps acceleration, not braking authority. A lead/stop/model request
    below the cap passes through unchanged. It cannot guarantee a safe speed.
    """
    previous_target = self.target_speed
    self.target_speed = None
    valid_cruise = (cruise_speed is None or
                    (math.isfinite(cruise_speed) and cruise_speed >= 0.))
    valid = (valid_cruise and self.config['corner_assist'] and active and curvature is not None
             and all(math.isfinite(v) for v in (base, speed, curvature, coast, now))
             and .3 <= speed <= 15.65 and 0 <= curvature <= .25)
    dt = now-self.last_time if self.last_time is not None else .05
    self.last_time = now
    if not valid or not 0 < dt <= .25:
      self.cap = None
      self.reason = 'Off or vehicle/model input unavailable; normal planner'
      return base
    if curvature < MIN_CURVATURE:
      # There is no curve authority to carry into the next bend.
      self.cap = None
      self.reason = 'Normal planner'
      return base

    # An absolute geometry target, capped by the driver's unmodified cruise
    # setting. Neither current speed nor the previous curve output is a new
    # cruise reference. Keep stronger lead/stop/model requests via min(base,...).
    self.target_speed = math.sqrt(self.comfort.corner_lateral_accel / curvature)
    if cruise_speed is not None:
      self.target_speed = min(self.target_speed, cruise_speed)
    if target_override is not None:
      self.target_speed = min(self.target_speed, target_override) if cruise_speed is None else min(cruise_speed, target_override)
    if target_override is None and previous_target is not None and self.target_speed > previous_target:
      # Release only curve-owned authority progressively. Do not preserve an
      # obsolete target below current speed once geometry permits recovery.
      self.target_speed = min(self.target_speed,
                              max(previous_target, min(speed, self.target_speed)) + .6 * dt)
    approach_distance = max(8., speed * 2.5)
    target_cap = min(max(0., base), max(-self.comfort.corner_max_decel,
                     (self.target_speed**2 - speed**2) / (2 * approach_distance)))
    if self.cap is not None:
      # A milder selection must not inherit stronger optional curve braking.
      self.cap = max(-self.comfort.corner_max_decel, self.cap)
    if self.cap is None:
      # Do not memorize a lead/stop deceleration as curve-owned braking.
      self.cap = max(0., base)
    # Never bank unused positive authority that delays a tighter next bend.
    self.cap = min(self.cap, max(0., base))
    self.cap += max(-CAP_JERK * dt, min(CAP_JERK * dt, target_cap - self.cap))
    if speed <= self.target_speed:
      # Old code forced coasting below target and its jerk filter prolonged it.
      # At/below target, release curve braking immediately; positive recovery
      # remains bounded by the curve target and the normal planner.
      self.cap = max(0., self.cap)
      self.reason = 'Curve target reached; no additional braking'
    else:
      self.reason = 'Above absolute curve target; gradual slowdown'
    return min(base, self.cap, target_cap) if target_cap >= 0. else min(base, self.cap)

  def prepare(self, sm, CP, reset, cruise_speed, decision=None):
    """Prepare the same geometry ceiling for MPC and the final corner cap.

    Pedals change longitudinal ownership, not lateral turn intent. No speed
    authority is granted while either pedal is touched or long is disengaged.
    """
    now = time.monotonic()
    if isinstance(decision,dict) and 'city_turn_plan' in decision and decision.get('signal_owner') in ('driver','navigation'):
      return self.prepare_shared(sm, CP, reset, cruise_speed, decision, now)
    try:
      cs, cc, model = sm['carState'], sm['carControl'], sm['modelV2']
      fresh = all(sm.valid[k] and sm.alive[k] and 0 <= now-sm.logMonoTime[k]/1e9 <= .5
                  for k in ('carState', 'carControl', 'modelV2'))
      signaled = bool(cs.leftBlinker) != bool(cs.rightBlinker)
      turn_intent = (manual_turn_intent(cs, decision, model, now, approach=True)
                     and (decision.get('manual_turn_phase') in (None, 'TURN', 'EXIT')
                          or decision.get('turn_locked') is True))
      # Completion clears steering/model intent immediately, while the already
      # active speed ceiling releases through the same bounded speed planner.
      if (paired_decision(decision, model, now) and decision.get('manual_turn_phase') == 'COMPLETE'
          and self.turn_plan.target is not None and self.config['corner_assist']
          and fresh and cs.canValid and cc.latActive and not cs.steeringPressed
          and str(cs.gearShifter) == 'drive' and .3 <= cs.vEgo <= 15.65):
        # Completion ends the maneuver, not the exit road's curvature.
        if CP.steerRatio <= 0 or CP.wheelbase <= 0:
          self.turn_plan.reset()
          return None
        actual = abs(math.tan(math.radians(cs.steeringAngleDeg)/CP.steerRatio)/CP.wheelbase)
        target = path_turn_target(model.position.x, model.position.y,
                                  self.comfort.corner_lateral_accel, self.comfort.corner_max_decel, cruise_speed)
        if target is not None and actual >= MIN_CURVATURE:
          target = min(target, math.sqrt(self.comfort.corner_lateral_accel/actual))
        return self.turn_plan.update(target, cs.vEgo, actual, cruise_speed, now, True,
                                     driver_braking(cs) or cs.gasPressed, cc.longActive and not reset)
      nav_turn = (paired_decision(decision, model, now) and decision.get("source") == "navigation"
                  and decision.get("confirmed") is True and decision.get("final_desire") in ("turnLeft", "turnRight"))
      eligible = (self.config['corner_assist'] and (turn_intent or nav_turn) and signaled and fresh
                  and cs.canValid and cc.latActive and not cs.steeringPressed
                  and str(cs.gearShifter) == 'drive' and .3 <= cs.vEgo <= 40*.44704
                  and not getattr(cs, 'steeringDisengage', False)
                  and CP.steerRatio > 0 and CP.wheelbase > 0)
      if not eligible:
        self.turn_plan.reset()
        return None
      identity = (decision.get('signal_owner'), decision.get('source'),
                  decision.get('manual_turn_direction'), decision.get('route_id'), decision.get('maneuver_id'))
      if identity != getattr(self, 'turn_identity', None):
        self.turn_plan.reset()
      self.turn_identity = identity
      actual = abs(math.tan(math.radians(cs.steeringAngleDeg)/CP.steerRatio)/CP.wheelbase)
      target = path_turn_target(model.position.x, model.position.y,
                               self.comfort.corner_lateral_accel, self.comfort.corner_max_decel, cruise_speed)
      if target is not None and actual >= MIN_CURVATURE:
        target = min(target, math.sqrt(self.comfort.corner_lateral_accel/actual))
      return self.turn_plan.update(target, cs.vEgo, actual, cruise_speed, now, eligible,
                                   driver_braking(cs) or cs.gasPressed, cc.longActive and not reset, locked=decision.get('turn_locked') is True)
    except (AttributeError, ValueError, TypeError, KeyError, OverflowError):
      self.turn_plan.reset()
      return None

  def prepare_shared(self, sm, CP, reset, cruise_speed, decision, now):
    """Consume one acquired profile; do not infer another turn lifecycle."""
    try:
      cs,cc,model=sm['carState'],sm['carControl'],sm['modelV2']
      plan=paired_turn_plan(decision,model,now)
      fresh=all(sm.valid[k] and sm.alive[k] and 0<=now-sm.logMonoTime[k]/1e9<=.35
                for k in ('carState','carControl','modelV2'))
      nav=decision.get('navigation_turn') or {}
      nav_request=nav.get('request') if isinstance(nav,dict) else None
      nav_owned=(decision.get('signal_owner')=='navigation' and navigation_request_valid(nav_request,now,allow_release=True)
                 and plan is not None and plan.get('owner_key')==nav_request['key']
                 and decision.get('route_id')==nav_request['route_id']
                 and decision.get('maneuver_id')==nav_request['maneuver_id'])
      life=nav.get('lifecycle',{}) if nav_owned else {}
      eligible=(plan is not None and paired_decision(decision,model,now)
                and self.config['corner_assist'] and decision.get('city_turns_enabled') is True
                and fresh and cs.canValid and cc.latActive and cc.longActive and not reset
                and not cs.steeringPressed and not getattr(cs,'steeringDisengage',False)
                and not driver_braking(cs) and not cs.gasPressed
                and str(cs.gearShifter)=='drive' and .3<=cs.vEgo<=40*.44704
                and CP.steerRatio>0 and CP.wheelbase>0
                and (nav_owned and decision.get('source')=='navigation_city_turn'
                     or not nav_owned and decision.get('signal_owner')=='driver'
                     and decision.get('source') in ('none','driver_desire_helper'))
                and decision.get('tap_signal_active') is False
                and (nav_owned or decision.get('synthetic_indicator_request')=='none'))
      if eligible:
        direction=plan['direction']
        release=(nav_owned and nav_request.get('release_only') is True
                 and life.get('phase')=='COMPLETE' and plan['phase']=='HANDOFF')
        eligible=((release or nav_owned and navigation_signal_matches(cs,nav_request,now)
                   or not nav_owned and bool(cs.leftBlinker)!=bool(cs.rightBlinker)
                   and (direction==1)==bool(cs.leftBlinker))
                  and not (cs.leftBlindspot if direction==1 else cs.rightBlindspot)
                  and (life.get('direction') if nav_owned else decision.get('manual_turn_direction'))==direction
                  and plan.get('lateral_phase')==(life.get('phase') if nav_owned else decision.get('manual_turn_phase'))
                  and ((life.get('blocked') if nav_owned else decision.get('manual_turn_blocked')) is not True
                       or (life.get('phase') if nav_owned else decision.get('manual_turn_phase'))=='COMPLETE'
                       and plan['phase'] in ('HANDOFF','COMPLETE')))
      if not eligible or plan['phase'] not in ('WAITING','APPROACH','TURN','EXIT','HANDOFF'):
        self.turn_plan.reset()
        self.plan_reason='NO_SHARED_PROFILE_AUTHORITY'
        return None
      identity=(plan.get('owner','driver'),plan.get('owner_key'),plan['id'],plan['direction'])
      if identity!=getattr(self,'turn_identity',None):
        self.turn_plan.reset()
        # A restarted consumer cannot invent the preceding maneuver's handoff.
        if plan['phase']=='HANDOFF':
          return None
      if plan['phase']=='HANDOFF' and self.turn_plan.target is None:
        return None
      self.turn_identity=identity
      actual=abs(math.tan(math.radians(cs.steeringAngleDeg)/CP.steerRatio)/CP.wheelbase)
      target=profile_target(plan.get('profile'),self.comfort.corner_lateral_accel,
                            self.comfort.corner_max_decel,cruise_speed)
      if plan.get('geometry_status')=='HELD_CEILING':
        age=plan.get('geometry_age_s')
        if (type(age) not in (int,float) or not math.isfinite(age) or not 0<=age<=.15
            or self.turn_plan.target is None):
          self.turn_plan.reset()
          return None
        if target is not None:
          target=min(target,self.turn_plan.target)
      if target is not None and actual>=MIN_CURVATURE:
        target=min(target,math.sqrt(self.comfort.corner_lateral_accel/actual))
      self.plan_reason=plan['phase']
      return self.turn_plan.update(target,cs.vEgo,actual,cruise_speed,now,True,False,True,
                                   phase=plan['phase'])
    except (AttributeError,TypeError,ValueError,KeyError,OverflowError):
      self.turn_plan.reset()
      self.plan_reason='SHARED_PROFILE_UNAVAILABLE'
      return None

  def apply(self, base, sm, CP, reset, coast, cruise_speed=None, decision=None, prepared=False):
    now=time.monotonic()
    target=(self.turn_plan.target if self.turn_plan.owner == 'PLANNER' else None) if prepared else self.prepare(sm,CP,reset,cruise_speed,decision)
    if target is None:
      self.cap=None
      self.target_speed=None
      self.last_time=now
      self.reason='Turn speed '+self.turn_plan.owner.lower()+'; normal planner'
      return base
    # The speed planner already owns target release. Keep the existing optional
    # corner deceleration envelope and the single final longitudinal handoff.
    curvature=max(MIN_CURVATURE,min(.25,self.comfort.corner_lateral_accel/max(target*target,.01)))
    return self.update(base,sm['carState'].vEgo,curvature,coast,True,now,cruise_speed,target_override=target)
