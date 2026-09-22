"""Direction-matched route stops with driver braking and explicit pedal resume.

shouldStop requests regen through the existing longitudinal controller. Physical
stopping/holding remains the driver's job on the pedal-only Pre-AP installation.
No CAN, steering, or engagement authority lives here.
"""
import json
import math


def number(value):
  return type(value) in (int, float) and math.isfinite(value)


def read_snapshot(path):
  try:
    with open(path) as stream:
      raw = stream.read(4097)
    state = json.loads(raw) if len(raw) <= 4096 else None
    return state if isinstance(state, dict) else None
  except (OSError, ValueError, TypeError, RecursionError):
    return None


def fresh(state, now, ttl):
  if not isinstance(state, dict):
    return False
  start, end = state.get('received_mono'), state.get('expires_mono')
  return (number(start) and number(end) and start <= now < end
          and 0 < end-start <= ttl)


def stop_candidate(state, now):
  if not fresh(state, now, 3):
    return None
  if (state.get('version') != 4 or state.get('enabled') is not True
      or state.get('route_active') is not True or state.get('route_state') != 'active'
      or state.get('pause_reason') or not isinstance(state.get('route_id'), str)
      or not state['route_id']):
    return None
  q, stop = state.get('position_quality'), state.get('mapped_stop_sign')
  if not isinstance(q, dict) or not isinstance(stop, dict):
    return None
  if q.get('projection_valid') is not True or q.get('path_ambiguous') is not False:
    return None
  for name, limit in (('match_error_m', 8), ('gps_accuracy_m', 8),
                      ('gps_age_s', 1.5), ('heading_error_deg', 20)):
    v = q.get(name)
    if not number(v) or not 0 <= v <= limit or name == 'gps_accuracy_m' and v == 0:
      return None
  if q['gps_age_s'] + now-state['received_mono'] > 1.5:
    return None
  distance, identity = stop.get('distance_m'), stop.get('id')
  if not isinstance(identity, str) or not identity or not number(distance) or not -8 <= distance <= 220:
    return None
  return state['route_id'], identity, distance


class MapStop:
  def __init__(self):
    self.phase = 'idle'
    self.event = None
    self.session = None
    self.sequence = None
    self.consumed = {}
    self.travel = 0.0
    self.last_time = None

  def clear(self):
    self.phase, self.event = 'idle', None

  def update(self, nav, resume, *, enabled, active, inputs_valid, v_ego, gas, now):
    if not number(now) or not number(v_ego) or not 0 <= v_ego <= 45:
      self.clear()
      return None, False
    if self.last_time is not None and inputs_valid:
      self.travel += v_ego * max(0., min(.2, now-self.last_time))
    self.last_time = now
    # A route refresh cannot resurrect the same nearby sign. Revisit only after
    # actual travel, not elapsed time, snapshot disappearance, or an ID change.
    self.consumed = {key: at for key, at in self.consumed.items() if self.travel-at < 150}
    status_ok = (fresh(resume, now, .5) and isinstance(resume.get('session'), str)
                 and type(resume.get('sequence')) is int and resume['sequence'] >= 0)
    if not enabled or not inputs_valid:
      self.clear()
      return None, False
    if status_ok:
      token = (resume['session'], resume['sequence'])
      if resume.get('cruise') is not True or resume.get('enabled') is not True:
        self.clear()
        self.session, self.sequence = token
        return None, False
      # Restarted carstate must not reuse a previous process's acknowledgment.
      if self.session is not None and self.session != token[0]:
        self.clear()
      accepted = (self.session == token[0] and self.sequence is not None
                  and token[1] > self.sequence and resume.get('healthy') is True
                  and resume.get('brake') is False and resume.get('paused') is False)
      self.session, self.sequence = token
      if accepted and self.event is not None and self.phase == 'held':
        self.consumed[self.event[1]] = self.travel
        self.clear()
        return None, False
      if self.event is not None and v_ego <= .3 and resume.get('paused') is True:
        self.phase = 'held'
    # Hold the *logical* stop through map loss while the driver brakes. A fresh
    # accepted tap above is the only automatic release of an acquired held stop.
    if self.phase == 'held':
      allowed = active and status_ok and resume.get('healthy') is True and not gas
      return (0., True) if allowed else (None, False)
    if not status_ok or resume.get('healthy') is not True:
      self.clear()
      return None, False
    candidate = stop_candidate(nav, now)
    if candidate is None:
      self.clear()
      return None, False
    route, identity, distance = candidate
    if identity in self.consumed:
      self.clear()
      return None, False
    if self.event != (route, identity):
      self.clear()
      # Never acquire a sign already at/behind the bumper, or while overridden.
      if not active or gas or resume.get('paused') is True or not 8 < distance <= 220 or v_ego > 25:
        return None, False
      self.event = route, identity
    self.phase = 'stopping' if distance <= 8 else 'approach'
    if not active or gas or resume.get('paused') is True:
      return None, False
    # Propagate only the snapshot's brief age, never renew stale GPS authority.
    distance -= v_ego * (now-nav['received_mono'])
    cap = math.sqrt(2*.8*max(0., distance-8))
    return cap, distance <= 8 and v_ego <= 3
