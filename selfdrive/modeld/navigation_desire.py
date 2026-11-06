from openpilot.selfdrive.controls.lib.turn_context import driver_braking, classify_route_maneuver, maneuver_context, city_turns_enabled
from openpilot.selfdrive.controls.lib.turn_steering_gate import matching_signal
from openpilot.selfdrive.modeld.exit_sequence import ExitSequence
from openpilot.selfdrive.modeld.navigation_city_turn import NavigationCityTurn
"""Driver-confirmed navigation desires and synthetic announcement requests."""
import json
import hashlib
import math
import os
import tempfile
import time

SNAPSHOT = '/dev/shm/nap_navigation_desire.json'
DIAGNOSTICS = '/dev/shm/nap_navigation_decision.json'
SIGNAL_REQUEST = '/dev/shm/nap_navigation_signal_request.json'
TURN_SPEED_MAX = 25 * 0.44704
TURN_CONTINUE_SPEED_MAX = 35 * 0.44704
TURN_CONFIRM_DISTANCE = 80.0
EXIT_SIGNAL_SECONDS = 7.0
EXIT_SIGNAL_DISTANCE_MIN = 60.0
EXIT_SIGNAL_DISTANCE_MAX = 250.0
TURN_SIGNAL_SECONDS = 5.0
TURN_SIGNAL_DISTANCE_MIN = 60.0
TURN_SIGNAL_DISTANCE_MAX = 80.0


class NavigationTurnLatch:
  """Do not reuse a navigation blinker as a second manual turn request."""

  def __init__(self):
    self.previous_request = False
    self.blocked = False
    self.quiet_since = None

  def update(self, request_active, vehicle_blinker_active, now):
    if self.previous_request and not request_active:
      self.blocked = True
    self.previous_request = request_active
    if request_active or vehicle_blinker_active:
      self.quiet_since = None
    elif self.blocked:
      if self.quiet_since is None:
        self.quiet_since = now
      elif now - self.quiet_since >= .5:
        self.blocked = False
        self.quiet_since = None
    return self.blocked


def reliable_position(state):
  quality = state.get('position_quality')
  if not isinstance(quality, dict):
    return False
  if state.get('version') == 4 and (quality.get('projection_valid') is not True
      or quality.get('path_ambiguous') is not False
      or type(quality.get('heading_error_deg')) not in (int, float)
      or not 0 <= quality['heading_error_deg'] <= 20):
    return False
  limits = (('match_error_m', 15.0), ('gps_accuracy_m', 20.0), ('gps_age_s', 1.5))
  return all(type(quality.get(name)) in (int, float) and
             math.isfinite(quality[name]) and 0 <= quality[name] <= limit
             and (name != 'gps_accuracy_m' or quality[name] > 0)
             for name, limit in limits)


def read_navigation(path, now, ownership=None):
  try:
    with open(path) as stream:
      raw = stream.read(2049)
    if len(raw) > 2048:
      return None, 'Navigation snapshot too large'
    state = json.loads(raw)
    if ownership is not None:
      # Acquisition is conservative; release needs a fresh explicit server state.
      if state.get('enabled') is True and state.get('route_active') is True:
        ownership.owns_blinker = True
      received, expires = state.get('received_mono'), state.get('expires_mono')
      fresh = (type(received) in (int, float) and type(expires) in (int, float)
               and math.isfinite(received) and math.isfinite(expires)
               and received <= now < expires and 0 < expires-received <= 3)
      if fresh and (state.get('enabled') is False or
                    (state.get('enabled') is True and state.get('route_active') is False)):
        ownership.owns_blinker = False

    if state.get('enabled') is not True:
      return None, 'Navigation influence is off'
    if isinstance(state.get('pause_reason'), str) and state['pause_reason']:
      return None, 'Navigation paused: ' + state['pause_reason'][:180]
    received, expires = state['received_mono'], state['expires_mono']
    if (type(received) not in (int, float) or type(expires) not in (int, float)
        or not math.isfinite(received + expires) or not received <= now < expires
        or not 0 < expires-received <= 3):
      return None, 'Navigation or GPS is stale'
    if state.get('route_state') != 'active' or not isinstance(state.get('route_id'), str) or not state['route_id']:
      return None, 'No active route instruction'
    if state.get('version') == 4:
      quality = state.get('position_quality', {})
      age = quality.get('gps_age_s')
      if type(age) not in (int, float) or not 0 <= age + now-received <= 1.5:
        return None, 'Projected GPS position expired'
    maneuver = state['maneuver']
    distance = maneuver['distance_m']
    if type(distance) not in (int, float) or not math.isfinite(distance):
      return None, 'Invalid maneuver distance'
    if not isinstance(maneuver.get('type'), str) or not isinstance(maneuver.get('modifier'), str):
      return None, 'Invalid maneuver'
    return state, ''
  except (OSError, ValueError, TypeError, KeyError, AttributeError, RecursionError):
    return None, 'Navigation snapshot unavailable or invalid'


class NavigationDesire:
  def __init__(self, path=SNAPSHOT, diagnostics_path=DIAGNOSTICS, signal_path=SIGNAL_REQUEST):
    self.owns_blinker = True  # Route permission; claims_tap tracks active reservation separately.
    self.path = path
    self.diagnostics_path = diagnostics_path
    self.signal_path = signal_path
    self.previous_signal = None
    self.key = None
    self.confirmed = False
    self.blocked = False
    self.started = None
    self.last_write = -1e9
    self.manual_diagnostics_active = False
    self.navigation_diagnostics_active = False
    self.decision = {}
    self.indicator_request = 'none'
    self.prepared_navigation = None
    self.signal_cancelled_key = None
    self.cancelled_maneuvers = set()
    self.previous_lat_active = False
    self.turn_guidance_started = False
    self.passed_key = None
    self.exit_sequence = ExitSequence()
    self.city_bridge = NavigationCityTurn()
    self.signal_status_path = '/dev/shm/nap_navigation_signal_status.json'

  def reset_countdown(self, state=None):
    self._distance_valid = False
    if state is not None:
      self._distance_snapshot = (state['route_id'], state.get('maneuver_id', ''), state['received_mono'], state['maneuver']['distance_m'])


  def maneuver_distance(self, state, speed, now):
    """Interpolate one fresh server snapshot; never carry state across routes."""
    raw = state['maneuver']['distance_m']
    snapshot = (state['route_id'], state.get('maneuver_id', ''), state['received_mono'], state['maneuver']['distance_m'])
    elapsed = now - getattr(self, '_distance_time', now)
    if snapshot != getattr(self, '_distance_snapshot', None):
      # The producer has already projected GPS to received_mono. Deduct only
      # transport/consumer age, never the GPS sensor age a second time.
      age = now - state['received_mono']
      self._distance_valid = 0 <= age <= .25
      self._distance_value = raw - max(0.0, speed) * age if self._distance_valid else raw
    elif not 0 <= elapsed <= .25:
      # A loop gap must not revive a maneuver already counted past zero.
      # Wait for a new server snapshot rather than guessing unobserved travel.
      self._distance_valid = False
    elif self._distance_valid:
      self._distance_value -= max(0.0, speed) * elapsed
    self._distance_snapshot = snapshot
    self._distance_time = now
    return self._distance_value if self._distance_valid else None


  def maneuvers_enabled(self):
    if not hasattr(self, 'params'):
      from openpilot.common.params import Params
      self.params = Params()
    return self.params.get_bool("NAPNavigationManeuvers")

  def claims_tap(self, now=None, prepare=False):
    now = time.monotonic() if now is None else now
    if not self.maneuvers_enabled():
      self.owns_blinker = False
      if prepare:
        self.prepared_navigation = (None, 'Disabled in NAP settings')
      return False
    state, reason = read_navigation(self.path, now, self)
    if prepare:
      # modeld asks for ownership immediately before update. Reuse this exact
      # atomic snapshot so a distance/route update cannot split arbitration.
      self.prepared_navigation = (state, reason)
    # Route permission is not an active signal owner. A paused route with no
    # request must not consume a fresh physical tap, even while owns_blinker
    # remains conservatively latched until an explicit server release.
    active_request = self.indicator_request != 'none' or self.turn_guidance_started
    if state is None:
      # Preserve startup/restart protection until a valid route observation
      # establishes context. Once known, an idle route's GPS expiry is not an
      # ongoing reservation of driver input.
      return active_request or (self.owns_blinker and not getattr(self, '_tap_snapshot_observed', False))
    self._tap_snapshot_observed = True
    if not self.owns_blinker:
      return False
    key = (state['route_id'], state.get('maneuver_id', ''))
    if ((key == self.key and self.blocked) or self.signal_cancelled_key == key
        or self.passed_key == key):
      return False
    if not reliable_position(state):
      return active_request
    maneuver = state['maneuver']
    kind, distance = maneuver['type'], maneuver['distance_m']
    if (kind in ('turn','end of road') or self.city_bridge.accepted()) and not city_turns_enabled(self.params):
      return False
    if self.city_bridge.accepted() and state['route_id']==self.city_bridge.key[0]:
      return True
    return ((kind in ('fork', 'off ramp') and 10 <= distance <= 300)
            or (kind in ('turn', 'end of road') and -8 <= distance <= TURN_CONFIRM_DISTANCE)
            or self.started is not None)

  def update(self, cs, cc, inputs_valid, driver_desire_none=True, now=None,
             signal_owned_by_tap=False, physical_direction=None,
             exit_corridor=None, exit_model_frame=None, model=None, model_inputs_valid=False,
             actual_curvature=None, city_enabled=False):
    now = time.monotonic() if now is None else now
    lat_active = bool(cc.latActive)
    lat_disengaged = self.previous_lat_active and not lat_active
    steering_override = bool(cs.steeringPressed or getattr(cs, 'steeringDisengage', False))
    self.previous_lat_active = lat_active
    if physical_direction in (0, 1, 2):
      # Pre-AP passes the filtered physical STW_ACTN_RQ state. Synthetic frames
      # and their RX reflections can therefore never authorize navigation.
      signal = {0: None, 1: 'left', 2: 'right'}[physical_direction]
    else:
      signal = ('left' if cs.leftBlinker else 'right') if cs.leftBlinker != cs.rightBlinker else None
    self.previous_signal = signal
    if self.prepared_navigation is not None:
      state, reason = self.prepared_navigation
      self.prepared_navigation = None
    else:
      state, reason = read_navigation(self.path, now, self)

    if not self.maneuvers_enabled():
      state = None
      reason = 'Disabled in NAP settings'

    self.decision = {'received_mono': now, 'desire': 'none', 'reason': reason,
                     'route_id': '', 'maneuver_id': '', 'confirmed': False,
                     'physical_signal': signal or 'none',
                     'tap_signal_active': bool(signal_owned_by_tap),
                     'driver_steering_override': steering_override}
    current_maneuver_id = ''
    exit_sequence_used = False

    def result(desire, reason, indicator_direction=0):
      if not exit_sequence_used:
        self.exit_sequence.withdraw()
      if 'navigation_turn' not in self.decision:
        self.city_bridge.withdraw(reason)
      if desire in ('turnLeft', 'turnRight'):
        self.turn_guidance_started = True
      if self.blocked or not self.confirmed:
        self.turn_guidance_started = False
      if self.city_bridge.intent.phase=='COMPLETE':
        self.turn_guidance_started = False
      self.indicator_request = {0: 'none', 1: 'left', 2: 'right'}[indicator_direction]
      self._publish_signal(indicator_direction, current_maneuver_id, desire, reason, now)
      self.decision.update(desire=desire, reason=reason, confirmed=self.confirmed,
                           navigation_route_owned=bool(self.owns_blinker),
                           blinker_owner=('driver' if signal is not None else 'navigation'
                                          if indicator_direction else 'tap'
                                          if signal_owned_by_tap else 'none'),
                           synthetic_indicator_request=self.indicator_request)
      return desire

    if state is None:
      self.city_bridge.withdraw('NAVIGATION_WITHDRAWN')
      self.reset_countdown()
      if ((steering_override or lat_disengaged or getattr(cs, 'steeringDisengage', False)
           or signal is not None or signal_owned_by_tap)
          and self.key is not None and
          (steering_override or self.confirmed or self.started is not None or self.indicator_request != 'none')):
        self.blocked = True
        self.signal_cancelled_key = self.key
        self.cancelled_maneuvers.add(self.key)
      self.confirmed = False
      self.started = None
      return result('none', reason)
    maneuver = state['maneuver']
    kind, modifier, raw_distance = maneuver['type'], maneuver['modifier'], maneuver['distance_m']
    maneuver_id = state.get('maneuver_id', '')
    current_maneuver_id = maneuver_id if isinstance(maneuver_id, str) else ''

    distance = raw_distance
    key = (state['route_id'], maneuver_id)
    if key != self.key:
      self.key = key
      self.turn_guidance_started = False
      self.confirmed = False
      self.blocked = key in self.cancelled_maneuvers
      self.started = None
      self.signal_cancelled_key = key if self.blocked else None
    self.decision.update(route_id=state['route_id'], maneuver_id=maneuver_id,
                         maneuver_type=kind, modifier=modifier, distance_m=distance)
    # Manual steering cancels this route step, including an announced or
    # queued request. Releasing the wheel is not a new authorization.
    hard_takeover = (
      steering_override
      or (lat_disengaged and (cs.vEgo>.3 or self.city_bridge.intent.physical_turn_started))
      or bool(getattr(cs, 'steeringDisengage', False))
      or signal_owned_by_tap
      or not self.owns_blinker
    )

    if (
      hard_takeover
      and (
        steering_override
        or self.confirmed
        or self.started is not None
        or self.indicator_request != 'none'
      )
    ):
      self.blocked = True
      self.signal_cancelled_key = key
      self.cancelled_maneuvers.add(key)
    if self.blocked:
      self.confirmed = False
      self.started = None
      return result('none', 'Driver takeover cancelled this maneuver; waiting for next route step')

    if not self.owns_blinker:
      self.confirmed = False
      self.started = None
      return result('none', 'No navigation-owned route')
    if signal_owned_by_tap:
      return result('none', 'Active synthetic tap lane change owns the signal')
    if not inputs_valid:
      self.reset_countdown(state)
      self.confirmed = False
      self.started = None
      return result('none', 'Vehicle state is stale or invalid; confirm again')
    if not driver_desire_none and not self.owns_blinker:
      if self.started is not None:
        self.blocked = True
      self.confirmed = False
      self.started = None
      return result('none', 'Existing driver desire has priority')
    if not math.isfinite(cs.vEgo) or cs.vEgo < -.1:
      self.reset_countdown(state)
      return result('none', 'Invalid vehicle speed')
    speed = max(0.0, cs.vEgo)  # Tolerate small standstill sensor noise.
    if str(getattr(cs, 'gearShifter', 'drive')) not in ('drive', 'low', 'sport', 'eco'):
      self.reset_countdown(state)
      return result('none', 'Navigation requires a forward driving gear')
    side = {'left': 'left', 'slight left': 'left', 'right': 'right', 'slight right': 'right'}.get(modifier)
    if (self.city_bridge.accepted() or self.city_bridge.handoff_active(now)) and state['route_id']==self.city_bridge.key[0]:
      side='left' if self.city_bridge.direction==1 else 'right'
    if side is None:
      return result('none', 'Unsupported direction; no U-turn or sharp-turn automation')
    if not reliable_position(state):
      if (self.turn_guidance_started or self.confirmed) and speed < 1.0:
        pass
      else:
        self.reset_countdown(state)
        self.confirmed = False
        self.started = None
        return result('none', 'Navigation position uncertain: GPS accuracy or route match unavailable')
    distance = self.maneuver_distance(state, speed, now)
    self.decision.update(raw_distance_m=raw_distance, distance_m=distance,
                         snapshot_age_s=now-state['received_mono'],
                         source_received_mono=state['received_mono'],
                         source_gps_age_s=state.get('position_quality',{}).get('gps_age_s'),
                         decision_at_unix=time.time())
    if distance is None:
      if (self.turn_guidance_started or self.confirmed) and speed < 1.0:
        distance = 0.0
      else:
        self.confirmed = False
        self.started = None
        self.turn_guidance_started = False
        return result('none', 'Countdown paused; waiting for a fresh route snapshot')
    if self.passed_key == key:
      self.confirmed = False
      self.started = None
      return result('none', 'Maneuver already passed; waiting for next route step')
    if kind in ('turn','end of road') or self.city_bridge.accepted() or self.city_bridge.handoff_active(now):
      if not isinstance(maneuver_id,str) or not maneuver_id:
        self.city_bridge.withdraw('MANEUVER_ID_MISSING')
        return result('none','Intersection identifier missing')
      status=None
      try:
        with open(self.signal_status_path) as stream:
          raw=stream.read(4097)
        status=json.loads(raw) if len(raw)<=4096 else None
      except (OSError,ValueError,TypeError):
        pass
      outcome=self.city_bridge.step(state,distance,cs,cc,now=now,enabled=city_enabled,
        model=model,model_valid=model_inputs_valid,physical_direction=physical_direction,
        signal_status=status,actual_curvature=actual_curvature)
      self.decision['navigation_turn']=outcome
      request=outcome.get('request')
      self.confirmed=request is not None
      if request is not None:
        self.decision.update(route_id=request['route_id'],maneuver_id=request['maneuver_id'],
                             maneuver_type=request['kind'],modifier='left' if request['direction']==1 else 'right')
        current_maneuver_id=request['maneuver_id']
      return result(outcome['desire'],outcome['reason'],outcome['indicator'])
    self.city_bridge.withdraw('NOT_A_CITY_TURN')
    if distance <= 0:
      self.passed_key = key
    if classify_route_maneuver(maneuver) in ('highway_exit', 'road_fork'):
      if not isinstance(maneuver_id, str) or not maneuver_id:
        return result('none', 'Exit identifier missing')
      if not 10 <= distance <= 300:
        self.confirmed = False
        self.started = None
        return result('none', 'Fork/exit outside 10–300 m window')
      # The fresh route maneuver authorizes navigation. A physical stalk owns
      # the signal while held, but cannot poison this maneuver before or after
      # the action window.
      self.confirmed = True
      if signal is not None and signal != side:
        if self.indicator_request != 'none':
          self.signal_cancelled_key = key
        self.started = None
        return result('none', 'Opposite physical blinker cancelled navigation signal')
      signal_window = min(EXIT_SIGNAL_DISTANCE_MAX,
                          max(EXIT_SIGNAL_DISTANCE_MIN, speed * EXIT_SIGNAL_SECONDS))
      indicator = 1 if side == 'left' else 2
      if (distance < 20 or distance > signal_window or signal is not None or not cc.latActive
          or not reliable_position(state)
          or self.signal_cancelled_key == key):
        indicator = 0
      else:
        if self.started is None:
          self.started = now
      # Actual lamp feedback must precede any maneuver. This is an evidence
      # consumer, not a ramp detector: existing modeld supplies no corridor
      # evidence, so this patch alone cannot authorize automatic ramp entry.
      exit_sequence_used = True
      sequence = self.exit_sequence.update(
        route_id=state['route_id'], maneuver_id=maneuver_id,
        direction=1 if side == 'left' else 2, now=now, distance_m=distance,
        route_fresh=reliable_position(state), inputs_valid=bool(inputs_valid),
        lateral_active=bool(cc.latActive), signal_announced=bool(indicator or signal == side),
        vehicle_signal_matches=matching_signal(cs, 1 if side == 'left' else 2),
        driver_override=steering_override, braking=driver_braking(cs),
        opposite_signal=signal is not None and signal != side,
        model_frame=exit_model_frame, evidence=exit_corridor)
      self.decision['exit_sequence'] = sequence.diagnostic()
      return result(sequence.desire, sequence.reason, indicator)
    return result('none', 'Maneuver type is display-only: ' + kind)

  def _publish_signal(self, direction, maneuver_id, desire, reason, now):
    name = None
    try:
      with tempfile.NamedTemporaryFile(mode='w', dir=os.path.dirname(self.signal_path),
                                       prefix='.nap-nav-signal-', delete=False) as stream:
        name = stream.name
        request=self.city_bridge.request or {}
        controller=request.get('controller',self.city_bridge.controller)
        route_id=request.get('route_id',self.decision.get('route_id',''))
        key=request.get('key') or hashlib.sha256(('|'.join((controller,route_id,maneuver_id))).encode()).hexdigest()[:32]
        stream.write(json.dumps({'version': 2, 'time': now, 'active': direction in (1, 2),
                   'route_id': route_id,
                   'controller': controller, 'request_key': key,
                   'signal_owner': 'navigation', 'cadence_mode': 'factory',
                   'stationary_wait_allowed': bool(request and request.get('stationary_wait_allowed')),
                   'direction': direction if direction in (1, 2) else 0,
                   'maneuver_id': maneuver_id if isinstance(maneuver_id, str) else '',
                   'desire': desire, 'reason': reason}, allow_nan=False))
      os.replace(name, self.signal_path)
    except (OSError, ValueError, TypeError):
      pass  # Signaling must never interrupt model inference.
    finally:
      if name:
        try:
          os.unlink(name)
        except OSError:
          pass

  def record_model_selection(self, selected, source, evaluated, frame_id, *,
                             signal_owner='none', tap_enabled=False,
                             tap_status='unavailable', tap_signal_active=False,
                             manual_turn_direction=0, city_turns_enabled=False, manual_turn_blocked=False,
                             manual_turn_continuing=False, manual_turn_phase=None, manual_turn_reason=None, city_turn_diagnostics=None, city_turn_plan=None):
    self.city_turn_diagnostics = city_turn_diagnostics
    self.decision['city_turn_plan'] = city_turn_plan
    self.decision['turn_locked'] = (isinstance(city_turn_diagnostics, dict)
                                    and city_turn_diagnostics.get('turn_locked') is True)
    self.decision['city_turn_stage'] = city_turn_diagnostics.get('stage') if isinstance(city_turn_diagnostics, dict) else None
    # Captured after model.run, before DesireHelper updates for the next frame.
    self.decision.update(selected_desire=selected, final_desire=selected if evaluated else None,
                         source=source, model_evaluated=bool(evaluated), frame_id=int(frame_id),
                         signal_owner=signal_owner, blinker_owner=signal_owner,
                         tap_enabled=bool(tap_enabled),
                         tap_status=tap_status, tap_signal_active=bool(tap_signal_active),
                         manual_turn_direction=manual_turn_direction,
                         city_turns_enabled=bool(city_turns_enabled),
                         manual_turn_blocked=bool(manual_turn_blocked),
                         manual_turn_continuing=bool(manual_turn_continuing),
                         manual_turn_phase=manual_turn_phase, manual_turn_reason=manual_turn_reason)
    self.decision['maneuver_context'] = maneuver_context(self.decision, time.monotonic(), int(frame_id))

  def publish_diagnostics(self, now=None):
    now = time.monotonic() if now is None else now
    # Manual commitment needs fresh model ownership and an immediate revocation.
    # Navigation commitment consumes the same 0.35 s freshness contract.
    # Armed physical intent needs the same freshness as a selected desire.
    # Otherwise the 0.5 s idle cadence outlives controlsd's 0.35 s validity.
    manual = (self.decision.get('signal_owner') == 'driver'
              and self.decision.get('city_turns_enabled') is True
              and self.decision.get('manual_turn_direction') in (1, 2))
    navigation = (self.decision.get('desire') in ('turnLeft', 'turnRight', 'keepLeft', 'keepRight')
                  or self.decision.get('source') in ('navigation','navigation_city_turn')
                  or self.decision.get('synthetic_indicator_request') in ('left', 'right'))
    # One final fast publication withdraws a request promptly when it ends.
    interval = .05 if (manual or self.manual_diagnostics_active
                       or navigation or self.navigation_diagnostics_active) else .5
    if now-self.last_write < interval - 1e-6:
      return
    self.last_write = now
    self.manual_diagnostics_active = manual
    self.navigation_diagnostics_active = navigation
    name = None
    try:
      with tempfile.NamedTemporaryFile(mode='w', dir=os.path.dirname(self.diagnostics_path),
                                       prefix='.nap-decision-', delete=False) as stream:
        name = stream.name
        stream.write(json.dumps(self.decision, allow_nan=False))
      os.replace(name, self.diagnostics_path)
      if isinstance(getattr(self, 'city_turn_diagnostics', None), dict):
        trace = dict(self.city_turn_diagnostics)
        if not hasattr(self, 'city_turn_boot_id'):
          try:
            with open('/proc/sys/kernel/random/boot_id') as boot:
              self.city_turn_boot_id = boot.read(80).strip()
          except OSError:
            self.city_turn_boot_id = None
        trace['boot_id'] = self.city_turn_boot_id
        trace['implementation'] = 'navigation-city-turn-v6-candidate'
        trace['city_turn_plan'] = self.decision.get('city_turn_plan')
        trace.update(producer_mono=self.decision.get('received_mono'),
                     inference_frame=self.decision.get('frame_id'), evaluated=self.decision.get('model_evaluated'),
                     final_desire=self.decision.get('final_desire'), source=self.decision.get('source'),
                     owner=self.decision.get('signal_owner'), synthetic=self.decision.get('synthetic_indicator_request'),
                     tap_active=self.decision.get('tap_signal_active'))
        with tempfile.NamedTemporaryFile(mode='w', dir=os.path.dirname(self.diagnostics_path),
                                         prefix='.nap-city-turn-', delete=False) as stream:
          name = stream.name
          stream.write(json.dumps(trace, allow_nan=False))
        os.replace(name, os.path.join(os.path.dirname(self.diagnostics_path), 'nap_city_turn_intent.json'))
    except (OSError, ValueError, TypeError):
      pass  # Diagnostics must never interrupt model inference.
    finally:
      if name:
        try:
          os.unlink(name)
        except OSError:
          pass
