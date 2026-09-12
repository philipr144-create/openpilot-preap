"""Driver-confirmed navigation desires and synthetic announcement requests."""
import json
import math
import os
import tempfile
import time

SNAPSHOT = '/dev/shm/nap_navigation_desire.json'
DIAGNOSTICS = '/dev/shm/nap_navigation_decision.json'
SIGNAL_REQUEST = '/dev/shm/nap_navigation_signal_request.json'
TURN_SPEED_MAX = 25 * 0.44704
TURN_CONFIRM_DISTANCE = 80.0
EXIT_SIGNAL_SECONDS = 4.0
EXIT_SIGNAL_DISTANCE_MIN = 60.0
EXIT_SIGNAL_DISTANCE_MAX = 120.0
TURN_SIGNAL_SECONDS = 5.0
TURN_SIGNAL_DISTANCE_MIN = 60.0
TURN_SIGNAL_DISTANCE_MAX = 80.0


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
    self.owns_blinker = True  # Unknown ownership cannot silently authorize manual fallback.
    self.path = path
    self.diagnostics_path = diagnostics_path
    self.signal_path = signal_path
    self.previous_signal = None
    self.key = None
    self.confirmed = False
    self.blocked = False
    self.started = None
    self.last_write = -1e9
    self.decision = {}
    self.indicator_request = 'none'
    self.prepared_navigation = None
    self.signal_cancelled_key = None
    self.previous_lat_active = False

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
    if state is None:
      return self.owns_blinker  # Unknown/stale active route cannot authorize a tap.
    if not self.owns_blinker:
      return False
    maneuver = state['maneuver']
    kind, distance = maneuver['type'], maneuver['distance_m']
    return ((kind in ('fork', 'off ramp') and 10 <= distance <= 300)
            or (kind in ('turn', 'end of road') and -8 <= distance <= TURN_CONFIRM_DISTANCE)
            or self.started is not None)

  def update(self, cs, cc, inputs_valid, driver_desire_none=True, now=None,
             signal_owned_by_tap=False, physical_direction=None):
    now = time.monotonic() if now is None else now
    lat_active = bool(cc.latActive)
    lat_disengaged = self.previous_lat_active and not lat_active
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
                     'tap_signal_active': bool(signal_owned_by_tap)}
    current_maneuver_id = ''

    def result(desire, reason, indicator_direction=0):
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
      self.confirmed = False
      self.started = None
      return result('none', reason)
    maneuver = state['maneuver']
    kind, modifier, distance = maneuver['type'], maneuver['modifier'], maneuver['distance_m']
    maneuver_id = state.get('maneuver_id', '')
    current_maneuver_id = maneuver_id if isinstance(maneuver_id, str) else ''
    key = (state['route_id'], maneuver_id)
    if key != self.key:
      self.key = key
      self.confirmed = False
      self.blocked = False
      self.started = None
      self.signal_cancelled_key = None
    self.decision.update(route_id=state['route_id'], maneuver_id=maneuver_id,
                         maneuver_type=kind, modifier=modifier, distance_m=distance)
    if not self.owns_blinker:
      self.confirmed = False
      self.started = None
      return result('none', 'No navigation-owned route')
    if signal_owned_by_tap:
      return result('none', 'Active synthetic tap lane change owns the signal')
    if not inputs_valid:
      self.confirmed = False
      self.started = None
      return result('none', 'Vehicle state is stale or invalid; confirm again')
    steering_override = cs.steeringPressed or getattr(cs, 'steeringDisengage', False)
    if not driver_desire_none and not self.owns_blinker:
      if self.started is not None:
        self.blocked = True
      self.confirmed = False
      self.started = None
      return result('none', 'Existing driver desire has priority')
    if not math.isfinite(cs.vEgo) or cs.vEgo < -.1:
      return result('none', 'Invalid vehicle speed')
    speed = max(0.0, cs.vEgo)  # Tolerate small standstill sensor noise.
    if str(getattr(cs, 'gearShifter', 'drive')) not in ('drive', 'low', 'sport', 'eco'):
      return result('none', 'Navigation requires a forward driving gear')
    side = {'left': 'left', 'slight left': 'left', 'right': 'right', 'slight right': 'right'}.get(modifier)
    if side is None:
      return result('none', 'Unsupported direction; no U-turn or sharp-turn automation')
    if lat_disengaged and self.indicator_request != 'none':
      self.signal_cancelled_key = key
    if kind in ('fork', 'off ramp'):
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
      if (distance > signal_window or signal is not None or not cc.latActive
          or self.signal_cancelled_key == key):
        indicator = 0
      else:
        if self.started is None:
          self.started = now
      if steering_override:
        return result('none', 'Driver steering override; exit signal remains active', indicator)
      if cs.brakePressed or not cc.latActive:
        return result('none', 'Exit guidance paused; brake or lateral control gate', indicator)
      return result('keepLeft' if side == 'left' else 'keepRight',
                    'Navigation-authorized bounded exit preference', indicator)
    if kind not in ('turn', 'end of road'):
      return result('none', 'Maneuver type is display-only: ' + kind)
    if not isinstance(maneuver_id, str) or not maneuver_id:
      return result('none', 'Intersection identifier missing; reload navigation server')
    # The phone/server clamps maneuver distance at zero. Never let a request
    # begin at zero: that is normally a maneuver which has already been
    # reached but has not yet advanced to the next route step.
    if not 0 < distance <= TURN_CONFIRM_DISTANCE:
      self.confirmed = False
      self.started = None
      return result('none', 'Intersection outside active approach window (0–80 m ahead)')
    self.confirmed = True
    if signal is not None and signal != side:
      if self.indicator_request != 'none':
        self.signal_cancelled_key = key
      self.started = None
      return result('none', 'Opposite physical blinker cancelled navigation signal')
    signal_window = min(TURN_SIGNAL_DISTANCE_MAX,
                        max(TURN_SIGNAL_DISTANCE_MIN, speed * TURN_SIGNAL_SECONDS))
    indicator = 1 if side == 'left' else 2
    if (distance > signal_window or signal is not None or not cc.latActive
        or self.signal_cancelled_key == key):
      indicator = 0
    elif self.started is None:
      self.started = now

    # Signaling announces the already-authorized route step and therefore
    # starts on approach. The model turn desire remains conservatively gated
    # by speed and vehicle control state.
    if steering_override:
      return result('none', 'Driver steering override; turn signal remains active', indicator)
    if speed >= TURN_SPEED_MAX:
      return result('none', 'Turn signal active; model guidance waits below 25 mph', indicator)
    if cs.brakePressed or not cc.latActive:
      return result('none', 'Intersection guidance paused; brake or lateral control gate', indicator)
    desire_window = min(30.0, max(12.0, speed*2.0))
    if distance > desire_window:
      waiting_reason = ('Turn signal active; waiting until model guidance window' if indicator
                        else 'Turn confirmed; waiting until signal approach window')
      return result('none', waiting_reason, indicator)
    return result('turnLeft' if side == 'left' else 'turnRight',
                  'Navigation-authorized low-speed intersection turn',
                  indicator)

  def _publish_signal(self, direction, maneuver_id, desire, reason, now):
    name = None
    try:
      with tempfile.NamedTemporaryFile(mode='w', dir=os.path.dirname(self.signal_path),
                                       prefix='.nap-nav-signal-', delete=False) as stream:
        name = stream.name
        json.dump({'time': now, 'active': direction in (1, 2),
                   'direction': direction if direction in (1, 2) else 0,
                   'maneuver_id': maneuver_id if isinstance(maneuver_id, str) else '',
                   'desire': desire, 'reason': reason}, stream, allow_nan=False)
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
                             tap_status='unavailable', tap_signal_active=False):
    # Captured after model.run, before DesireHelper updates for the next frame.
    self.decision.update(selected_desire=selected, final_desire=selected if evaluated else None,
                         source=source, model_evaluated=bool(evaluated), frame_id=int(frame_id),
                         signal_owner=signal_owner, blinker_owner=signal_owner,
                         tap_enabled=bool(tap_enabled),
                         tap_status=tap_status, tap_signal_active=bool(tap_signal_active))

  def publish_diagnostics(self, now=None):
    now = time.monotonic() if now is None else now
    if now-self.last_write < .5:
      return
    self.last_write = now
    name = None
    try:
      with tempfile.NamedTemporaryFile(mode='w', dir=os.path.dirname(self.diagnostics_path),
                                       prefix='.nap-decision-', delete=False) as stream:
        name = stream.name
        json.dump(self.decision, stream, allow_nan=False)
      os.replace(name, self.diagnostics_path)
    except (OSError, ValueError, TypeError):
      pass  # Diagnostics must never interrupt model inference.
    finally:
      if name:
        try:
          os.unlink(name)
        except OSError:
          pass
