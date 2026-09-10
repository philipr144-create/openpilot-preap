"""Driver-confirmed navigation desires; no vehicle-command interfaces."""
import json
import math
import os
import tempfile
import time

SNAPSHOT = '/dev/shm/nap_navigation_desire.json'
DIAGNOSTICS = '/dev/shm/nap_navigation_decision.json'
TURN_SPEED_MAX = 25 * 0.44704
TURN_CONFIRM_DISTANCE = 80.0
TURN_TIMEOUT = 12.0


def read_navigation(path, now):
  try:
    with open(path) as stream:
      raw = stream.read(2049)
    if len(raw) > 2048:
      return None, 'Navigation snapshot too large'
    state = json.loads(raw)
    if state.get('enabled') is not True:
      return None, 'Navigation influence is off'
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
  def __init__(self, path=SNAPSHOT, diagnostics_path=DIAGNOSTICS):
    self.path = path
    self.diagnostics_path = diagnostics_path
    self.previous_signal = None
    self.key = None
    self.confirmed = False
    self.blocked = False
    self.started = None
    self.last_write = -1e9
    self.decision = {}

  def update(self, cs, cc, inputs_valid, driver_desire_none=True, now=None):
    now = time.monotonic() if now is None else now
    signal = ('left' if cs.leftBlinker else 'right') if cs.leftBlinker != cs.rightBlinker else None
    edge = signal is not None and signal != self.previous_signal
    self.previous_signal = signal
    state, reason = read_navigation(self.path, now)
    self.decision = {'received_mono': now, 'desire': 'none', 'reason': reason,
                     'route_id': '', 'maneuver_id': '', 'confirmed': False}

    def result(desire, reason):
      self.decision.update(desire=desire, reason=reason, confirmed=self.confirmed)
      return desire

    if state is None:
      self.confirmed = False
      self.started = None
      return result('none', reason)
    maneuver = state['maneuver']
    kind, modifier, distance = maneuver['type'], maneuver['modifier'], maneuver['distance_m']
    maneuver_id = state.get('maneuver_id', '')
    key = (state['route_id'], maneuver_id)
    if key != self.key:
      self.key = key
      self.confirmed = False
      self.blocked = False
      self.started = None
    self.decision.update(route_id=state['route_id'], maneuver_id=maneuver_id,
                         maneuver_type=kind, modifier=modifier, distance_m=distance)
    if not inputs_valid:
      self.confirmed = False
      self.started = None
      return result('none', 'Vehicle state is stale or invalid; confirm again')
    if cs.steeringPressed or getattr(cs, 'steeringDisengage', False):
      if self.confirmed or self.started is not None:
        self.blocked = True
      self.confirmed = False
      return result('none', 'Driver steering override')
    if not driver_desire_none:
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
    if kind in ('fork', 'off ramp'):
      if cs.leftBlinker and cs.rightBlinker:
        return result('none', 'Hazard lights are on')
      if signal is not None and signal != side:
        return result('none', 'Blinker conflicts with route')
      if not 10 <= distance <= 300:
        return result('none', 'Fork/exit outside 10–300 m window')
      if cs.brakePressed or not cc.latActive:
        return result('none', 'Release brake and engage lateral control; manual speed control is allowed')
      return result('keepLeft' if side == 'left' else 'keepRight', 'Fresh fork/exit preference')
    if kind != 'turn':
      return result('none', 'Maneuver type is display-only')
    if not isinstance(maneuver_id, str) or not maneuver_id:
      return result('none', 'Intersection identifier missing; reload navigation server')
    if self.blocked:
      return result('none', 'This intersection was cancelled by driver takeover')
    if not -8 <= distance <= TURN_CONFIRM_DISTANCE:
      self.confirmed = False
      self.started = None
      return result('none', 'Intersection outside confirmation window (80 m ahead to 8 m past)')
    if signal != side:
      self.confirmed = False
      self.started = None
      return result('none', 'Use the matching manual blinker to confirm this turn')
    if edge and speed < TURN_SPEED_MAX:
      self.confirmed = True
      self.started = None
    if not self.confirmed:
      return result('none', 'Below 25 mph, switch matching blinker off/on within 80 m to confirm')
    if speed >= TURN_SPEED_MAX:
      self.confirmed = False
      self.started = None
      return result('none', 'Intersection guidance requires speed below 25 mph; confirm again')
    if self.started is not None and now-self.started >= TURN_TIMEOUT:
      self.confirmed = False
      self.blocked = True
      return result('none', 'Intersection request timed out; manual turn required')
    if cs.brakePressed or not cc.latActive:
      if self.started is not None:
        self.confirmed = False
        self.started = None
      return result('none', 'Release brake and engage lateral control; a previously started turn needs a new blinker confirmation')
    window = min(30.0, max(12.0, speed*2.0))
    if distance > window:
      return result('none', 'Turn confirmed; waiting until close to intersection')
    if self.started is None:
      self.started = now
    return result('turnLeft' if side == 'left' else 'turnRight', 'Driver-confirmed low-speed intersection turn')

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
