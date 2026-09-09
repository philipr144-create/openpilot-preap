"""Bounded, fail-closed reader for authenticated NAP branch guidance."""
import json
import math
import time

SNAPSHOT = '/dev/shm/nap_navigation_desire.json'


def navigation_desire(path=SNAPSHOT, now=None):
  try:
    with open(path, 'r') as stream:
      raw = stream.read(2049)
    if len(raw) > 2048:
      return 'none'
    state = json.loads(raw)
    now = time.monotonic() if now is None else now
    received = state['received_mono']
    expires = state['expires_mono']
    if (state.get('enabled') is not True or state.get('route_state') != 'active'
        or not state.get('route_id') or type(received) not in (int, float)
        or type(expires) not in (int, float) or not math.isfinite(received + expires)
        or not received <= now < expires or not 0 < expires - received <= 3.0):
      return 'none'
    maneuver = state['maneuver']
    distance = maneuver['distance_m']
    if (type(distance) not in (int, float) or not math.isfinite(distance)
        or not 10 <= distance <= 300 or maneuver.get('type') not in ('fork', 'off ramp')):
      return 'none'
    return {'left': 'keepLeft', 'slight left': 'keepLeft',
            'right': 'keepRight', 'slight right': 'keepRight'}.get(maneuver.get('modifier'), 'none')
  except (OSError, ValueError, TypeError, KeyError, AttributeError):
    return 'none'
