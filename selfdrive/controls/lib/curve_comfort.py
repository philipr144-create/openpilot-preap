"""Bounded user presets for optional curve comfort; never a safety-limit override."""
from dataclasses import dataclass
import json
import os
from pathlib import Path
import tempfile

PROFILE_PATH = '/data/nap_curve_comfort.json'
PROFILE_NAMES = ('more', 'standard', 'mild')
DEFAULT_PROFILE = 'standard'


@dataclass(frozen=True)
class CurveComfort:
  name: str
  corner_lateral_accel: float
  map_lateral_accel: float
  corner_max_decel: float


PROFILES = {
  # corner_lateral_accel controls signaled city-turn CornerAssist.
  # map_lateral_accel controls ordinary Map Driving Assist curve speed.
  #
  # Ordinary road curves use explicit comfort targets, below the previous
  # speed-growing 3–5 m/s² budgets. City-turn behavior is unchanged.
  'more': CurveComfort('more', 1.6, 1.9, .6),
  'standard': CurveComfort('standard', 1.76, 2.2, .45),
  'mild': CurveComfort('mild', 1.92, 2.5, .3),
}


def read_curve_comfort(path=PROFILE_PATH):
  try:
    with open(path) as stream:
      raw = stream.read(1025)
    data = json.loads(raw) if len(raw) <= 1024 else None
    name = data.get('profile') if isinstance(data, dict) else None
    if isinstance(name, str) and name in PROFILES:
      return PROFILES[name]
  except (OSError, ValueError, TypeError):
    pass
  return PROFILES[DEFAULT_PROFILE]


def write_curve_comfort(name, path=PROFILE_PATH):
  if not isinstance(name, str) or name not in PROFILES:
    raise ValueError('Unknown curve comfort preset')
  target = Path(path)
  temporary = None
  try:
    with tempfile.NamedTemporaryFile(mode='w', dir=target.parent, prefix=target.name+'.', delete=False) as stream:
      temporary = Path(stream.name)
      json.dump({'profile': name}, stream)
      stream.write('\n')
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(temporary, target)
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)


def road_curve_speed(curvature, comfort):
  """Absolute road-curve target shared by mapped and forward-model geometry.

  A fixed radius has a fixed speed, independent of measured or prior speed.
  These are optional comfort targets, not physical steering/braking limits.
  """
  import math
  if not math.isfinite(curvature) or not .0018 <= curvature <= .12:
    return None
  return math.sqrt(comfort.map_lateral_accel / curvature)
