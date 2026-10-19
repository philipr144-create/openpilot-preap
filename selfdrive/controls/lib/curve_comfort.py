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
  # Current More retains the former Mild mapped-curve target (3.4).
  # Standard and Mild progressively reduce optional mapped-curve slowing.
  # Signaled city-turn CornerAssist keeps its independent existing tuning.
  'more': CurveComfort('more', 1.0, 3.4, .8),
  'standard': CurveComfort('standard', 1.2, 3.8, .8),
  'mild': CurveComfort('mild', 1.6, 4.2, .6),
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
  """Geometry-owned road speed with a speed-dependent comfort budget.

  Solve v*v*k <= lateral_budget(v). Never feed measured vehicle speed back
  into the target: a fixed road radius must have a fixed target. City-turn
  tuning is independent. Existing high-speed profile maxima stay unchanged.
  """
  import math
  if not math.isfinite(curvature) or not .0018 <= curvature <= .12:
    return None
  scale = comfort.map_lateral_accel / PROFILES['mild'].map_lateral_accel
  def budget(v):
    points = ((0., 1.6), (13.4112, 2.0), (22.352, 3.4), (31.2928, 4.2))
    for (v0, a0), (v1, a1) in zip(points, points[1:]):
      if v <= v1:
        return scale * (a0 + (a1-a0) * (v-v0)/(v1-v0))
    return comfort.map_lateral_accel
  lo, hi = 0., math.sqrt(comfort.map_lateral_accel / curvature)
  for _ in range(40):
    mid = (lo+hi)/2
    if mid*mid*curvature <= budget(mid):lo = mid
    else:hi = mid
  return lo
