"""Experimental acceleration cap; never overrides a stronger planner slowdown."""
import json
import math
import time

SETTINGS = '/data/nap_turn_settings.json'


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

  def refresh(self, now=None):
    now = time.monotonic() if now is None else now
    if now-self.last_config >= 1 or now < self.last_config:
      self.config = settings()
      if not hasattr(self, 'params'):
        from openpilot.common.params import Params
        self.params = Params()
      param_val = self.params.get("NAPCornerAssist")
      if param_val is not None:
        self.config['corner_assist'] = (param_val == b"1")
      self.last_config = now

  def update(self, base, speed, curvature, coast, active, now):
    """City-speed experiment, with limited additional deceleration and jerk.

    This caps acceleration, not braking authority. A lead/stop/model request
    below the cap passes through unchanged. It cannot guarantee a safe speed.
    """
    valid = (self.config['corner_assist'] and active and curvature is not None
             and all(math.isfinite(v) for v in (base, speed, curvature, coast, now))
             and 2.5 <= speed <= 15.65 and 0 <= curvature <= .25)
    dt = now-self.last_time if self.last_time is not None else .05
    self.last_time = now
    if not valid or not 0 < dt <= .25:
      self.cap = None
      self.reason = 'Off or vehicle/model input unavailable; normal planner'
      return base
    if self.cap is None:
      self.cap = max(-.8, base)
    # Prototype comfort values; these require road-log validation before use.
    turning = curvature >= .008 and speed*speed*curvature >= .6
    if turning:
      target_speed = math.sqrt(1.2/max(curvature, 1e-6))
      approach_distance = max(8., speed*2.5)
      slow = min(0., (target_speed*target_speed-speed*speed)/(2*approach_distance))
      # Reuse the planner's coast estimate, bounded to mild deceleration.
      target_cap = max(-.8, min(slow, max(-.4, min(0., coast))))
      self.reason = 'Curve: gradual slowdown/coast cap'
    else:
      target_cap = base
      self.reason = 'Straightening: release acceleration cap'
    self.cap += max(-.4*dt, min(.4*dt, target_cap-self.cap))
    if not turning and self.cap >= base:
      self.cap = None
      self.reason = 'Normal planner'
      return base
    return min(base, self.cap)

  def apply(self, base, sm, CP, reset, coast):
    now = time.monotonic()
    try:
      cs, cc, model = sm['carState'], sm['carControl'], sm['modelV2']
      fresh = all(sm.valid[s] and sm.alive[s] and 0 <= now-sm.logMonoTime[s]/1e9 <= .5
                  for s in ('carState', 'carControl', 'modelV2'))
      # Exactly one physical indicator must be active; hazards do not qualify.
      signaled = bool(cs.leftBlinker) != bool(cs.rightBlinker)
      active = (signaled and fresh and not reset and cs.canValid and cc.longActive and cc.latActive
                and not cs.brakePressed and not cs.gasPressed and not cs.steeringPressed
                and str(cs.gearShifter) == 'drive')
      curve = path_curve(model.position.x, model.position.y, min(40., max(12., cs.vEgo*3)))
      # Require usable model geometry even when steering indicates a curve.
      if curve is not None and CP.steerRatio > 0 and CP.wheelbase > 0:
        actual = abs(math.tan(math.radians(cs.steeringAngleDeg)/CP.steerRatio)/CP.wheelbase)
        curve = max(curve, actual)
      return self.update(base, cs.vEgo, curve, coast, active, now)
    except (AttributeError, ValueError, TypeError, KeyError, OverflowError):
      self.cap = None
      self.reason = 'Invalid input; normal planner'
      return base
