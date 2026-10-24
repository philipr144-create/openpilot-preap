"""Bounded predictive lane clearance for Pre-AP cruising; positive Y is right.

This is a soft trajectory constraint, not collision avoidance or a guarantee of
lane containment. Physical curvature and actuator limits remain downstream.
"""
import math
from bisect import bisect_right


def clip(value, low, high):
  return max(low, min(high, value))


def sample(curve, distance):
  xs, ys = list(curve.x), list(curve.y)
  if (len(xs) < 2 or len(xs) != len(ys)
      or not all(math.isfinite(x) for x in xs + ys)
      or any(b <= a for a, b in zip(xs, xs[1:]))
      or not xs[0] <= distance <= xs[-1]):
    return None
  i = min(bisect_right(xs, distance), len(xs) - 1)
  return ys[i-1] + (ys[i] - ys[i-1]) * (distance - xs[i-1]) / (xs[i] - xs[i-1])


class LaneClearanceGuard:
  # Conservative body envelope, excluding mirrors, for this Pre-AP-only guard.
  HALF_WIDTH = 1.0
  CLEARANCE = 0.25
  MAX_ACCEL = 0.35  # extra lateral m/s^2, before existing total limits
  MAX_JERK = 0.60   # extra lateral m/s^3, including fade-out
  HORIZONS = (0.8, 1.1, 1.4)
  QUALIFY_TIME = 0.25

  def __init__(self):
    self.accel = self.target_accel = self.good_time = 0.0
    self.frame = None
    self.frame_age = 0.0
    self.reason = 'off'
    self.predicted_margin = None
    self.limited = False

  def reset(self, reason='off'):
    self.accel = self.target_accel = self.good_time = 0.0
    self.frame = None
    self.frame_age = 0.0
    self.reason = reason
    self.predicted_margin = None
    self.limited = False
    return 0.0

  def update(self, model, speed, curvature, *, enabled, active, healthy,
             overriding, maneuver=False, dt=0.01):
    if not enabled:
      return self.reset('off')
    if (not active or not healthy or overriding or maneuver
        or not math.isfinite(speed) or not math.isfinite(curvature)
        or not math.isfinite(dt) or not 0 < dt <= 0.1):
      return self.reset('control/driver gate')
    if (not 8.9408 <= speed <= 36.0 or abs(curvature) > 0.004
        or abs(curvature) * speed**2 > 1.8):
      return self.reset('speed/curve gate')
    if str(model.meta.laneChangeState) != 'off':
      return self.reset('lane change')

    frame = getattr(model, 'frameId', None)
    new_frame = frame is None or frame != self.frame
    if frame is not None and self.frame is not None and frame < self.frame:
      return self.reset('out-of-order model')
    self.frame_age = 0.0 if new_frame else self.frame_age + dt
    if new_frame:
      self.frame = frame
      target = self._target(model, speed)
      if target is None:
        self.good_time = 0.0
        self.target_accel = 0.0
      else:
        self.target_accel = target
    elif self.frame_age > 0.15:
      self.reason = 'stale model'
      self.target_accel = self.good_time = 0.0

    if self.reason in ('clear', 'correcting', 'limited'):
      self.good_time += dt
    target = self.target_accel if self.good_time >= self.QUALIFY_TIME else 0.0
    self.accel += clip(target - self.accel, -self.MAX_JERK * dt, self.MAX_JERK * dt)
    self.accel = clip(self.accel, -self.MAX_ACCEL, self.MAX_ACCEL)
    return self.accel / speed**2

  def _target(self, model, speed):
    self.predicted_margin = None
    self.limited = False
    probs = list(model.laneLineProbs)
    desire = list(model.meta.desireState)
    if (len(probs) < 3 or len(model.laneLines) < 3
        or not all(math.isfinite(p) and 0 <= p <= 1 for p in probs)
        or min(probs[1:3]) < 0.90 or not desire
        or not all(math.isfinite(p) for p in desire) or desire[0] < 0.95):
      self.reason = 'confidence/intent'
      return None
    constraints, widths, margins = [], [], []
    for horizon in self.HORIZONS:
      distance = speed * horizon
      path = sample(model.position, distance)
      left = sample(model.laneLines[1], distance)
      right = sample(model.laneLines[2], distance)
      if path is None or left is None or right is None:
        self.reason = 'invalid geometry'
        return None
      width = right - left
      if (not 2.5 <= width <= 4.2 or path < left or path > right):
        self.reason = 'lane geometry'
        return None
      widths.append(width)
      margins.append(min(path - left, right - path) - self.HALF_WIDTH)
      lower = left + self.HALF_WIDTH + self.CLEARANCE - path
      upper = right - self.HALF_WIDTH - self.CLEARANCE - path
      # Small-angle additional-motion prediction: dy = 0.5 * da * t^2.
      constraints.append((0.5 * horizon**2, lower, upper))
    if max(widths) - min(widths) > 0.6:
      self.reason = 'changing lane width'
      return None
    self.predicted_margin = min(margins)
    if self.predicted_margin < -0.50:
      self.reason = 'large model departure'
      return None

    lo = max(low / factor for factor, low, high in constraints)
    hi = min(high / factor for factor, low, high in constraints)
    if lo <= hi:
      target = clip(0.0, lo, hi)
    else:
      # No single acceleration meets all samples. Minimize squared corridor
      # violations instead of discontinuously switching between lane edges.
      lo, hi = -self.MAX_ACCEL, self.MAX_ACCEL
      for _ in range(32):
        mid = (lo + hi) / 2
        gradient = sum(factor * (factor * mid - clip(factor * mid, low, high))
                       for factor, low, high in constraints)
        if gradient > 0:
          hi = mid
        else:
          lo = mid
      target = (lo + hi) / 2

    # Do not pull toward a high-probability, laterally displaced nearby lead
    # that may explain the model's deviation. This is a conservative veto,
    # not an obstacle detector or a substitute for longitudinal lead handling.
    for lead in getattr(model, 'leadsV3', ()):
      if lead.prob > 0.5 and len(lead.x) and len(lead.y):
        x, y = lead.x[0], lead.y[0]
        if (math.isfinite(x) and math.isfinite(y) and 0 < x < 40
            and 0.6 < abs(y) < 3.5 and target * y > 0):
          self.reason = 'displaced lead'
          return None
    self.limited = abs(target) > self.MAX_ACCEL
    self.reason = 'limited' if self.limited else ('correcting' if abs(target) > 1e-6 else 'clear')
    return clip(target, -self.MAX_ACCEL, self.MAX_ACCEL)
