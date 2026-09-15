"""Experimental, bounded lane-center feedback for Pre-AP model curvature.

Model coordinates and curvature are positive right. This does not alter model
messages, replace the model trajectory, or bypass downstream actuator limits.
"""
import math
from bisect import bisect_right


def clip(x, lo, hi):
  return max(lo, min(hi, x))


def sample_curve(curve, distance):
  xs, ys = list(curve.x), list(curve.y)
  if len(xs) < 2 or len(xs) != len(ys):
    return None
  if not all(math.isfinite(v) for v in xs + ys):
    return None
  if any(b <= a for a, b in zip(xs, xs[1:])) or not xs[0] <= distance <= xs[-1]:
    return None
  i = min(bisect_right(xs, distance), len(xs) - 1)
  return ys[i-1] + (ys[i] - ys[i-1]) * (distance - xs[i-1]) / (xs[i] - xs[i-1])


class LaneCenteringAssist:
  MAX_ACCEL = 0.15  # m/s^2 of additional lateral acceleration
  MAX_JERK = 0.10  # m/s^3 while applying or fading this correction

  # Signed lane-position bias.
  # Model Y is positive to the right.
  #
  # -3 = Left 12"
  # -2 = Left 9"
  # -1 = Left 6"
  #  0 = Center
  #  1 = Right 6"
  #  2 = Right 9"
  #  3 = Right 12"
  OFFSET_BY_SETTING = {
    -3: -0.30,
    -2: -0.23,
    -1: -0.15,
     0:  0.00,
     1:  0.15,
     2:  0.23,
     3:  0.30,
  }

  def __init__(self):
    self.accel = 0.0
    self.good_time = 0.0

  def reset(self):
    self.accel = self.good_time = 0.0
    return 0.0

  def update(self, model, speed, curvature, *, enabled, active, healthy,
             overriding, strength=1, offset_setting=0, dt=0.01):
    if (not enabled or not active or overriding or not healthy
        or not math.isfinite(speed) or not math.isfinite(curvature)
        or not math.isfinite(dt) or not 0 < dt <= 0.1
        or str(model.meta.laneChangeState) != 'off'):
      return self.reset()
    # Only ordinary road cruising; no sharp curves or low-speed intersections.
    if speed < 8.9408 or speed > 31.3 or abs(curvature) > 0.004 or abs(curvature) * speed**2 > 1.5:
      return self.reset()
    target = self._target(model, speed, strength, offset_setting)
    if target is None:
      self.good_time = 0.0
      target = 0.0
    else:
      self.good_time += dt
      if self.good_time < 0.5:
        target = 0.0
    self.accel += clip(target - self.accel, -self.MAX_JERK * dt, self.MAX_JERK * dt)
    self.accel = clip(self.accel, -self.MAX_ACCEL, self.MAX_ACCEL)
    return self.accel / speed**2

  def _target(self, model, speed, strength, offset_setting):
    probs = list(model.laneLineProbs)
    if len(probs) < 3 or len(model.laneLines) < 3:
      return None
    if not all(math.isfinite(p) and 0 <= p <= 1 for p in probs):
      return None
    confidence = min(probs[1], probs[2])
    if confidence < 0.85:
      return None
    # Do not pull against a model desire to turn, merge or change lanes.
    desire = list(model.meta.desireState)
    if not desire or not all(math.isfinite(p) for p in desire) or desire[0] < 0.95:
      return None
    lookahead = clip(speed * 1.5, 15.0, 40.0)

    try:
      offset_setting = int(offset_setting)
    except (TypeError, ValueError):
      offset_setting = 0

    lane_offset = self.OFFSET_BY_SETTING.get(offset_setting, 0.0)
    errors, widths = [], []
    for distance in (5.0, lookahead):
      path = sample_curve(model.position, distance)
      left = sample_curve(model.laneLines[1], distance)
      right = sample_curve(model.laneLines[2], distance)
      if path is None or left is None or right is None:
        return None
      width = right - left
      if not 2.6 <= width <= 4.2 or not left + 0.35 < path < right - 0.35:
        return None
      lane_center = (left + right) / 2
      target_center = lane_center + lane_offset
      error = target_center - path

      # Reject geometry that would require an implausibly large correction.
      if abs(error) > 0.75:
        return None
      errors.append(error)
      widths.append(width)
    if abs(widths[1] - widths[0]) > 0.4 or abs(errors[1] - errors[0]) > 0.3:
      return None
    # Near/far disagreement can indicate a merge, curve transition, or noise.
    if errors[0] * errors[1] < 0 and min(abs(e) for e in errors) > 0.03:
      return None
    slope_error = (errors[1] - errors[0]) / (lookahead - 5.0)
    error = errors[0] - 5.0 * slope_error
    error = math.copysign(max(0.0, abs(error) - 0.03), error)
    # Position bias may intentionally request up to ~30 cm.
    # MAX_ACCEL remains the final authority limit.
    error = clip(error, -0.35, 0.35)
    gain = {1: 0.15, 2: 0.25, 3: 0.4}.get(strength, 0.0)
    weight = clip((confidence - 0.85) / 0.10, 0.0, 1.0)
    weight *= clip((speed - 8.9408) / 2.2352, 0.0, 1.0)
    # Position and heading feedback relative to the model path. The heading
    # term damps the correction as the path approaches the lane center.
    # Critical damping in an ideal straight-road bicycle model; real vehicle
    # response, model feedback, limits and latency still require validation.
    delta = 2 * gain * error / lookahead**2
    delta += 2 * math.sqrt(2 * gain) * slope_error / lookahead
    return clip(weight * delta * speed**2, -self.MAX_ACCEL, self.MAX_ACCEL)
