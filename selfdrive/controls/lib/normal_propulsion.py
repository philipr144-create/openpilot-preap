"""Validated Normal-mode propulsion permission; MPC retains all constraints."""
import math


class NormalPropulsion:
  def __init__(self, dt):
    self.dt = dt
    self.evidence_time = 0.0

  def update(self, model, speed, target, *, enabled, valid, blocked):
    credible = False
    if enabled and valid and not blocked:
      try:
        times, speeds = list(model.velocity.t), list(model.velocity.x)
        accel = float(model.action.desiredAcceleration)
        credible = (not model.action.shouldStop and math.isfinite(accel) and accel >= .10
                    and all(math.isfinite(v) for v in [speed, target] + times + speeds)
                    and speed >= 0.0 and target > speed + .10
                    and len(times) >= 3 and len(times) == len(speeds)
                    and times[0] <= .1 and times[-1] >= 9.9
                    and all(b > a for a, b in zip(times, times[1:]))
                    and min(speeds) >= 0.0 and abs(speeds[0] - speed) <= 2.0
                    and min(speeds) >= speeds[0] - .5)
      except (AttributeError, TypeError, ValueError):
        credible = False
    self.evidence_time = self.evidence_time + self.dt if credible else 0.0
    return self.evidence_time >= .30 - 1e-6
