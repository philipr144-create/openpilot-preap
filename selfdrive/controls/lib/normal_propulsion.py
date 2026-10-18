"""Shared recovery permission; request selection and braking retain authority."""
import math


class NormalPropulsion:
  def __init__(self, dt):
    self.dt = dt
    self.evidence_time = 0.0
    self.reason = "OFF"

  def update(self, model, speed, target, *, enabled, valid, blocked):
    credible = False
    self.reason = "BLOCKED" if blocked or not enabled or not valid else "FORECAST"
    if enabled and valid and not blocked:
      try:
        times, speeds = list(model.velocity.t), list(model.velocity.x)
        accel = float(model.action.desiredAcceleration)
        forecast_valid = (not model.action.shouldStop and math.isfinite(accel)
                          and all(math.isfinite(v) for v in [speed, target] + times + speeds)
                          and speed >= 0.0 and len(times) >= 3 and len(times) == len(speeds)
                          and 0.0 <= times[0] <= .1 and times[-1] >= 9.9
                          and all(b > a for a, b in zip(times, times[1:]))
                          and min(speeds) >= 0.0 and abs(speeds[0] - speed) <= 2.0)
        if forecast_valid:
          # Retain the established positive-model recovery case.
          positive_model = (target > speed + .10 and accel >= .10
                            and min(speeds) >= speeds[0] - .5)
          # A weak coast preference is not an acceleration ceiling. Allow
          # ordinary below-target recovery only with a complete forecast,
          # no approaching stop, and bounded predicted speed loss. This
          # grants permission, never adds acceleration to a selected request.
          near_speeds = [v for t, v in zip(times, speeds) if t <= 3.0]
          speed_recovery = (target > speed + 1.0 and speed >= 5.0
                            and accel >= -.10 and min(speeds) > 2.5
                            and min(near_speeds) >= speeds[0] - 1.0
                            and min(speeds) >= min(speed, speeds[0]) - 1.5)
          credible = positive_model or speed_recovery
          if credible:
            self.reason = "MODEL_DRIVE" if positive_model else "SPEED_DEFICIT"

      except (AttributeError, TypeError, ValueError):
        credible = False
    self.evidence_time = self.evidence_time + self.dt if credible else 0.0
    return self.evidence_time >= .30 - 1e-6
