"""Opt-in model stop requests in PreAP Normal mode; no map stop authority."""
import math


class NormalModelStop:
  def __init__(self, dt):
    self.dt = dt
    self.active = False
    self.clear_time = 0.0

  def update(self, model, speed, *, enabled, valid, reset):
    if not enabled or not valid or reset:
      self.active = False
      self.clear_time = 0.0
      return False
    try:
      accel = float(model.action.desiredAcceleration)
      if not math.isfinite(accel):
        raise ValueError('invalid model acceleration')
      stopping = bool(model.action.shouldStop)
      # shouldStop is an imminent-stop flag. The full velocity forecast
      # provides the earlier approach request, before that flag asserts.
      times, speeds = list(model.velocity.t), list(model.velocity.x)
      complete = (len(times) >= 3 and len(times) == len(speeds)
                  and all(math.isfinite(v) for v in times + speeds + [speed])
                  and all(b > a for a, b in zip(times, times[1:]))
                  and times[0] <= 0.1 and times[-1] >= 9.9
                  and min(speeds) >= -0.1 and abs(speeds[0] - speed) <= 2.0)
      stopping = stopping or (complete and min(speeds) <= 0.5 and accel < 0.0)
    except AttributeError:
      # The explicit imminent-stop flag remains useful without a forecast.
      stopping = bool(getattr(model.action, 'shouldStop', False))
    except (TypeError, ValueError):
      self.active = False
      self.clear_time = 0.0
      return False
    if stopping:
      self.active = True
      self.clear_time = 0.0
    elif self.active:
      # A brief forecast dropout cannot jump directly back to cruise torque.
      self.clear_time = self.clear_time + self.dt if accel >= 0.0 else 0.0
      if self.clear_time >= 0.30 - 1e-6:
        self.active = False
        self.clear_time = 0.0
    return self.active
