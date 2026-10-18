"""Shared forecast validity and braking evidence; no acceleration authority."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ModelForecast:
  valid: bool = False
  times: tuple = ()
  speeds: tuple = ()
  planned_stop: bool = False
  near_braking: bool = False

  @property
  def full_horizon(self):
    return self.valid and self.times[-1] >= 9.9


def assess_model_forecast(model, speed):
  explicit_stop = bool(getattr(getattr(model, 'action', None), 'shouldStop', False))
  try:
    ts, vs = tuple(model.velocity.t), tuple(model.velocity.x)
    if (len(ts) < 3 or len(ts) != len(vs)
        or not all(math.isfinite(x) for x in ts + vs + (speed,))
        or not 0 <= ts[0] <= .1 or ts[-1] < 3.
        or any(b <= a for a, b in zip(ts, ts[1:]))
        or min(vs) < -.1 or abs(vs[0] - speed) > 2.):
      return ModelForecast(planned_stop=explicit_stop)
    near = [v for t, v in zip(ts, vs) if t <= 3.]
    return ModelForecast(True, ts, vs, explicit_stop or min(vs) <= 2.5,
                         min(near) < vs[0] - 1.5)
  except (AttributeError, TypeError, ValueError, OverflowError):
    return ModelForecast(planned_stop=explicit_stop)
