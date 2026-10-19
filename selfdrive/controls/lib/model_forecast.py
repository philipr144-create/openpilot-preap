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


@dataclass(frozen=True)
class ModelIntent:
  reason: str
  braking_required: bool
  support_weight: float


def assess_model_intent(model, speed, request, forecast=None):
  """One model-intent decision for recovery, arbitration, and final handoff.

  Bounded support is only available with a complete forecast and no planned
  stop or near-term braking. Leads, geometry ceilings, and driver ownership
  are separate constraints applied by the caller; this grants no authority.
  """
  f = assess_model_forecast(model, speed) if forecast is None else forecast
  if f.planned_stop:
    return ModelIntent('PLANNED_MODEL_BRAKING', True, 0.0)
  if not math.isfinite(request):
    return ModelIntent('UNPROVEN_MODEL_BRAKING', True, 0.0)
  if request <= -.75:
    return ModelIntent('STRONG_BRAKING', True, 0.0)
  if f.near_braking:
    return ModelIntent('PLANNED_MODEL_BRAKING', True, 0.0)
  if request >= 0.:
    return ModelIntent('MODEL_DRIVE', False, 1.0)
  # Support the established 3-second legacy forecast and the
  # complete 10-second forecast.
  #
  # Reject incomplete extended forecasts. A truncated 10-second
  # prediction must not gain recovery authority merely because
  # it contains three seconds of otherwise valid samples.
  legacy_short_horizon = (
    f.valid
    and len(f.times) == 4
    and 3.0 <= f.times[-1] <= 3.1
  )

  if not (f.full_horizon or legacy_short_horizon):
    return ModelIntent('UNPROVEN_MODEL_BRAKING', True, 0.0)
  near = [v for t, v in zip(f.times, f.speeds) if t <= 3.]
  loss = max(0.0, f.speeds[0] - min(near))

  # Conservative recovery; preserve baseline braking authority.
  request_weight = max(0.0, min(1.0, 1.0 + request / 0.20))

  # Do not support a genuine predicted slowdown.
  if loss <= 0.30:
    forecast_weight = 1.0
  elif loss >= 0.50:
    forecast_weight = 0.0
  else:
    t = (loss - 0.30) / 0.20
    forecast_weight = 1.0 - (3.0*t*t - 2.0*t*t*t)

  weight = request_weight * forecast_weight
  return ModelIntent('MODEL_COAST', False, weight)
