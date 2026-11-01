"""Bounded speed-frame alignment for hill preview only; no command authority.

The model predicts its own starting velocity. A modest vision-speed offset
does not invalidate the predicted changes in speed. Anchor those changes to
measured speed, preserving both raw and aligned stop/braking evidence. Other
longitudinal consumers retain their existing strict forecast checks.
"""
from dataclasses import dataclass
import math

from openpilot.selfdrive.controls.lib.model_forecast import ModelForecast, assess_model_forecast


@dataclass(frozen=True)
class GradeForecast:
  forecast: ModelForecast
  reason: str
  speed_offset_mps: float = 0.0


def assess_grade_forecast(model, speed):
  strict = assess_model_forecast(model, speed)
  if strict.valid:
    return GradeForecast(strict, 'STRICT')
  try:
    raw_speed = model.velocity.x[0]
    raw = assess_model_forecast(model, raw_speed)
    if not raw.valid:
      return GradeForecast(strict, 'INVALID_TRAJECTORY')
    offset = speed - raw_speed
    if (type(speed) not in (int, float) or not math.isfinite(speed)
        or not 8.9408 <= speed <= 36.0
        or not math.isfinite(offset) or abs(offset) > min(5.0, 0.20 * speed)):
      return GradeForecast(strict, 'SPEED_DISAGREEMENT')
    aligned = tuple(max(0.0, v + offset) for v in raw.speeds)
    near = [v for t, v in zip(raw.times, aligned) if t <= 3.0]
    forecast = ModelForecast(True, raw.times, aligned,
                            raw.planned_stop or min(aligned) <= 2.5,
                            raw.near_braking or min(near) < aligned[0] - 1.5)
    return GradeForecast(forecast, 'ALIGNED_MODEL_SPEED', offset)
  except (AttributeError, IndexError, TypeError, ValueError, OverflowError):
    return GradeForecast(strict, 'INVALID_TRAJECTORY')
