"""Bounded Pre-AP cruise assistance; no lead or stop authority."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CruiseRefinementConfig:
  # Code-level tuning, in SI units. Adaptive Accel gates speed support.
  map_release_accel: float = 0.6
  speed_gain: float = 0.20
  falling_speed_gain: float = 0.15
  support_max: float = 0.25
  support_rise_jerk: float = 0.20
  accel_filter_seconds: float = 0.5


class CruiseRefinement:
  def __init__(self, dt, config=CruiseRefinementConfig()):
    self.dt = dt
    self.config = config
    self.map_cap = None
    self.filtered_accel = 0.0
    self.support = 0.0

  def curve_target(self, cap, cruise, speed, enabled):
    """Accept tighter ceilings immediately; release rising/missing caps gently."""
    if not enabled:
      self.map_cap = None
      return cruise
    target = min(cruise, cap) if cap is not None else cruise
    if self.map_cap is None:
      self.map_cap = target
    else:
      # Stale/missing map data must not keep asking for further slowdown.
      previous = max(self.map_cap, speed) if cap is None else self.map_cap
      self.map_cap = min(target, previous + self.config.map_release_accel * self.dt)
    return self.map_cap

  def speed_support(self, target, speed, accel, eligible):
    """Progressive, nonintegrating support, bounded by the old 0.25 bias.

    Falling-speed anticipation requires an actual speed deficit, so aEgo
    noise at set speed cannot request propulsion. The caller retains all
    braking, model-stop, lead, curve, and driver-override gates.
    """
    if not eligible or not all(math.isfinite(x) for x in (target, speed, accel)):
      self.filtered_accel = 0.0
      self.support = 0.0
      return 0.0
    self.filtered_accel += self.dt / (self.config.accel_filter_seconds + self.dt) * (accel - self.filtered_accel)
    error = max(0.0, target - speed - 0.10)
    falling = max(0.0, -self.filtered_accel) * min(1.0, error / 0.5)
    wanted = min(self.config.support_max, self.config.speed_gain * error + self.config.falling_speed_gain * falling)
    self.support = min(wanted, self.support + self.config.support_rise_jerk * self.dt)
    return self.support
