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
      # A loosening curve must not retain an obsolete target below actual
      # speed and keep braking while the current corridor permits recovery.
      # Tighter ceilings still win immediately; command jerk is bounded later.
      previous = self.map_cap
      if target > previous:
        previous = max(previous, min(speed, target))
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


def model_support_weight(model, speed, request):
  """Allow bounded compensation for weak model coasting, never a planned stop.

  Full support at zero demand fades continuously to zero at -0.2 m/s².
  Negative requests additionally require a complete, nearly speed-holding
  three-second prediction. Missing/invalid predictions preserve the request.
  """
  if request >= 0.0:
    return 1.0
  if request <= -0.2:
    return 0.0
  try:
    times, speeds = list(model.velocity.t), list(model.velocity.x)
    if (len(times) < 3 or len(times) != len(speeds)
        or not all(math.isfinite(x) for x in times + speeds + [speed, request])
        or any(b <= a for a, b in zip(times, times[1:]))
        or times[0] > 0.1 or times[-1] < 3.0
        or abs(speeds[0] - speed) > 2.0):
      return 0.0
    near = [v for t, v in zip(times, speeds) if 0.0 <= t <= 3.0]
    # Model velocity has its own ego-speed estimate. Compare its predicted
    # change with that estimate; a bounded initial offset from wheel speed
    # is not a predicted deceleration. The 2 m/s plausibility gate above
    # still rejects unreliable estimates.
    if not near or min(near) < speeds[0] - 0.5:
      return 0.0
    return max(0.0, 1.0 + request / 0.2)
  except (AttributeError, TypeError, ValueError):
    return 0.0


def mild_transition_forecast(model, speed):
  """Recognize a complete, nearly steady-speed forecast, never a planned stop."""
  try:
    ts, vs = list(model.velocity.t), list(model.velocity.x)
    if (len(ts) < 3 or len(ts) != len(vs)
        or not all(math.isfinite(x) for x in ts + vs + [speed])
        or any(b <= a for a, b in zip(ts, ts[1:]))
        or ts[0] > .1 or ts[-1] < 9.9 or speed < 8.
        or abs(vs[0] - speed) > 2. or model.action.shouldStop):
      return False
    near = [v for t, v in zip(ts, vs) if t <= 6.]
    return min(near) >= vs[0] - .75 and min(vs) >= max(5., vs[0] - 1.5)
  except (AttributeError, TypeError, ValueError):
    return False


def shape_mild_transition(request, previous, dt, eligible):
  """Smooth only independently validated model/cruise disagreements.

  The caller bypasses this for any lead, stop, binding speed ceiling, MPC
  braking, or stronger model braking. Negative entry removes added propulsion
  immediately, then ramps regen. Recovery releases regen before adding drive.
  """
  if not eligible or not all(math.isfinite(v) for v in (request, previous, dt)) or not 0 < dt <= .1:
    return request
  step = .4 * dt
  if request < previous:
    return max(request, min(0., previous) - step)
  if previous < 0.:
    return min(request, previous + step)
  return request
