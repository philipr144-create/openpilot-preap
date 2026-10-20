"""Bounded hill preview and load tracking inside unified longitudinal control.

Model-relative pitch confirms an approaching climb. Tracking feedback then
maintains bounded support while the climb persists; no IMU reference is assumed.
The caller owns target speed, braking, preview limits and physical acceleration.
"""
from __future__ import annotations

import math

MAX_BOOST_MPS2 = 0.35
HIGH_SPEED_BOOST_MPS2 = 0.20
FULL_SPEED_MPS = 65.0 / 2.2369362921
OFF_SPEED_MPS = 80.0 / 2.2369362921
MIN_SPEED_MPS = 20.0 / 2.2369362921
TRIGGER_DEG = 2.0
RELEASE_DEG = 1.25
MIN_HOLD_SEC = 0.25
MAX_RISE_PER_SEC = 0.12
MAX_FALL_PER_SEC = 0.20
SPEED_TOLERANCE_MPS = 0.20  # avoid state chatter on small overshoot at 80 mph
MAX_CLIMB_SECONDS = 60.0
LOAD_FILTER_SECONDS = 1.0
PREVIEW_SUPPORT_MPS2 = 0.12


def _finite(value):
  try:
    x = float(value)
    return x if math.isfinite(x) else None
  except (TypeError, ValueError, OverflowError):
    return None


def _interpolate(xs, ys, query):
  if len(xs) != len(ys) or len(xs) < 4:
    return None
  pairs = []
  last = float('-inf')
  for xx, yy in zip(xs, ys):
    x, y = _finite(xx), _finite(yy)
    if x is None or y is None or x <= last:
      return None
    pairs.append((x, y))
    last = x
  if not (pairs[0][0] <= query <= pairs[-1][0]):
    return None
  for i in range(1, len(pairs)):
    x0, y0 = pairs[i - 1]
    x1, y1 = pairs[i]
    if x0 <= query <= x1:
      return y0 + (y1 - y0) * (query - x0) / (x1 - x0)
  return None


def future_pitch_rise_deg(model, speed=None):
  """Relative model pitch over 10–40 m, extending to 2.5 s at higher speed.

  Preserve this model's replay-validated sign convention; never extrapolate a
  short trajectory to fabricate the longer (at most 90 m) preview.
  """
  try:
    xs, ys = model.position.x, model.orientation.y
  except (AttributeError, TypeError):
    return None
  v = _finite(speed)
  far = max(40., min(90., 2.5 * v)) if v is not None and v >= 0. else 40.
  p10, pfar = _interpolate(xs, ys, 10.), _interpolate(xs, ys, far)
  if p10 is None or pfar is None:
    return None
  delta = math.degrees(pfar - p10)
  return delta if -12. < delta < 12. else None


class PredictiveGradeAssist:
  def __init__(self, dt=0.05):
    self.dt = max(.01, min(.10, _finite(dt) or .05))
    self.reset()

  def reset(self):
    self.proposal = self.output = self.hold = 0.0
    self.last_pitch_rise_deg = None
    self.triggered = self.climb_active = False
    self.climb_remaining = self.load_estimate = self.clear_seconds = 0.0
    self.block_reasons = []
    self.reason = 'OFF'

  def update(self, *, model, enabled, engaged, v_ego, a_ego,
             effective_speed, driver_speed, base_accel, mpc_accel,
             model_accel, brake_reason, model_stop, mpc_stop,
             corner_braking, force_decel, gas_pressed, brake_pressed,
             lead=None, leads=None, inputs_valid=False, forecast_valid=False,
             model_braking_required=True, max_accel=None):
    """Return additional acceleration, bounded by every existing authority.

    Cruise's neutral request is not an open-road prohibition. Lead MPC remains
    a ceiling. Missing forecasts, explicit braking and tighter speed/acceleration
    constraints cannot be reinterpreted as an uphill load.
    """
    delta = future_pitch_rise_deg(model, v_ego)
    self.last_pitch_rise_deg = delta
    self.triggered = False
    values = [_finite(v) for v in (v_ego, a_ego, effective_speed,
                                   driver_speed, base_accel, mpc_accel, model_accel)]
    upper = _finite(max_accel)
    if any(v is None for v in values) or upper is None:
      self.reset()
      self.reason = 'INVALID_INPUT'
      self.block_reasons = [self.reason]
      return 0.0
    speed, acceleration, target, driver_target, base, mpc, model_a = values
    active_leads = [c for c in (leads if leads is not None else (lead,))
                    if c is not None and bool(getattr(c, 'status', False))]
    permissions = ('ORDINARY', '') + (('LEAD_CONSTRAINT',) if active_leads else ())
    checks = [
      ('TOGGLE_OFF', not enabled), ('NOT_ENGAGED', not engaged),
      ('INVALID_INPUT', not inputs_valid), ('INVALID_FORECAST', not forecast_valid),
      ('PITCH_UNAVAILABLE', delta is None),
      ('MODEL_STOP', model_stop), ('MPC_STOP', mpc_stop),
      ('MODEL_BRAKING', model_braking_required or model_a <= -.20),
      ('CURVE_BRAKING', corner_braking), ('FORCE_DECEL', force_decel),
      ('DRIVER_GAS', gas_pressed), ('DRIVER_BRAKE', brake_pressed),
      ('SPEED_RANGE', speed < MIN_SPEED_MPS or speed > OFF_SPEED_MPS + SPEED_TOLERANCE_MPS),
      ('BRAKING_OWNER', str(brake_reason).upper() not in permissions),
      ('SPEED_RESTRICTION', target < driver_target - .25),
      ('NEGATIVE_BASE_REQUEST', base < -.015),
      ('MPC_BRAKING', mpc < -.015),
      ('ABOVE_TARGET', speed > target + .20),
      ('ACCEL_CEILING', upper <= max(0., base)),
    ]
    for candidate in active_leads:
      distance = _finite(getattr(candidate, 'dRel', None))
      relative = _finite(getattr(candidate, 'vRel', None))
      checks.append(('LEAD_NEAR_OR_CLOSING', distance is None or relative is None or
                     distance < max(12., 1.25 * speed) or relative < -.25))
    self.block_reasons = list(dict.fromkeys(name for name, blocked in checks if blocked))
    if self.block_reasons:
      blocks = self.block_reasons
      self.reset()
      self.last_pitch_rise_deg = delta
      self.block_reasons = blocks
      self.reason = blocks[0]
      return 0.0

    # Confirm preview before adding authority; one model-frame blips never arm.
    if delta >= TRIGGER_DEG:
      self.hold = min(1., self.hold + self.dt)
    elif delta < RELEASE_DEG:
      self.hold = 0.0
    if self.hold >= MIN_HOLD_SEC:
      self.climb_active = True
      self.climb_remaining = MAX_CLIMB_SECONDS
      self.triggered = True
    elif self.climb_active:
      self.climb_remaining = max(0., self.climb_remaining - self.dt)
      if self.climb_remaining == 0.:
        self.climb_active = False

    # A crest ends preview support now, but do not remove the tracked load
    # while the car is still reaching it. Limit the remaining bridge to 2.5 s.
    if delta <= -RELEASE_DEG:
      self.climb_remaining = min(self.climb_remaining, 2.5)
      self.hold = 0.0

    fade = max(0., min(1., (speed - FULL_SPEED_MPS) / (OFF_SPEED_MPS - FULL_SPEED_MPS)))
    cap = MAX_BOOST_MPS2 + fade * (HIGH_SPEED_BOOST_MPS2 - MAX_BOOST_MPS2)
    headroom = max(0., upper - max(0., base))
    if active_leads:
      headroom = min(headroom, max(0., mpc - max(0., base)))
    cap = min(cap, headroom)

    desired = 0.0
    if self.climb_active:
      # Estimate missing load from the previous *total* request. Including the
      # prior support prevents it disappearing merely because speed is held.
      # This bounded, filtered estimate is not an accumulating speed integral.
      residual = max(0., min(cap, base + self.output - acceleration))
      self.load_estimate += self.dt / (LOAD_FILTER_SECONDS + self.dt) * (residual - self.load_estimate)
      self.load_estimate = max(0., min(cap, self.load_estimate))
      deficit = target - speed
      preview = PREVIEW_SUPPORT_MPS2 if delta >= RELEASE_DEG else 0.
      feedback = .18 * max(0., deficit) + .15 * max(0., -acceleration)
      speed_guard = max(0., min(1., (target + .12 - speed) / .12))
      desired = min(cap, max(preview, self.load_estimate) + feedback) * speed_guard
      # Release a spent prediction once tracking says the load has disappeared.
      clear = delta < RELEASE_DEG and self.load_estimate < .025 and abs(deficit) < .10
      self.clear_seconds = self.clear_seconds + self.dt if clear else 0.
      if self.clear_seconds >= 2.:
        self.climb_active = False
        desired = 0.
    else:
      self.load_estimate = self.clear_seconds = 0.

    self.proposal = desired
    rate = MAX_RISE_PER_SEC if desired > self.output else MAX_FALL_PER_SEC
    self.output += max(-rate * self.dt, min(rate * self.dt, desired - self.output))
    self.output = max(0., min(cap, self.output))
    self.reason = ('APPLYING' if self.output > 1e-6 else
                   'NO_MPC_HEADROOM' if cap <= 0. else
                   'CONFIRMING_UPHILL' if self.hold > 0. and not self.climb_active else
                   'TRACKING_CLIMB' if self.climb_active else 'NO_UPHILL_PREDICTION')
    return self.output


class FileGradeToggle:
  """Fail-closed, cache-limited toggle; no Params registry/build changes.

  File is writable while parked by the companion CLI, and can later be bound
  to the MCU2 UI. An absent, malformed, unreadable or symlink file means OFF.
  """
  def __init__(self, path='/data/nap_settings/v9e_predictive_grade.json', dt=.05):
    self.path = path
    self.dt = _finite(dt) or .05
    self.enabled = False
    self._clock = .51  # first read is immediate

  def read(self):
    import json
    import os
    self._clock += self.dt
    if self._clock < .5:
      return self.enabled
    self._clock = 0.
    try:
      if os.path.islink(self.path):
        self.enabled = False
      else:
        with open(self.path, 'rb') as f:
          blob = f.read(512)
        config = json.loads(blob)
        self.enabled = isinstance(config, dict) and config.get('enabled') is True
    except (OSError, ValueError, TypeError):
      self.enabled = False
    return self.enabled
