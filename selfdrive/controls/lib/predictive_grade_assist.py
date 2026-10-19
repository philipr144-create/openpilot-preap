"""NAP V9e: optional, bounded model-pitch *trigger* for uphill support.

This module is independent of the IMU pitch-reference calibration. The 3-D
vision model identifies an upcoming *change* in pitch. It never commands a stop,
regen, or a speed target; it adds at most 0.08 m/s^2 to an already non-braking
longitudinal request, subject to the lead-aware MPC limit and the existing
planner limits. The caller must apply its normal final acceleration clipping.

The boolean toggle is NAPPredictiveGradeAssist (default False).
"""
from __future__ import annotations

import math

MAX_BOOST_MPS2 = 0.08
FULL_SPEED_MPS = 50.0 / 2.2369362921
OFF_SPEED_MPS = 55.0 / 2.2369362921
MIN_SPEED_MPS = 20.0 / 2.2369362921
TRIGGER_DEG = 2.0
RELEASE_DEG = 1.25
MIN_HOLD_SEC = 0.25
MAX_RISE_PER_SEC = 0.055    # m/s^3; >=1.45 s from 0 to 0.08
MAX_FALL_PER_SEC = 0.095


def _finite(value):
  try:
    x = float(value)
    return x if math.isfinite(x) else None
  except (TypeError, ValueError, OverflowError):
    return None


def _interpolate(xs, ys, query):
  # Only valid, ordered, complete model trajectories are accepted; never infer
  # an index-based "40 m" from one short or malformed array.
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


def future_pitch_rise_deg(model):
  """Return predicted pitch change from 10 to 40 meters ahead, or None.

  For this particular recorded model, positive delta orientation.y(40m-10m)
  preceded a positive change in the logged vehicle pitch. Do not generalize
  this convention across model versions without replay validation.
  """
  try:
    xs = model.position.x
    ys = model.orientation.y
  except (AttributeError, TypeError):
    return None
  p10 = _interpolate(xs, ys, 10.)
  p40 = _interpolate(xs, ys, 40.)
  if p10 is None or p40 is None:
    return None
  delta = math.degrees(p40 - p10)
  return delta if -12. < delta < 12. else None


class PredictiveGradeAssist:
  def __init__(self, dt=0.05):
    self.dt = max(.01, min(.10, _finite(dt) or .05))
    self.reset()

  def reset(self):
    self.proposal = 0.0
    self.output = 0.0
    self.hold = 0.0
    self.last_pitch_rise_deg = None
    self.triggered = False

  def update(self, *, model, enabled, engaged, v_ego, a_ego,
             effective_speed, driver_speed, base_accel, mpc_accel,
             model_accel, brake_reason, model_stop, mpc_stop,
             corner_braking, force_decel, gas_pressed, brake_pressed,
             lead=None):
    """Return *additional* positive acceleration, never final accel.

    `lead` is None or the radar lead message (status/dRel/vRel). The model
    signal only triggers a small boost when normal planner permission exists.
    In particular, a present lead is not a blanket veto, but a closing/near
    lead is. `mpc_accel` remains a hard ceiling for the combined command.
    """
    self.last_pitch_rise_deg = future_pitch_rise_deg(model)
    self.triggered = False
    values = [_finite(v) for v in (v_ego, a_ego, effective_speed,
                                    driver_speed, base_accel, mpc_accel, model_accel)]
    if any(v is None for v in values):
      self.reset()
      return 0.0
    speed, acceleration, target, driver_target, base, mpc, model_a = values

    hard_block = (not enabled or not engaged or model_stop or mpc_stop or
                  corner_braking or force_decel or gas_pressed or brake_pressed or
                  speed < MIN_SPEED_MPS or speed >= OFF_SPEED_MPS or
                  str(brake_reason).upper() not in ('ORDINARY', ''))
    # An active map/nav/curve speed ceiling is an owner even *below* that cap.
    # Do not use an upcoming hill to undo curve slowdown.
    hard_block |= target < driver_target - .25
    # Prevent an uphill trigger from fighting actual requests to slow down.
    hard_block |= (base < -.015 or base >= .18 or mpc <= .02
                   or model_a <= .035 or model_a - base <= .06)
    # Near cruise speed or legitimately losing speed, but not speed-chasing.
    hard_block |= speed > target + (.45 if acceleration < -.12 else .20)
    # Reject overly strong actual acceleration: it needs no extra help.
    hard_block |= acceleration > .20

    if lead is not None and bool(getattr(lead, 'status', False)):
      distance = _finite(getattr(lead, 'dRel', None))
      relative_speed = _finite(getattr(lead, 'vRel', None))
      # No additional help while closing on a lead or following closely.
      hard_block |= (distance is None or relative_speed is None or
                     distance < max(12.0, 1.25 * speed) or relative_speed < -.25)

    delta = self.last_pitch_rise_deg
    valid_rise = delta is not None and delta >= TRIGGER_DEG
    if hard_block or delta is None or delta < RELEASE_DEG:
      self.hold = 0.0
    elif valid_rise:
      self.hold = min(1., self.hold + self.dt)

    if hard_block:
      # Safety-critical inhibition takes precedence over a smooth release.
      self.output = 0.0
      return 0.0

    self.triggered = self.hold >= MIN_HOLD_SEC and delta >= RELEASE_DEG
    speed_gain = min(1.0, max(0.0, (OFF_SPEED_MPS - speed) /
                              (OFF_SPEED_MPS - FULL_SPEED_MPS)))
    # MPC is the lead-aware owner. Never exceed its immediate accel request.
    headroom = max(0.0, mpc - max(0., base))
    desired = min(MAX_BOOST_MPS2 * speed_gain, headroom) if self.triggered else 0.0
    limit = (MAX_RISE_PER_SEC if desired > self.output else MAX_FALL_PER_SEC) * self.dt
    self.output += max(-limit, min(limit, desired - self.output))
    self.output = min(max(0., self.output), max(0., mpc - max(0., base)))
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
