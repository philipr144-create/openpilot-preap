"""Source-independent Experimental acceleration handoff; no speed/stop authority."""
import math
from opendbc.car.tesla.preap.accel_policy import ACCEL_RISE, REGEN_RELEASE_JERK, personality_index

DECEL_JERK = {0: .8, 1: .6, 2: .4}


def braking_priority(model, speed, request, mpc, has_lead, stop, force_decel,
                     effective_speed, driver_speed, corner_speed=None, valid=True):
  """Explain when comfort may not delay a stronger braking request."""
  if stop:
    return 'STOP'
  if force_decel:
    return 'FORCE_DECEL'
  if not valid:
    return 'INVALID_INPUT'
  if has_lead and (mpc < 0. or request < 0.):
    return 'LEAD_BRAKING'
  if request <= -1. or float(model.action.desiredAcceleration) <= -.75:
    return 'STRONG_BRAKING'
  if ((request < 0. and speed >= effective_speed - .3)
      or (speed > driver_speed + .5 and mpc < 0.)
      or (type(corner_speed) in (int, float) and math.isfinite(corner_speed) and speed >= corner_speed)):
    return 'SPEED_CEILING'
  if model.action.desiredAcceleration >= 0.:
    return 'ORDINARY'
  try:
    ts, vs = list(model.velocity.t), list(model.velocity.x)
    if (len(ts) < 3 or len(ts) != len(vs)
        or not all(math.isfinite(x) for x in ts + vs + [speed])
        or any(b <= a for a, b in zip(ts, ts[1:]))
        or ts[0] > .1 or ts[-1] < 3. or abs(vs[0] - speed) > 2.):
      return 'UNPROVEN_MODEL_BRAKING'
    near = [v for t, v in zip(ts, vs) if t <= 3.]
    if min(vs) <= 2.5 or min(near) < vs[0] - 1.5:
      return 'PLANNED_MODEL_BRAKING'
    return 'ORDINARY'
  except (AttributeError, TypeError, ValueError):
    return 'UNPROVEN_MODEL_BRAKING'


def handoff_acceleration(request, previous, dt, personality, lower, upper, priority='ORDINARY'):
  """Carry the previous final command across source changes, with one slew policy.

  Lowered physical/profile bounds apply immediately. Priority braking may get
  stronger immediately; release still travels continuously through neutral.
  Source names deliberately are not an input and cannot reset this history.
  """
  if not all(math.isfinite(v) for v in (request, previous, dt, lower, upper)) or not 0 < dt <= .1 or lower > upper:
    return request
  target = max(lower, min(upper, request))
  previous = max(lower, min(upper, previous))
  p = personality_index(personality)
  if target < previous:
    value = target if priority != 'ORDINARY' else max(target, previous - DECEL_JERK.get(p, .6) * dt)
  elif previous < 0.:
    release = REGEN_RELEASE_JERK.get(p, .8)
    zero_time = -previous / release
    value = previous + release * dt if dt <= zero_time else ACCEL_RISE.get(p, .4) * (dt - zero_time)
    value = min(target, value)
  else:
    value = min(target, previous + ACCEL_RISE.get(p, .4) * dt)
  return max(lower, min(upper, value))
