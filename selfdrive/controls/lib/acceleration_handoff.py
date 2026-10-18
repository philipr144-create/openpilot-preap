"""Pre-AP acceleration handoff with shared cruise capture and braking priority."""
import math
from openpilot.selfdrive.controls.lib.model_forecast import assess_model_forecast
from opendbc.car.tesla.preap.accel_policy import ACCEL_RISE, REGEN_RELEASE_JERK, personality_index

DECEL_JERK = {0: .8, 1: .6, 2: .4}


def braking_priority(model, speed, request, mpc, has_lead, stop, force_decel,
                     effective_speed, driver_speed, corner_speed=None, valid=True, forecast=None):
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
  forecast = assess_model_forecast(model, speed) if forecast is None else forecast
  if forecast.planned_stop or forecast.near_braking:
    return 'PLANNED_MODEL_BRAKING'
  if model.action.desiredAcceleration >= 0.:
    return 'ORDINARY'
  return 'ORDINARY' if forecast.valid else 'UNPROVEN_MODEL_BRAKING'


def handoff_acceleration(request, previous, dt, personality, lower, upper, priority='ORDINARY',
                         rise_jerk=None, decel_jerk=None):
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
  rise = ACCEL_RISE.get(p, .4) if rise_jerk is None else rise_jerk
  decel = DECEL_JERK.get(p, .6) if decel_jerk is None else decel_jerk
  if target < previous:
    value = target if priority != 'ORDINARY' else max(target, previous - decel * dt)
  elif previous < 0.:
    release = REGEN_RELEASE_JERK.get(p, .8)
    zero_time = -previous / release
    value = previous + release * dt if dt <= zero_time else rise * (dt - zero_time)
    value = min(target, value)
  else:
    value = min(target, previous + rise * dt)
  return max(lower, min(upper, value))


def capture_positive_acceleration(request, speed_error, filtered_accel):
  """Ease propulsion before the vehicle reaches its target; never add braking."""
  if request > 0.:
    return min(request, max(0., (speed_error - .45 * max(0., filtered_accel)) / .8))
  return request


class NormalCruiseSmoother:
  """Normal open-road capture only; the planner excludes braking constraints.

  Reduced propulsion remains immediate. Only ordinary regen entry and added
  propulsion are slew-limited. Zero net acceleration holds speed; it is not
  assumed to be the drivetrain's zero-torque pedal position.
  """
  def __init__(self):
    self.filtered_accel = None

  def reset(self):
    self.filtered_accel = None

  def update(self, request, previous, dt, personality, speed, cruise, accel, lower, upper):
    if (not all(math.isfinite(v) for v in (request, previous, dt, speed, cruise, accel, lower, upper))
        or not 0 < dt <= .1 or lower > upper):
      self.reset()
      return request
    if self.filtered_accel is None:
      self.filtered_accel = accel
    else:
      self.filtered_accel += dt / (.25 + dt) * (accel - self.filtered_accel)
    error = cruise - speed
    if request > 0.:
      # Anticipate the existing actuator delay rather than pushing through
      # set speed while the MPC's filtered state is still catching up.
      request = capture_positive_acceleration(request, error, self.filtered_accel)
    elif -.05 < request < 0. and abs(error) <= .20 and abs(previous) < .12:
      request = 0.
    p = personality_index(personality)
    # Standard sits between its old rise/entry rates and Relaxed. Relaxed
    # retains its speed-dependent positive rise policy in the actuator.
    rise = .35 if p == 1 else ACCEL_RISE.get(p, .4)
    decel = .50 if p == 1 else DECEL_JERK.get(p, .6)
    history = min(0., previous) if request < 0. else previous
    value = handoff_acceleration(request, history, dt, personality, lower, upper,
                                 rise_jerk=rise, decel_jerk=decel)
    # Do not carry added propulsion after its request has been withdrawn.
    return min(request, value) if previous >= 0. and request >= 0. else value


class ExperimentalCruiseSmoother:
  """One final Experimental handoff with shared positive target capture.

  Model, lead, stop and curve braking retain the existing priority policy.
  A withdrawn positive request cannot survive in planner command history.
  Actuator slew remains downstream. Normal's tuning/state are independent.
  """
  def __init__(self):
    self.filtered_accel = None

  def reset(self):
    self.filtered_accel = None

  def update(self, request, previous, dt, personality, speed, cruise, accel,
             lower, upper, priority='ORDINARY', capture=True):
    if (not all(math.isfinite(v) for v in (request, previous, dt, speed, cruise, accel, lower, upper))
        or not 0 < dt <= .1 or lower > upper):
      self.reset()
      return request
    capture_active = capture and priority == 'ORDINARY'
    if capture_active:
      if self.filtered_accel is None:
        self.filtered_accel = accel
      else:
        self.filtered_accel += dt / (.25 + dt) * (accel - self.filtered_accel)
      request = capture_positive_acceleration(request, cruise - speed, self.filtered_accel)
    else:
      self.reset()
    value = handoff_acceleration(request, previous, dt, personality, lower, upper, priority)
    # Never prolong positive drive after the selected request has eased.
    # Negative entry still uses the existing braking-priority handoff.
    return max(lower, min(upper, min(request, value))) if capture_active and previous >= 0. and request >= 0. else value


class UnifiedCruiseSmoother:
  """Shared Pre-AP final command shaping across modes and speed constraints.

  Selection of leads, stops and model authority remains upstream. This owns
  only ordinary transitions and positive target capture, with one measured
  acceleration history across Chill/Experimental switches.
  """
  def __init__(self):
    self.filtered_accel = None

  def reset(self):
    self.filtered_accel = None

  def update(self, request, previous, dt, personality, speed, cruise, accel,
             lower, upper, priority='ORDINARY', capture=True):
    if (not all(math.isfinite(v) for v in (request, previous, dt, speed, cruise, accel, lower, upper))
        or not 0 < dt <= .1 or lower > upper):
      self.reset()
      return request
    if self.filtered_accel is None:
      self.filtered_accel = accel
    else:
      self.filtered_accel += dt / (.25 + dt) * (accel - self.filtered_accel)
    p = personality_index(personality)
    rise = .35 if p == 1 else ACCEL_RISE.get(p, .4)
    decel = .50 if p == 1 else DECEL_JERK.get(p, .6)
    if capture and priority == 'ORDINARY':
      # Anticipate the speed gained during delay and the remaining ordinary
      # acceleration release: gap = delay*a_measured + a_command²/(2*j).
      # Capture shapes the REQUEST, never an after-the-fact output ceiling.
      gap = max(0., cruise - speed - .45 * max(0., self.filtered_accel))
      capture_cap = math.sqrt(2. * decel * gap)
      if request > 0.:
        request = min(request, capture_cap)
      # Actual overspeed is an explicit constraint, not ordinary capture.
      if speed > cruise + .5:
        request = min(request, 0.)
        priority = 'SPEED_CEILING'
    return handoff_acceleration(
      request, previous, dt, personality, lower, upper, priority,
      rise_jerk=rise, decel_jerk=decel)



def curve_recovery_accel_limit(future_cap, speed, horizon):
  """Limit added speed before the next bend; never create a braking request."""
  if (type(future_cap) not in (int, float) or not all(math.isfinite(v) for v in (future_cap, speed, horizon))
      or horizon <= 0. or speed < 0.):
    return None
  return max(0., (future_cap - speed) / horizon)


class PersonalityAccelerationLimit:
  """Blend comfort-envelope changes; physical turn/brake limits stay separate."""
  def __init__(self):
    self.value = None
    self.personality = None
    self.transition = False

  def update(self, target, personality, dt, reset=False):
    p = personality_index(personality)
    if reset or self.value is None:
      self.value, self.personality, self.transition = target, p, False
      return target
    if p != self.personality:
      self.transition = True
    self.personality = p
    if self.transition:
      rise = .35 if p == 1 else ACCEL_RISE.get(p, .4)
      fall = .50 if p == 1 else DECEL_JERK.get(p, .6)
      self.value = max(self.value - fall * dt, min(self.value + rise * dt, target))
      self.transition = abs(self.value - target) > 1e-9
    else:
      self.value = target
    return self.value
