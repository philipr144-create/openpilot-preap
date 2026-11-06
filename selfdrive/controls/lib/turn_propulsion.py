"""Offline candidate: bounded turn speed feedback, never an actuator writer.

The acceleration circle is a comfort envelope, not an estimate of tire grip.
All constants require closed-loop validation before deployment.
"""
import math
import json
import os
from pathlib import Path
import tempfile

class TurnPropulsionPlan:
  MAX_ACCEL = .35
  TOTAL_COMFORT_ACCEL = 1.8
  POSITIVE_JERK = .25
  KP = .5
  KI = .1
  WEAK_COAST_MIN = -.15

  def __init__(self):
    self.next_publish = 0.
    try:
      self.boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    except OSError:
      self.boot_id = None
    self.reset()

  def reset(self, reason='OFF'):
    self.integral = self.accel = 0.
    self.last_time = None
    self.upper = None
    self.diagnostic = {'reason': reason, 'supporting': False, 'upper_accel': None}

  def update(self, base, *, speed, target, actual_curvature, mpc_accel, model_accel,
             upper_accel, now, ready, blocked, profile_owned=False,
             turn_assist_enabled=False, steer_angle_deg=0.0):
    values=(base,speed,target,actual_curvature,mpc_accel,model_accel,upper_accel,now)
    if not ready or blocked or not all(type(v) in (int,float) and math.isfinite(v) for v in values):
      self.reset('BLOCKED_OR_UNAVAILABLE');return base

    abs_steer = abs(steer_angle_deg) if (type(steer_angle_deg) in (int, float) and math.isfinite(steer_angle_deg)) else 0.0

    # ── Stop-to-Turn Launch Assist (behind UI toggle) ──
    if turn_assist_enabled and speed < 2.5 and (abs_steer >= 25.0 or actual_curvature >= 0.005):
      dt = .05 if self.last_time is None else now - self.last_time
      if not 0 < dt <= .15:
        self.reset('STALE_TIMESTEP'); return base
      self.last_time = now
      steer_intensity = min(1.0, max(0.0, (abs_steer - 25.0) / 75.0))
      speed_fade = max(0.0, min(1.0, (2.5 - max(0.0, speed)) / 2.5))
      launch_floor = (0.18 + 0.17 * steer_intensity) * speed_fade
      launch_cap = min(0.40, upper_accel)
      self.upper = launch_cap
      request = min(launch_cap, max(base, launch_floor))
      self.diagnostic = {
        'reason': 'LAUNCH_TURN_ASSIST',
        'supporting': request > max(0., base),
        'target_mps': target,
        'speed_mps': speed,
        'steer_angle_deg': abs_steer,
        'launch_floor': launch_floor,
        'upper_accel': launch_cap,
        'request_accel': request,
        'base_accel': base,
      }
      return request

    if not turn_assist_enabled:
      if (not .3 <= speed <= 15.65 or target <= 0 or actual_curvature < 0
          or base < self.WEAK_COAST_MIN or model_accel < self.WEAK_COAST_MIN or mpc_accel <= 0):
        self.reset('BRAKING_OR_NO_MPC_PERMISSION');return base
    else:
      # Turn assist active: do not drop out on mild model hesitation, but respect true braking (< -0.50 m/s^2)
      if (not .3 <= speed <= 15.65 or actual_curvature < 0 or base < -0.50):
        self.reset('BRAKING_OR_NO_MPC_PERMISSION');return base
      if not (type(target) in (int, float) and math.isfinite(target) and target > 0):
        target = min(max(3.0, speed + 1.5), 6.7)
    dt=.05 if self.last_time is None else now-self.last_time
    if not 0 < dt <= .15:
      self.reset('STALE_TIMESTEP');return base
    self.last_time=now
    lateral=speed*speed*actual_curvature
    remaining=math.sqrt(max(0.,self.TOTAL_COMFORT_ACCEL**2-lateral**2))
    error=target-speed
    if not turn_assist_enabled:
      upper=max(0.,min(self.MAX_ACCEL,upper_accel,mpc_accel,remaining,max(0.,error)/dt))
    else:
      upper=max(0.,min(self.MAX_ACCEL,upper_accel,remaining,max(0.,error)/dt))
    self.upper=upper
    if turn_assist_enabled and (abs_steer >= 20.0 or actual_curvature >= 0.005 or profile_owned):
      # Turn assist enabled: provide a comfort-bounded maintenance floor to prevent bogging down in any turn
      curv_intensity = min(1.0, max(0.0, (actual_curvature - 0.005) / 0.020))
      steer_intensity = min(1.0, max(0.0, (abs_steer - 20.0) / 60.0))
      intensity = max(curv_intensity, steer_intensity)
      maint_floor = (0.10 + 0.18 * intensity) * min(1.0, speed / 2.5)
      maint_floor = min(maint_floor, remaining * 0.70, upper)
      request = min(upper, max(base, maint_floor))
      self.diagnostic = {
        'reason': 'TURN_SPEED_MAINTENANCE',
        'supporting': request > max(0., base),
        'target_mps': target,
        'speed_error_mps': error if math.isfinite(error) else 0.0,
        'actual_curvature': actual_curvature,
        'lateral_accel': lateral,
        'comfort_remaining': remaining,
        'upper_accel': upper,
        'maint_floor': maint_floor,
        'request_accel': request,
        'base_accel': base,
      }
      return request
    if profile_owned:
      if not turn_assist_enabled:
        # Stock behavior: apply ceiling only, without a second PI loop
        self.integral=self.accel=0.
        self.diagnostic={'reason':'TURN_PROFILE_CEILING','supporting':False,
          'target_mps':target,'upper_accel':upper,'base_accel':base,
          'request_accel':min(base,upper)}
        return min(base,upper)
    # No integral accumulation against a binding ceiling or above target.
    if error > 0 and upper > 0:
      self.integral=min(1.,self.integral+error*dt)
    else:
      self.integral=0.
    desired=min(upper,max(0.,self.KP*error+self.KI*self.integral))
    self.accel=min(desired,self.accel+self.POSITIVE_JERK*dt)
    request=min(upper,max(base,self.accel))
    self.diagnostic={'reason':'TURN_SPEED_FEEDBACK','supporting':request>max(0.,base),
      'target_mps':target,'speed_error_mps':error,'actual_curvature':actual_curvature,
      'lateral_accel':lateral,'comfort_remaining':remaining,'upper_accel':upper,
      'request_accel':request,'base_accel':base,'mpc_accel':mpc_accel,'model_accel':model_accel}
    return request

  def publish(self, now, final_accel, model_frame, city_enabled, long_active,
              path='/dev/shm/nap_turn_longitudinal.json'):
    if now < self.next_publish:
      return
    self.next_publish = now+.2
    name = None
    try:
      point = dict(self.diagnostic)
      point.update(version=1, implementation='unified-city-turn-v1', producer_mono=now,
                   boot_id=self.boot_id, final_accel=float(final_accel), model_input_frame=int(model_frame),
                   city_turns_enabled=bool(city_enabled), long_active=bool(long_active))
      raw = json.dumps(point, allow_nan=False, separators=(',', ':'))
      if len(raw.encode()) > 8192:
        return
      destination = Path(path)
      with tempfile.NamedTemporaryFile(mode='w',dir=destination.parent,prefix='.nap-turn-long-',delete=False) as stream:
        name = stream.name
        stream.write(raw)
      os.replace(name,destination)
    except (OSError,ValueError,TypeError):
      pass
    finally:
      if name:
        try:os.unlink(name)
        except OSError:pass
