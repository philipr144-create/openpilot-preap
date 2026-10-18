import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# Acceleration-domain motor compensation, independent of personality.
MAX_POSITIVE_TRACKING_TRIM = 0.60
STOP_RELEASE_CONFIRM_TIME = 0.30
STOP_RELEASE_MIN_ACCEL = 0.0
STOP_HOLD_SPEED = 0.50


class StopRelease:
  """Debounce release only near standstill; acquisition is immediate.

  A single clear model frame must not restart the stop ramp. Moving braking
  requests retain their existing authority and can clear without this dwell.
  Driver override/disengagement clears history instead of retaining a hold.
  """
  def __init__(self):
    self.reset()

  def reset(self):
    self.holding = False
    self.clear_time = 0.0

  def update(self, active, speed, should_stop, accel, brake, cruise_standstill, dt):
    if not active:
      self.reset()
      return bool(should_stop)
    if should_stop:
      self.holding = True
      self.clear_time = 0.0
      return True
    if not self.holding:
      return False
    if speed > STOP_HOLD_SPEED:
      self.reset()
      return False
    release = (np.isfinite(accel) and accel >= STOP_RELEASE_MIN_ACCEL
               and not brake and not cruise_standstill)
    self.clear_time = self.clear_time + dt if release else 0.0
    if self.clear_time >= STOP_RELEASE_CONFIRM_TIME - 1e-9:
      self.reset()
      return False
    return True


def update_preap_feedback(pid, error, speed, target, *, freeze_integrator=False):
  """Acceleration feedback with separate correction and total-output bounds.

  Conditional integration prevents windup; output limits never overwrite the
  stored correction. The generic PID used by lateral/other cars is untouched.
  """
  pid.speed = speed
  pid.p = pid.k_p * float(error)
  pid.d = 0.0
  pid.f = target
  if not freeze_integrator:
    candidate_i = pid.i + pid.k_i * pid.i_dt * error
    candidate_trim = pid.p + candidate_i
    candidate_output = target + candidate_trim
    saturated_high = candidate_trim > MAX_POSITIVE_TRACKING_TRIM or candidate_output > pid.pos_limit
    saturated_low = candidate_output < pid.neg_limit
    if not ((error > 0. and saturated_high) or (error < 0. and saturated_low)):
      pid.i = candidate_i
  trim = min(MAX_POSITIVE_TRACKING_TRIM, pid.p + pid.i)
  pid.control = float(np.clip(target + trim, pid.neg_limit, pid.pos_limit))
  return pid.control


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.capture_accel = None
    self.stop_release = StopRelease()
    self.releasing_stop = False

  def reset(self):
    self.pid.reset()
    self.capture_accel = None
    self.stop_release.reset()
    self.releasing_stop = False

  def update(self, active, CS, a_target, should_stop, accel_limits, cruise_capture_speed=None):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]
    is_preap = self.CP.carFingerprint == "TESLA_MODEL_S_PREAP"
    previous_state = self.long_control_state
    if is_preap:
      # Keep PID output bounds in output units. A negative a_target must not
      # become an upper bound on the integral correction itself.
      should_stop = self.stop_release.update(
        active, CS.vEgo, should_stop, a_target, CS.brakePressed,
        CS.cruiseState.standstill, DT_CTRL)

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.pid.reset()
      self.capture_accel = None
      self.releasing_stop = False

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      if is_preap and previous_state == LongCtrlState.stopping:
        self.releasing_stop = True
      error = a_target - CS.aEgo
      freeze_capture = False
      if (self.CP.carFingerprint == "TESLA_MODEL_S_PREAP"
          and cruise_capture_speed is not None
          and np.isfinite(cruise_capture_speed) and np.isfinite(CS.aEgo)
          and np.isfinite(CS.vEgo) and CS.vEgo >= 5.):
        if self.capture_accel is None:
          self.capture_accel = CS.aEgo
        else:
          self.capture_accel += DT_CTRL / (.25 + DT_CTRL) * (CS.aEgo - self.capture_accel)
        # Remove only surplus positive trim during actual target capture.
        # A grade requiring torque at steady speed retains its integral trim.
        if (0. <= a_target <= .25 and self.pid.i > 0.
            and CS.vEgo >= cruise_capture_speed - .30
            and CS.vEgo + .45 * max(0., self.capture_accel) >= cruise_capture_speed - .10
            and self.capture_accel > a_target + .04):
          self.pid.i = max(0., self.pid.i - .20 * DT_CTRL)
          freeze_capture = True
      else:
        self.capture_accel = None

      # PREAP_OUTER_PID_ANTI_WINDUP_V2
      # Rapidly unwind only when the requested acceleration also opposes
      # the stored correction. Brief aEgo fluctuations during a steady
      # request must not repeatedly erase the trim needed to track it.
      # Normal PID integration still corrects all acceleration errors.
      # Strong braking retains faster unwind; stopping retains its own ramp.
      if (
          self.CP.carFingerprint == "TESLA_MODEL_S_PREAP"
          and abs(error) >= 0.04
          and self.pid.i * error < 0.0
          and self.pid.i * a_target < 0.0
      ):
        unwind_rate = 0.60

        if a_target <= -0.75:
          unwind_rate = 2.00

        unwind_step = unwind_rate * DT_CTRL

        if self.pid.i > 0.0:
          self.pid.i = max(
            0.0,
            self.pid.i - unwind_step,
          )
        else:
          self.pid.i = min(
            0.0,
            self.pid.i + unwind_step,
          )

      if is_preap:
        output_accel = update_preap_feedback(
          self.pid, error, CS.vEgo, a_target,
          freeze_integrator=freeze_capture or self.releasing_stop)
      else:
        output_accel = self.pid.update(error, speed=CS.vEgo, feedforward=a_target,
                                       freeze_integrator=freeze_capture)
      if is_preap:
        if self.releasing_stop:
          # Carry the stop command into release at the configured physical
          # ramp rate, without seeding the feedback integrator with braking.
          release_ceiling = self.last_output_accel + self.CP.stoppingDecelRate * DT_CTRL
          if output_accel > release_ceiling:
            output_accel = release_ceiling
          else:
            self.releasing_stop = False
        self.pid.control = output_accel

    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
