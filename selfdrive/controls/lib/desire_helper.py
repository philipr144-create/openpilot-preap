import json
import time
import math

from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnEntry
from openpilot.selfdrive.controls.lib.turn_context import driver_braking, city_turns_enabled

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
CITY_TURN_SPEED_MAX = 25.0 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.manual_turns_enabled = False
    self.manual_turn_poll = 0
    self.last_city_toggle_read = -math.inf
    self.params = Params()

    # PREAP_MANUAL_TURN_CANCEL_LATCH_V1
    # Once the driver takes over an active manual city turn, do not request
    # that turn again until the physical blinker has been canceled.
    self.manual_turn_command_active = False
    self.manual_turn_cancelled = False
    self.manual_entry = ManualTurnEntry()
    self.manual_turn_gates = {}
    self.manual_turn_direction = 0
    self.accepted_request_controller = None

  def refresh_city_turns(self, now=None):
    now = time.monotonic() if now is None else now
    if now-self.last_city_toggle_read >= .1 or now < self.last_city_toggle_read:
      self.manual_turns_enabled = city_turns_enabled(self.params)
      self.last_city_toggle_read = now
    return self.manual_turns_enabled

  def suspend_for_navigation(self, carstate):
    self.manual_entry.reset()
    self.manual_turn_direction = 0
    self.accepted_request_controller = None
    self.manual_turn_gates = {'owner_suppression': True}
    self.manual_turn_command_active = False
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = bool(carstate.leftBlinker or carstate.rightBlinker)
    self.desire = log.Desire.none

  def update_tap(self, carstate, lane_change_prob, direction):
    """One authorized maneuver; completion always returns off, never preLaneChange."""
    self.prev_one_blinker = bool(carstate.leftBlinker or carstate.rightBlinker)
    self.keep_pulse_timer = 0.0
    self.lane_change_direction = LaneChangeDirection.left if direction == 1 else LaneChangeDirection.right
    if self.lane_change_state == LaneChangeState.off:
      self.lane_change_state = LaneChangeState.preLaneChange
      self.lane_change_ll_prob = 1.0
    if self.lane_change_state == LaneChangeState.preLaneChange:
      blindspot = carstate.leftBlindspot if direction == 1 else carstate.rightBlindspot
      if not blindspot:
        self.lane_change_state = LaneChangeState.laneChangeStarting
    elif self.lane_change_state == LaneChangeState.laneChangeStarting:
      self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)
      if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
        self.lane_change_state = LaneChangeState.laneChangeFinishing
    elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
      self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)
      if self.lane_change_ll_prob > 0.99:
        self.suspend_for_navigation(carstate)
        return 'complete'
    self.lane_change_timer += DT_MDL
    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]
    return {LaneChangeState.preLaneChange: 'waiting', LaneChangeState.laneChangeStarting: 'starting',
            LaneChangeState.laneChangeFinishing: 'finishing'}[self.lane_change_state]

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob, *, manual_turn_direction=0,
             model=None, inputs_valid=False, now=None, actual_curvature=None,
             manual_turn_request_valid=False, physical_direction=0, manual_turn_request_controller=None):
    now = time.monotonic() if now is None else now
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    self.manual_turn_direction = 0
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    self.refresh_city_turns(now)
    self.manual_turn_poll += 1

    if not one_blinker:
      self.manual_turn_cancelled = False
      self.manual_entry.reset()
      self.accepted_request_controller = None

    continuing_turn = self.manual_entry.started is not None
    turn_speed_max = 40 * CV.MPH_TO_MS
    if self.manual_turns_enabled and -0.1 <= v_ego <= turn_speed_max:
      # Fresh classified physical intent is required to start. An already
      # accepted turn may retain its direction with the same signal and live
      # request producer; it still passes every independent cancellation gate.
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.lane_change_ll_prob = 1.0
      self.keep_pulse_timer = 0.0
      self.prev_one_blinker = one_blinker
      self.desire = log.Desire.none
      direction = manual_turn_direction
      # The upstream classifier may suppress turn classification above 25 mph.
      # A fresh physical hold may gather approach evidence, without turn desire.
      if (v_ego > CITY_TURN_SPEED_MAX and manual_turn_request_valid is True
          and type(physical_direction) is int and physical_direction in (1, 2)
          and type(manual_turn_request_controller) is str and manual_turn_request_controller):
        direction = physical_direction
      classified = type(direction) is int and direction in (1, 2)
      accepted = self.manual_entry.direction
      controller_changed = ((self.manual_entry.started is not None or self.manual_entry.stop_armed or self.manual_entry.turn_locked) and self.accepted_request_controller is not None
                            and manual_turn_request_controller != self.accepted_request_controller)
      if controller_changed:
        self.manual_entry.cancel('REQUEST_CONTROLLER_CHANGED')
      turn_in_progress = (self.manual_entry.started is not None and not self.manual_entry.blocked)
      retained = (type(direction) is int and direction == 0
                  and (self.manual_entry.started is not None or self.manual_entry.stop_armed) and not self.manual_entry.blocked
                  and accepted in (1, 2) and (one_blinker or turn_in_progress)
                  and (((accepted == 1) == bool(carstate.leftBlinker)) or turn_in_progress)
                  and (manual_turn_request_valid is True or turn_in_progress)
                  and (not controller_changed or turn_in_progress)
                  and type(physical_direction) is int and physical_direction in (0, accepted))
      if retained:
        direction = accepted
      self.manual_turn_direction = direction if classified or retained else 0
      self.manual_turn_gates = {
        'city_turns_enabled': bool(self.manual_turns_enabled), 'inputs_valid': bool(inputs_valid),
        'lateral_active': bool(lateral_active), 'single_blinker': bool(one_blinker or turn_in_progress),
        'direction_authorized': classified or retained,
        'request_fresh_for_retention': classified or manual_turn_request_valid is True or turn_in_progress,
        'request_controller_unchanged': not controller_changed or turn_in_progress,
        'matching_blinker': bool(((direction == 1) == bool(carstate.leftBlinker)) or turn_in_progress),
        'no_driver_override': not carstate.steeringPressed and not getattr(carstate, 'steeringDisengage', False),
        'can_valid': bool(getattr(carstate, 'canValid', False)),
        'drive_gear': str(carstate.gearShifter) in ('drive', 'low', 'sport', 'eco'),
        'blindspot_clear': not (carstate.leftBlindspot if direction == 1 else carstate.rightBlindspot),
      }
      authorized = all(self.manual_turn_gates.values())
      stationary_wait = (not lateral_active and 0 <= v_ego <= .3
                         and not self.manual_entry.physical_turn_started
                         and all(v for k, v in self.manual_turn_gates.items() if k != 'lateral_active'))
      self.manual_turn_command_active = self.manual_entry.update(
        model, direction, max(0.0, v_ego), authorized, now,
        transient_unavailable=not inputs_valid, actual_curvature=actual_curvature, stationary_wait=stationary_wait,
        driver_assisting=(carstate.steeringPressed and not getattr(carstate, "steeringDisengage", False)
                          and inputs_valid and lateral_active and one_blinker
                          and direction in (1, 2)))
      if (self.manual_entry.started is not None or self.manual_entry.stop_armed or self.manual_entry.turn_locked) and classified:
        self.accepted_request_controller = (manual_turn_request_controller
                                            if type(manual_turn_request_controller) is str and manual_turn_request_controller else None)
      self.manual_entry.authorization_trace = {
        'raw_classified_direction': manual_turn_direction if type(manual_turn_direction) is int else None,
        'physical_direction': physical_direction if type(physical_direction) is int else None,
        'effective_direction': self.manual_turn_direction, 'direction_retained': retained,
        'request_valid': manual_turn_request_valid is True,
        'request_controller': manual_turn_request_controller if type(manual_turn_request_controller) is str else None,
        'reason': ('REQUEST_CONTROLLER_CHANGED' if controller_changed else 'ACCEPTED_TURN_SAME_SIGNAL'
                   if retained else 'FRESH_CLASSIFICATION' if classified else 'NO_CLASSIFIED_DIRECTION'),
      }
      self.manual_turn_cancelled = self.manual_entry.blocked
      if self.manual_turn_command_active:
        self.desire = log.Desire.turnLeft if direction == 1 else log.Desire.turnRight
      return

    self.manual_turn_gates = {'city_turns_enabled': bool(self.manual_turns_enabled),
                              'within_city_turn_speed': -.1 <= v_ego < turn_speed_max}
    if self.manual_entry.started is not None:
      self.manual_entry.cancel('CITY_TURN_DISABLED_OR_SPEED')
    elif not self.manual_entry.blocked:
      self.manual_entry.reset()
    self.manual_turn_command_active = False
    self.accepted_request_controller = None

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        # Initialize lane change direction to prevent UI alert flicker
        self.lane_change_direction = self.get_lane_change_direction(carstate)

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Update lane change direction
        self.lane_change_direction = self.get_lane_change_direction(carstate)

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif torque_applied and not blindspot_detected:
          self.lane_change_state = LaneChangeState.laneChangeStarting

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # fade out over .5s
        self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

        # 98% certainty
        if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
          self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            self.lane_change_state = LaneChangeState.preLaneChange
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight):
        self.desire = log.Desire.none
