import json
import time

from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnEntry
from openpilot.selfdrive.controls.lib.turn_context import driver_braking

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
    self.params = Params()

    # PREAP_MANUAL_TURN_CANCEL_LATCH_V1
    # Once the driver takes over an active manual city turn, do not request
    # that turn again until the physical blinker has been canceled.
    self.manual_turn_command_active = False
    self.manual_turn_cancelled = False
    self.manual_entry = ManualTurnEntry()

  def suspend_for_navigation(self, carstate):
    self.manual_entry.reset()
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
             model=None, inputs_valid=False, now=None):
    now = time.monotonic() if now is None else now
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < LANE_CHANGE_SPEED_MIN

    # Restored low-speed manual-blinker turn desires. Independent of longitudinal control.
    if self.manual_turn_poll % 100 == 0:
      raw_param = self.params.get("NAPCityTurns")
      if raw_param is not None:
        self.manual_turns_enabled = (raw_param is True)
      else:
        self.manual_turns_enabled = False
        try:
          with open("/data/nap_turn_settings.json") as stream:
            cfg = json.loads(stream.read(2049))
          self.manual_turns_enabled = cfg.get("manual_blinker_turns") is True
        except (OSError, ValueError, TypeError, AttributeError):
          pass
    self.manual_turn_poll += 1

    if not one_blinker:
      self.manual_turn_cancelled = False
      self.manual_entry.reset()

    continuing_turn = self.manual_entry.started is not None
    turn_speed_max = 35 * CV.MPH_TO_MS if continuing_turn else CITY_TURN_SPEED_MAX
    if self.manual_turns_enabled and -0.1 <= v_ego < turn_speed_max:
      # A lamp is not authorization: only a fresh, classified physical hold can
      # arm. TapModel owns half-stalk requests independently above SPEED_MIN.
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.lane_change_ll_prob = 1.0
      self.keep_pulse_timer = 0.0
      self.prev_one_blinker = one_blinker
      self.desire = log.Desire.none
      direction = manual_turn_direction
      authorized = (inputs_valid and lateral_active and one_blinker
                    and type(direction) is int and direction in (1, 2)
                    and (direction == 1) == bool(carstate.leftBlinker)
                    and not carstate.steeringPressed
                    and not getattr(carstate, 'steeringDisengage', False)
                    and (not driver_braking(carstate) or v_ego <= 0.3) and carstate.canValid
                    and str(carstate.gearShifter) in ('drive', 'low', 'sport', 'eco')
                    and not (carstate.leftBlindspot if direction == 1 else carstate.rightBlindspot))
      self.manual_turn_command_active = self.manual_entry.update(
        model, direction, max(0.0, v_ego), authorized, now,
        driver_assisting=(carstate.steeringPressed and not getattr(carstate, "steeringDisengage", False)
                          and inputs_valid and lateral_active and one_blinker
                          and direction in (1, 2) and not driver_braking(carstate)))
      self.manual_turn_cancelled = self.manual_entry.blocked
      if self.manual_turn_command_active:
        self.desire = log.Desire.turnLeft if direction == 1 else log.Desire.turnRight
      return

    if self.manual_entry.started is not None:
      self.manual_entry.cancel()
    elif not self.manual_entry.blocked:
      self.manual_entry.reset()
    self.manual_turn_command_active = False

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
