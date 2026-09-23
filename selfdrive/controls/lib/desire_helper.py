import json

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.common.params import Params

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
CITY_TURN_SPEED_MAX = 25 * CV.MPH_TO_MS
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

  def suspend_for_navigation(self, carstate):
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
    
    # --- TWO-STAGE TURN / POCKET LOGIC ---
    if v_ego < 6.7:
      if carstate.leftBlinker:
        self.desire = log.Desire.turnLeft
      elif carstate.rightBlinker:
        self.desire = log.Desire.turnRight
    elif v_ego < 11.1:
      if carstate.leftBlinker:
        self.desire = log.Desire.keepLeft
      elif carstate.rightBlinker:
        self.desire = log.Desire.keepRight
    # -------------------------------------
    return {LaneChangeState.preLaneChange: 'waiting', LaneChangeState.laneChangeStarting: 'starting',
            LaneChangeState.laneChangeFinishing: 'finishing'}[self.lane_change_state]

  @staticmethod
  def get_lane_change_direction(CS):
    return LaneChangeDirection.left if CS.leftBlinker else LaneChangeDirection.right

  def update(self, carstate, lateral_active, lane_change_prob):
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
      self.manual_turn_command_active = False
      self.manual_turn_cancelled = False

    if not self.manual_turns_enabled:
      # Clear out any active manual turn state if disabled mid-turn
      if self.manual_turn_command_active:
        self.desire = log.Desire.none
      self.manual_turn_command_active = False
      self.manual_turn_cancelled = False
    elif 0 <= v_ego < CITY_TURN_SPEED_MAX:
      # Do not carry an in-progress lane-change state into a low-speed turn.
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
      self.lane_change_timer = 0.0
      self.lane_change_ll_prob = 1.0
      self.keep_pulse_timer = 0.0
      self.prev_one_blinker = one_blinker
      self.desire = log.Desire.none

      # PREAP_MANUAL_TURN_CANCEL_LATCH_V1
      # Only latch a cancellation after a turn command was actually active.
      # Releasing the wheel cannot restart it while the same blinker remains on.
      if carstate.steeringPressed and self.manual_turn_command_active:
        self.manual_turn_cancelled = True
        self.manual_turn_command_active = False

      if (
          lateral_active
          and one_blinker
          and not carstate.steeringPressed
          and not self.manual_turn_cancelled
      ):
        if carstate.leftBlinker and not carstate.leftBlindspot:
          self.desire = log.Desire.turnLeft
          self.manual_turn_command_active = True
        elif carstate.rightBlinker and not carstate.rightBlindspot:
          self.desire = log.Desire.turnRight
          self.manual_turn_command_active = True
      return

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
    
    # --- TWO-STAGE TURN / POCKET LOGIC ---
    if v_ego < 6.7:
      if carstate.leftBlinker:
        self.desire = log.Desire.turnLeft
      elif carstate.rightBlinker:
        self.desire = log.Desire.turnRight
    elif v_ego < 11.1:
      if carstate.leftBlinker:
        self.desire = log.Desire.keepLeft
      elif carstate.rightBlinker:
        self.desire = log.Desire.keepRight
    # -------------------------------------

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none
