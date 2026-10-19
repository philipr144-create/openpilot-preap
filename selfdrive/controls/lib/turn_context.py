"""Shared turn eligibility; no steering or speed authority."""
import math


def driver_braking(cs):
  return bool(cs.brakePressed or getattr(cs, 'brake', 0.0) > 0.0)


def paired_decision(decision, model, now, require_pair=True):
  if not isinstance(decision, dict):
    return False
  stamp, frame = decision.get('received_mono'), decision.get('frame_id')
  model_frame = getattr(model, 'frameId', None)
  return (type(stamp) in (int, float) and math.isfinite(stamp)
          and 0 <= now - stamp <= .35 and type(frame) is int
          and type(model_frame) is int and (not require_pair or abs(frame - model_frame) <= 2)
          and decision.get('model_evaluated') is True)


def manual_turn_intent(cs, decision, model, now, require_pair=True):
  if not paired_decision(decision, model, now, require_pair=require_pair):
    return False
  direction = decision.get('manual_turn_direction')
  speed_max = (35 if decision.get("manual_turn_continuing") is True else 25) * .44704
  return (math.isfinite(cs.vEgo) and -.1 <= cs.vEgo < speed_max
          and type(direction) is int and direction in (1, 2)
          and bool(cs.leftBlinker) != bool(cs.rightBlinker)
          and (direction == 1) == bool(cs.leftBlinker)
          and decision.get('city_turns_enabled') is True
          and decision.get('signal_owner') == 'driver'
          and decision.get('tap_signal_active') is False
          and decision.get('synthetic_indicator_request') == 'none'
          and decision.get('manual_turn_phase') in (None, 'ARMED', 'TURN', 'EXIT')
          and decision.get('manual_turn_blocked') is not True
          and decision.get('source') in ('none', 'driver_desire_helper')
          and decision.get('final_desire') in ('none', 'turnLeft' if direction == 1 else 'turnRight')
          and not cs.steeringPressed and not getattr(cs, 'steeringDisengage', False)
          and cs.canValid
          and not (cs.leftBlindspot if direction == 1 else cs.rightBlindspot)
          and str(cs.gearShifter) in ('drive', 'low', 'sport', 'eco'))
