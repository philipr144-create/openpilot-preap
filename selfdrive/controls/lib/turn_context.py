"""Shared maneuver meaning and turn eligibility; no actuator commands."""
import math


def classify_route_maneuver(maneuver):
  """Map intent describes a maneuver, never the existence of a drivable lane."""
  if not isinstance(maneuver, dict):
    return 'unknown'
  kind = maneuver.get('type')
  if kind in ('off ramp', 'fork'):
    return 'highway_exit' if kind == 'off ramp' else 'road_fork'
  if kind in ('turn', 'end of road'):
    return 'city_turn'
  if kind == 'arrive':
    return 'arrival'
  if kind in ('lane change', 'laneChangeLeft', 'laneChangeRight'):
    return 'lane_change'
  return 'route_following' if kind in ('depart', 'continue', 'new name', 'merge') else 'unknown'


def route_reference_speed(maneuver):
  """Preserve existing route speeds; exits/lane changes get no guessed cap."""
  kind = classify_route_maneuver(maneuver)
  if kind == 'arrival':
    return 3.0
  if kind != 'city_turn' or maneuver.get('modifier') not in ('left', 'right', 'slight left', 'slight right'):
    return None
  return 5.0 if maneuver['type'] == 'end of road' else 7.0


def maneuver_context(decision, now, model_frame=None):
  """Common lateral/longitudinal vocabulary; labels do not grant authority.

  Preserve source/owner identity. A signal lamp alone is never enough to call
  something a lane change or a driver-authorized city turn. No experimental
  mode condition is introduced here.
  """
  d = decision if isinstance(decision, dict) else {}
  kind = classify_route_maneuver({'type': d.get('maneuver_type')})
  desire = d.get('final_desire')
  source = d.get('source') if isinstance(d.get('source'), str) else 'none'
  owner = d.get('signal_owner') if isinstance(d.get('signal_owner'), str) else 'none'
  if source == 'tap_lane_change' and desire in ('laneChangeLeft', 'laneChangeRight'):
    kind = 'lane_change'
  elif (source in ('none', 'driver_desire_helper') and owner == 'driver'
        and d.get('city_turns_enabled') is True and type(d.get('manual_turn_direction')) is int
        and d.get('manual_turn_direction') in (1, 2)):
    kind = 'city_turn'
  stamp, frame = d.get('received_mono'), d.get('frame_id')
  fresh = (type(stamp) in (int, float) and type(now) in (int, float)
           and math.isfinite(stamp) and math.isfinite(now) and 0 <= now-stamp <= .35
           and d.get('model_evaluated') is True and type(frame) is int
           and (model_frame is None or type(model_frame) is int and abs(model_frame-frame) <= 2))
  reason = d.get('reason') if isinstance(d.get('reason'), str) else ''
  sequence = d.get('exit_sequence') if isinstance(d.get('exit_sequence'), dict) else {}
  sequence_phase = sequence.get('phase')
  exit_entry_selected = (kind in ('highway_exit', 'road_fork') and fresh
                         and source == 'navigation' and desire in ('keepLeft', 'keepRight')
                         and sequence.get('desire') == desire and sequence_phase in ('ENTER', 'COMMIT'))
  distance = d.get('distance_m')
  distance = distance if type(distance) in (int, float) and math.isfinite(distance) else None
  if not fresh:
    phase = 'UNAVAILABLE'
  elif d.get('driver_steering_override') is True:
    phase = 'DRIVER_OVERRIDE'
  elif d.get('manual_turn_blocked') is True or 'cancelled' in reason.lower():
    phase = 'ABORTED'
  elif 'already passed' in reason.lower():
    phase = 'COMPLETE'
  elif kind in ('highway_exit', 'road_fork'):
    phase = (sequence_phase if sequence_phase in ('ANNOUNCE','WAIT_ENTRY','ENTER','COMMIT',
             'DRIVER_OVERRIDE','UNAVAILABLE','ABORTED','COMPLETE') else
             'ANNOUNCE' if d.get('synthetic_indicator_request') in ('left', 'right') else
             'APPROACH' if distance is not None and distance > 300 else 'WAIT_ENTRY')
  elif kind == 'lane_change':
    phase = {'waiting': 'WAIT_ENTRY', 'pending': 'WAIT_ENTRY', 'starting': 'ENTER',
             'active': 'COMMIT', 'finishing': 'EXIT', 'complete': 'COMPLETE'}.get(str(d.get('tap_status')), 'WAIT_ENTRY')
  elif kind == 'city_turn':
    phase = {'ARMED': 'APPROACH', 'WAIT': 'WAIT_ENTRY', 'ENTER': 'ENTER',
             'TURN': 'COMMIT', 'EXIT': 'EXIT'}.get(str(d.get('manual_turn_phase')),
             'GUIDANCE' if desire in ('turnLeft', 'turnRight') else 'APPROACH')
  else:
    phase = 'FOLLOW'
  return {'version': 1, 'kind': kind, 'phase': phase, 'fresh': fresh,
          'source': source[:80], 'signal_owner': owner[:40],
          'route_id': d['route_id'][:96] if isinstance(d.get('route_id'), str) else None,
          'maneuver_id': d['maneuver_id'][:96] if isinstance(d.get('maneuver_id'), str) else None, 'distance_m': distance,
          'lateral_policy': ('qualified_independent_exit_corridor' if exit_entry_selected else
                             'announce_only_no_verified_exit_corridor' if kind in ('highway_exit', 'road_fork')
                             else 'existing_driver_lane_change_gates' if kind == 'lane_change'
                             else 'existing_blinker_and_turn_geometry' if kind == 'city_turn' else 'existing_lane_following'),
          'longitudinal_policy': ('existing_turn_speed_envelope' if kind == 'city_turn'
                                  else 'existing_lead_stop_and_road_curve_plan'),
          'geometry_authority_added': False, 'experimental_mode_required': False}


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
