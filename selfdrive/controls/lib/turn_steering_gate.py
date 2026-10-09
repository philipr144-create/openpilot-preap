"""Low-speed turn permission; ordinary road following remains available."""
import math

TURN_SPEED_MAX = 25 * 0.44704
TIGHT_TURN_CURVATURE = 0.02  # radius below 50 m requires a turn indicator


def matching_signal(cs, direction):
  return (bool(cs.leftBlinker) != bool(cs.rightBlinker)
          and direction in (1, 2) and (direction == 1) == bool(cs.leftBlinker))


class TurnSteeringGate:
  def __init__(self):
    self.driver_cancelled = False

  def allowed(self, cs, model, requested_curvature=None):
    one = bool(cs.leftBlinker) != bool(cs.rightBlinker)
    if not cs.leftBlinker and not cs.rightBlinker:
      self.driver_cancelled = False
    if cs.vEgo >= TURN_SPEED_MAX:
      return True
    # HSO owns temporary yielding and quiet handback. Do not turn a brief
    # torque event into a separate persistent model-path cancellation.
    try:
      curvature = float(model.action.desiredCurvature if requested_curvature is None else requested_curvature)
      if not math.isfinite(curvature):
        return False
      probabilities = list(model.meta.desireState)
      turn_left = len(probabilities) > 2 and probabilities[1] > 0.5
      turn_right = len(probabilities) > 2 and probabilities[2] > 0.5
      tight = abs(curvature) > TIGHT_TURN_CURVATURE
      if not tight and not turn_left and not turn_right:
        return True
      if self.driver_cancelled or not one:
        return False
      if turn_left and not matching_signal(cs, 1):
        return False
      if turn_right and not matching_signal(cs, 2):
        return False
      return not tight or matching_signal(cs, 1 if curvature < 0.0 else 2)
    except (AttributeError, TypeError, ValueError):
      return False
