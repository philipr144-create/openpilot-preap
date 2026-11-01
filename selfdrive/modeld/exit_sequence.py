"""Offline-testable exit sequencing. No I/O, settings, publishers, or AI calls.

This module consumes corridor evidence; it is not a corridor detector. The
current modeld integration supplies none and therefore cannot authorize entry.
Only a separately validated, in-process scene provider may supply evidence.
Route JSON and the navigation-conditioned policy path are not such providers.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class CorridorEvidence:
  route_id: str
  maneuver_id: str
  direction: int
  model_frame: int
  sensor_mono: float
  provider: str
  geometry_source: str
  entry_open: bool
  branch_matches_route: bool
  connected_drivable_path: bool
  vehicle_envelope_clear: bool
  policy_proposal_contained: bool
  speed_plan_compatible: bool


@dataclass(frozen=True)
class ExitDecision:
  phase: str
  desire: str
  reason: str
  signal_lead_s: float = 0.
  evidence_frames: int = 0

  def diagnostic(self):
    return {'version': 1, 'phase': self.phase, 'desire': self.desire,
            'reason': self.reason, 'signal_lead_s': round(self.signal_lead_s, 3),
            'evidence_frames': self.evidence_frames}


def finite(value):
  return type(value) in (int, float) and math.isfinite(value)


class ExitSequence:
  # Candidate timing parameters, not road-validated or legal guarantees.
  # No entry is possible without a separate, validated scene provider.
  SIGNAL_LEAD_S = 3.0
  QUALIFY_S = .25
  QUALIFY_FRAMES = 5
  MAX_TICK_GAP_S = .15
  MAX_EVIDENCE_AGE_S = .15
  MAX_FRAME_GAP = 2

  def __init__(self):
    self.reset()

  def reset(self):
    self.key = None
    self.direction = 0
    self.last_now = None
    self.signal_since = None
    self.evidence_since = None
    self.evidence_frames = 0
    self.last_evidence_frame = None
    self.last_sensor_mono = None
    self.entered = False
    self.terminal = False
    self.terminal_phase = 'ABORTED'

  def _clear_evidence(self):
    self.evidence_since = None
    self.evidence_frames = 0
    self.last_evidence_frame = None
    self.last_sensor_mono = None

  def withdraw(self):
    """A skipped/invalid adapter tick cannot preserve acquired permission."""
    self.signal_since = None
    self._clear_evidence()
    if self.entered:
      self.terminal = True

  def _decision(self, phase, reason, now, desire='none'):
    lead = max(0., now-self.signal_since) if self.signal_since is not None and finite(now) else 0.
    return ExitDecision(phase, desire, reason, lead, self.evidence_frames)

  @staticmethod
  def _valid_evidence(evidence, key, direction, now, model_frame):
    if type(evidence) is not CorridorEvidence:
      return False
    e = evidence
    return (e.route_id == key[0] and e.maneuver_id == key[1]
            and type(e.direction) is int and e.direction == direction
            and type(e.model_frame) is int and e.model_frame >= 0
            and type(model_frame) is int and model_frame >= 0
            and 0 <= model_frame-e.model_frame <= ExitSequence.MAX_FRAME_GAP
            and finite(e.sensor_mono) and 0 <= now-e.sensor_mono <= ExitSequence.MAX_EVIDENCE_AGE_S
            and isinstance(e.provider, str) and 0 < len(e.provider) <= 80
            and e.geometry_source == 'independent_scene_corridor'
            and all(v is True for v in (e.entry_open, e.branch_matches_route,
                e.connected_drivable_path, e.vehicle_envelope_clear,
                e.policy_proposal_contained, e.speed_plan_compatible)))

  def update(self, *, route_id, maneuver_id, direction, now, distance_m,
             route_fresh, inputs_valid, lateral_active, signal_announced,
             vehicle_signal_matches, driver_override=False, braking=False,
             opposite_signal=False, model_frame=None, evidence=None):
    if (not finite(now) or not finite(distance_m)
        or not isinstance(route_id, str) or not route_id
        or not isinstance(maneuver_id, str) or not maneuver_id
        or type(direction) is not int or direction not in (1, 2)
        or any(type(value) is not bool for value in (route_fresh,inputs_valid,lateral_active,
            signal_announced,vehicle_signal_matches,driver_override,braking,opposite_signal))):
      self.reset()
      return ExitDecision('UNAVAILABLE', 'none', 'Invalid exit identity or clock')
    key = (route_id, maneuver_id)
    if self.key != key or self.direction != direction:
      self.reset()
      self.key, self.direction = key, direction
    gap = now-self.last_now if self.last_now is not None else 0.
    self.last_now = now
    if gap < 0 or gap > self.MAX_TICK_GAP_S:
      self.signal_since = None
      self._clear_evidence()
      if self.entered:
        self.terminal = True
    if self.terminal:
      return self._decision(self.terminal_phase, 'Exit needs a new route step after completion or evidence loss', now)
    if distance_m <= 0:
      self.terminal = True
      self.terminal_phase = 'COMPLETE'
      self.signal_since = None
      self._clear_evidence()
      return self._decision('COMPLETE', 'Mapped maneuver reached; do not start or restart entry', now)
    if opposite_signal is True:
      self.terminal = True
      self.signal_since = None
      self._clear_evidence()
      return self._decision('ABORTED', 'Opposite driver signal cancels this exit step', now)
    if route_fresh is not True or inputs_valid is not True or lateral_active is not True:
      self.signal_since = None
      self._clear_evidence()
      if self.entered:
        self.terminal = True
      return self._decision('UNAVAILABLE', 'Fresh route, vehicle and lateral control required', now)
    if signal_announced is not True or vehicle_signal_matches is not True:
      self.signal_since = None
      self._clear_evidence()
      if self.entered:
        self.terminal = True
      return self._decision('ANNOUNCE', 'Announce first; wait for actual matching vehicle blinker', now)
    if self.signal_since is None:
      self.signal_since = now
    if driver_override is True or braking is True:
      self._clear_evidence()
      # An ordinary pre-entry handover can requalify. After entering, loss of
      # authority cannot silently reapply a cached highway exit preference.
      if self.entered:
        self.terminal = True
      return self._decision('DRIVER_OVERRIDE', 'Driver owns maneuver; withdraw exit desire', now)
    if now-self.signal_since < self.SIGNAL_LEAD_S:
      self._clear_evidence()
      return self._decision('ANNOUNCE', 'Matching blinker must precede maneuver request', now)
    valid = self._valid_evidence(evidence, key, direction, now, model_frame)
    if valid and self.last_evidence_frame is not None:
      # Repeated frames do not earn dwell; changed/future timestamps cannot
      # make an old camera frame fresh. Require ordered independent evidence.
      valid = (evidence.model_frame >= self.last_evidence_frame
               and (evidence.model_frame != self.last_evidence_frame
                    or evidence.sensor_mono == self.last_sensor_mono)
               and (evidence.model_frame == self.last_evidence_frame
                    or evidence.sensor_mono > self.last_sensor_mono))
    if not valid:
      self._clear_evidence()
      if self.entered:
        self.terminal = True
      return self._decision('WAIT_ENTRY' if not self.terminal else 'ABORTED',
                            'No qualified independent drivable exit corridor', now)
    if evidence.model_frame != self.last_evidence_frame:
      if self.evidence_since is None:
        self.evidence_since = now
      self.evidence_frames += 1
      self.last_evidence_frame = evidence.model_frame
      self.last_sensor_mono = evidence.sensor_mono
    if self.evidence_frames < self.QUALIFY_FRAMES or now-self.evidence_since < self.QUALIFY_S:
      return self._decision('WAIT_ENTRY', 'Qualifying continuous fresh corridor evidence', now)
    phase = 'COMMIT' if self.entered else 'ENTER'
    self.entered = True
    return self._decision(phase, 'Signal lead and independent corridor checks qualified', now,
                          'keepLeft' if direction == 1 else 'keepRight')
