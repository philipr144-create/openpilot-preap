"""Optional longitudinal diagnostics; all filesystem/logging I/O is off control path."""
import json
import math
import os
from pathlib import Path
import threading
import time
from openpilot.selfdrive.controls.lib.predictive_grade_assist import FileGradeToggle

DIAG_CONFIG = '/data/nap_settings/predictive_grade_diagnostics.json'
STATUS_FILE = '/dev/shm/nap_predictive_grade_status.json'


class GradeDiagnostics:
  def __init__(self, config=DIAG_CONFIG, status=STATUS_FILE, start_worker=True):
    self.toggle = FileGradeToggle(config, dt=1.0)
    self.status = Path(status)
    self.enabled = False
    self.snapshot = None
    self.stop = threading.Event()
    if start_worker:
      threading.Thread(target=self._run, name='nap_grade_diagnostics', daemon=True).start()

  def publish(self, assist, *, assist_enabled, base, added, final, speed, target, mpc, model,
              throttle, propulsion, recovery_reason, handoff, personality):
    if not self.enabled:
      return
    try:
      delta = assist.last_pitch_rise_deg
      record_reason = ('TOGGLE_OFF' if not assist_enabled else
                       'FINAL_ACCEL_LIMIT' if assist.output > 1e-6 and float(added) <= 1e-6 else assist.reason)
      self.snapshot = dict(time=time.monotonic(), assist_enabled=bool(assist_enabled),
        reason=record_reason, blocks=list(assist.block_reasons),
        pitch_rise_deg=delta if delta is not None and math.isfinite(delta) else None,
        controller_version=10, climb_active=bool(assist.climb_active),
        climb_remaining=float(assist.climb_remaining), load_estimate=float(assist.load_estimate),
        proposed_target=float(assist.proposal),
        hold_seconds=float(assist.hold), triggered=bool(assist.triggered),
        base_accel=float(base), proposed_add=float(assist.output), applied_add=float(added),
        final_accel=float(final), speed_mps=float(speed), target_mps=float(target),
        mpc_accel=float(mpc), model_accel=float(model), throttle_probability=float(throttle),
        propulsion_allowed=bool(propulsion), recovery_reason=str(recovery_reason),
        handoff=str(handoff), personality=int(getattr(personality, 'raw', personality)))
    except (AttributeError, TypeError, ValueError, OverflowError):
      return

  def flush(self, logger=None):
    self.enabled = self.toggle.read()
    data = self.snapshot
    if not self.enabled:
      if self.status.exists():
        self.status.unlink(missing_ok=True)
      return
    if data is None:
      return
    payload = json.dumps(data, allow_nan=False, separators=(',', ':'))
    temporary = self.status.with_suffix('.tmp')
    temporary.write_text(payload)
    os.replace(temporary, self.status)
    if logger is not None:
      logger.info('NAP_GRADE_DIAG_V9F %s', payload)

  def _run(self):
    from openpilot.common.swaglog import cloudlog
    while not self.stop.is_set():
      try:
        self.flush(cloudlog)
      except (OSError, ValueError, TypeError):
        pass  # A diagnostic failure cannot change driving output.
      self.stop.wait(1.0)
