"""UI-side adapter for V9e's existing file-backed enable switch.

This deliberately does not create a new Params key: controlsd reads the
same JSON file, and the existing grade_toggle.py CLI can modify it as well.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

GRADE_FILE = Path('/data/nap_settings/v9e_predictive_grade.json')


def read_grade_enabled(path=GRADE_FILE):
  """Mirror FileGradeToggle: malformed, missing, or symlinked config = OFF."""
  path = Path(path)
  try:
    if path.is_symlink():
      return False
    with path.open('rb') as f:
      content = f.read(512)
    data = json.loads(content)
    return isinstance(data, dict) and data.get('enabled') is True
  except (OSError, ValueError, TypeError):
    return False


def _driving_process_running():
  """Match the existing V9e CLI's conservative control-process guard."""
  try:
    output = subprocess.check_output(['ps', '-eo', 'pid=,args='], text=True, timeout=2)
  except (OSError, subprocess.SubprocessError):
    return True  # fail closed: don't enable if the guard is unavailable
  for line in output.splitlines():
    if re.search(r'(?<![\w])(controlsd|modeld|plannerd)(?:\.py)?(?:\s|$)', line):
      return True
  return False


def write_grade_enabled(enabled, *, offroad, path=GRADE_FILE, process_checker=None):
  """Atomically update the existing V9e switch, or return False if unsafe.

  Enable only while offroad AND without control processes. Disabling is
  always allowed. Do not silently create a missing config or follow symlinks.
  """
  if not isinstance(enabled, bool):
    return False
  path = Path(path)
  if enabled:
    if not offroad:
      return False
    check = process_checker or _driving_process_running
    if check():
      return False
  if path.is_symlink() or not path.is_file() or path.parent.is_symlink():
    return False
  temp_path = None
  try:
    # Keep the operation within the same directory for an atomic replace.
    with tempfile.NamedTemporaryFile('w', encoding='utf-8', dir=str(path.parent),
                                     prefix='.v9e_ui_', suffix='.tmp', delete=False) as f:
      temp_path = Path(f.name)
      json.dump({'enabled': enabled}, f)
      f.write('\n')
      f.flush()
      os.fsync(f.fileno())
    # Recheck; never write through or overwrite a symlink selected meanwhile.
    if path.is_symlink() or not path.is_file():
      return False
    os.replace(temp_path, path)
    temp_path = None
    return read_grade_enabled(path) is enabled
  except (OSError, ValueError, TypeError):
    return False
  finally:
    if temp_path is not None:
      try:
        temp_path.unlink(missing_ok=True)
      except OSError:
        pass


DIAG_FILE = Path('/data/nap_settings/predictive_grade_diagnostics.json')
STATUS_FILE = Path('/dev/shm/nap_predictive_grade_status.json')

def read_grade_diagnostics():
  return read_grade_enabled(DIAG_FILE)

def write_grade_diagnostics(enabled, *, offroad):
  return write_grade_enabled(enabled, offroad=offroad, path=DIAG_FILE)

def grade_diagnostic_description():
  import time
  base = 'Record pitch prediction, actual added acceleration, and blocking reasons. No control change from this switch.'
  if not read_grade_diagnostics():
    return base
  labels = {
    'TRACKING_CLIMB': 'Tracking the climb', 'INVALID_FORECAST': 'Waiting for valid model forecast',
    'MODEL_BRAKING': 'Model slowdown has priority', 'MPC_BRAKING': 'Cruise braking has priority',
    'ACCEL_CEILING': 'Limited by acceleration ceiling', 'APPLYING': 'Adding uphill support', 'FINAL_ACCEL_LIMIT': 'Limited by acceleration cap', 'TOGGLE_OFF': 'Pitch assist OFF',
    'NOT_ENGAGED': 'Assistance disengaged', 'SPEED_RANGE': 'Outside 20–80 mph range',
    'NEGATIVE_BASE_REQUEST': 'Blocked: planner requests slowing',
    'NO_UPHILL_PREDICTION': 'No upcoming uphill change predicted',
    'PITCH_UNAVAILABLE': 'Pitch prediction unavailable', 'CONFIRMING_UPHILL': 'Confirming uphill prediction',
    'LEAD_NEAR_OR_CLOSING': 'Blocked: lead close or closing', 'BRAKING_OWNER': 'Blocked: braking or speed constraint',
    'NO_MODEL_DRIVE': 'Blocked: model not requesting propulsion',
    'NO_MPC_HEADROOM': 'No lead-aware acceleration headroom',
    'BASE_ALREADY_ACCELERATING': 'Base request already accelerating', 'ALREADY_ACCELERATING': 'Vehicle already accelerating',
    'DRIVER_GAS': 'Driver accelerator override', 'DRIVER_BRAKE': 'Driver braking',
    'MODEL_STOP': 'Blocked: model stop', 'MPC_STOP': 'Blocked: planned stop',
    'CURVE_BRAKING': 'Blocked: curve braking', 'FORCE_DECEL': 'Blocked: forced slowdown',
    'SPEED_RESTRICTION': 'Blocked: active speed restriction', 'ABOVE_TARGET': 'Above target speed',
  }
  try:
    if STATUS_FILE.is_symlink():
      return base
    data = json.loads(STATUS_FILE.read_bytes()[:4096])
    age = time.monotonic() - float(data['time'])
    if not 0 <= age < 3.0:
      return base + ' Waiting for fresh driving data.'
    label = labels.get(data.get('reason'), 'Waiting for valid inputs')
    return base + f" Last status: {label}. Added {float(data['applied_add']):+.3f} m/s²."
  except (OSError, ValueError, TypeError, KeyError):
    return base + ' Waiting for driving data.'
