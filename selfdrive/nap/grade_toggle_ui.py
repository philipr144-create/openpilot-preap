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
