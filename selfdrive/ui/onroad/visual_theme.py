"""Display-only preferences. Never read by driving controls."""
import colorsys
import json
import math
import os
from pathlib import Path
import tempfile

CONFIG = Path('/data/nap_settings/ui_visuals.json')
THEMES = ('Classic', 'Tesla blue', 'Rainbow', 'Dynamic')
DEFAULTS = dict(theme=0, wider=True, leads=True)

def read_settings(path=CONFIG):
  try:
    path = Path(path)
    if path.is_symlink() or path.parent.is_symlink():
      return DEFAULTS.copy()
    data = json.loads(path.read_bytes()[:1025])
    if (type(data) is not dict or set(data) != set(DEFAULTS) or
        type(data['theme']) is not int or data['theme'] not in range(4) or
        type(data['wider']) is not bool or type(data['leads']) is not bool):
      return DEFAULTS.copy()
    return data
  except (OSError, ValueError, TypeError):
    return DEFAULTS.copy()

def write_setting(key, value, *, offroad, path=CONFIG):
  if not offroad or key not in DEFAULTS:
    return False
  if key == 'theme':
    if type(value) is not int or value not in range(4):
      return False
  elif type(value) is not bool:
    return False
  path = Path(path)
  temporary = None
  try:
    if path.is_symlink() or path.parent.is_symlink():
      return False
    path.parent.mkdir(parents=True, exist_ok=True)
    data = read_settings(path)
    data[key] = value
    with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as stream:
      temporary = Path(stream.name)
      json.dump(data, stream)
      stream.flush()
      os.fsync(stream.fileno())
    if path.is_symlink():
      return False
    os.replace(temporary, path)
    return True
  except OSError:
    return False
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)

def acceleration_color(accel):
  # Model prediction, not measured acceleration or confirmed brake actuation.
  if not math.isfinite(accel):
    return (235, 235, 235)
  if accel < -0.15:
    return (255, 90, 65)
  if accel > 0.15:
    return (40, 245, 145)
  return (235, 235, 235)

def theme_colors(theme, accel, seconds):
  cue = acceleration_color(accel)
  result = []
  for i in range(9):
    t = i / 8
    rgb = ((45, 155, 255) if theme == 1 else cue)
    if theme == 2:
      rgb = tuple(round(c * 255) for c in colorsys.hsv_to_rgb((t * .8 - seconds * .08) % 1, .8, 1))
    result.append((*rgb, round(110 * max(0, 1 - t / .85))))
  return result
