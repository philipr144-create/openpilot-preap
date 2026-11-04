"""Display existing model lead estimates; no detector or vehicle control output."""
import json
import math
import os
from pathlib import Path
import tempfile
import time

CONFIG = Path('/data/nap_settings/vision_leads.json')
MAX_AGE = 0.5


def read_enabled(path=CONFIG):
  try:
    data = json.loads(Path(path).read_text()[:1024])
    return type(data) is dict and data.get('enabled') is True
  except (OSError, ValueError, TypeError):
    return False


def write_enabled(enabled, path=CONFIG):
  path = Path(path)
  temporary = None
  try:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as stream:
      temporary = Path(stream.name)
      json.dump({'enabled': bool(enabled)}, stream)
      stream.flush()
      os.fsync(stream.fileno())
    os.replace(temporary, path)
    return True
  except OSError:
    return False
  finally:
    if temporary is not None:
      temporary.unlink(missing_ok=True)


def fresh_model(seen, valid, received, started, now):
  return bool(seen and valid and received >= started and 0 <= now - received <= MAX_AGE)


def lead_estimates(leads):
  result = []
  for lead in list(leads)[:3]:
    if not len(lead.x) or not len(lead.y):
      continue
    x, y, probability, horizon = float(lead.x[0]), float(lead.y[0]), float(lead.prob), float(lead.probTime)
    if (all(math.isfinite(v) for v in (x, y, probability, horizon))
        and 0 <= probability <= 1 and 0 <= horizon <= 10):
      # x/y are model device-frame coordinates; y positive to the left.
      # Keep all valid probabilities, including those below fusion's cutoff.
      result.append((x, y, probability, horizon))
  return result


def project_box(transform, x, y, ground_z, rect):
  """Fixed 1.8 x 1.5 m envelope, not a detected outline. Device y is left."""
  if not 1.0 < x <= 150.0 or abs(y) > 30.0:
    return None
  corners = []
  for side, height in ((-.9, -1.5), (.9, -1.5), (.9, 0.), (-.9, 0.)):
    point = (x, y + side, ground_z + height)
    projected = [sum(float(transform[r][c]) * point[c] for c in range(3)) for r in range(3)]
    if not all(math.isfinite(v) for v in projected) or projected[2] <= 1e-3:
      return None
    corners.append((projected[0] / projected[2], projected[1] / projected[2]))
  left, right = min(p[0] for p in corners), max(p[0] for p in corners)
  top, bottom = min(p[1] for p in corners), max(p[1] for p in corners)
  if right < rect.x or left > rect.x + rect.width or bottom < rect.y or top > rect.y + rect.height:
    return None
  left, right = max(left, rect.x), min(right, rect.x + rect.width)
  top, bottom = max(top, rect.y), min(bottom, rect.y + rect.height)
  if right - left < 2 or bottom - top < 2:
    return None
  return left, top, right - left, bottom - top


class VisionLeadOverlay:
  def __init__(self, renderer):
    self.renderer = renderer
    self.enabled = False
    self.checked = -math.inf
    self.last_error = -math.inf

  def render(self, rect):
    try:
      self._render(rect)
    except Exception:
      # Optional diagnostics must never take down the main driving display.
      now = time.monotonic()
      if now - self.last_error >= 10.0:
        from openpilot.common.swaglog import cloudlog
        cloudlog.exception('Vision lead display unavailable')
        self.last_error = now

  def _render(self, rect):
    import pyray as rl
    from openpilot.selfdrive.ui.ui_state import ui_state
    from openpilot.system.ui.lib.application import gui_app, FontWeight

    now = time.monotonic()
    if now - self.checked >= 1.0:
      self.enabled = read_enabled()
      self.checked = now
    if not self.enabled or not ui_state.started:
      return
    sm = ui_state.sm
    fresh = fresh_model(sm.seen['modelV2'], sm.valid['modelV2'], sm.recv_time['modelV2'],
                        ui_state.started_time, now)
    geometry_fresh = (sm.valid['liveCalibration'] and sm.alive['liveCalibration']
                      and sm.recv_frame['liveCalibration'] >= ui_state.started_frame)
    renderer = self.renderer
    path = renderer._path.raw_points
    estimates = lead_estimates(sm['modelV2'].leadsV3) if fresh else []
    font = gui_app.font(FontWeight.NORMAL)
    if fresh and geometry_fresh and len(path):
      # Future selections first so the current estimate stays visible on top.
      for x, y, probability, horizon in sorted(estimates, key=lambda p: p[3], reverse=True):
        idx = renderer._get_path_length_idx(path[:, 0], x)
        ground_z = float(path[idx, 2]) + renderer._path_offset_z
        box = project_box(renderer._car_space_transform, x, y, ground_z, rect)
        if box is None:
          continue
        current = horizon < .1
        rgb = (70, 215, 255) if current else (255, 190, 65)
        color = rl.Color(*rgb, round(40 + 190 * probability))
        if current:
          rl.draw_rectangle_lines_ex(rl.Rectangle(*box), 3.0, color)
        else:
          # Corner marks distinguish future selections even when boxes overlap.
          left, top, width, height = box
          length = min(14.0, width * .3, height * .3)
          for px, py, dx, dy in ((left,top,1,1),(left+width,top,-1,1),
                                 (left,top+height,1,-1),(left+width,top+height,-1,-1)):
            rl.draw_line_ex(rl.Vector2(px,py),rl.Vector2(px+dx*length,py),1.5,color)
            rl.draw_line_ex(rl.Vector2(px,py),rl.Vector2(px,py+dy*length),1.5,color)
    if not fresh:
      status = 'VISION LEADS: waiting / stale - no boxes'
    elif not geometry_fresh:
      status = 'VISION LEADS: waiting for camera geometry'
    elif not estimates:
      status = 'VISION LEADS: no valid estimates'
    else:
      values = [f'{"now" if h < .1 else f"+{h:g}s"}: {p:.0%}' for _, _, p, h in estimates]
      status = 'VISION LEADS: ' + ' | '.join(values) + '  (cyan: now / amber: future)'
    x, y = rect.x + 24, rect.y + rect.height - 245
    rl.draw_rectangle(int(x - 8),int(y - 5),min(int(rect.width - 32),1100),40,rl.Color(0,0,0,155))
    rl.draw_text_ex(font,status,rl.Vector2(x,y),24,0,rl.Color(225,240,245,235))
