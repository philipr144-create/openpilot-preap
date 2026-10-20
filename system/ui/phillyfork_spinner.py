"""PhillyFork neon-road *startup-only* screen.

This module is imported by system.ui.spinner at runtime only. It does not
alter onroad UI, alerts, controls, model output or the manager process list.
"""
import math

import pyray as rl

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.text import wrap_text
from openpilot.system.ui.widgets import Widget


CYAN = rl.Color(31, 220, 255, 255)
SOFT_CYAN = rl.Color(102, 236, 255, 170)
TRACK = rl.Color(9, 27, 46, 235)
PANEL = rl.Color(4, 13, 27, 206)
TEXT = rl.Color(239, 247, 255, 240)


class PhillyForkSpinner(Widget):
  """Fullscreen animated road + actual logo assets + real progress semantics."""

  def __init__(self):
    super().__init__()
    # Fail during initialization (before first frame) if a required image cannot
    # be loaded. spinner.py falls back to the stock spinner in that case.
    self._road = gui_app.texture("images/phillyfork_road.png")
    # This upstream filename actually contains the authentic red/blue NotAutopilot logo.
    # Reuse the original PNG; do not generate or duplicate a substitute.
    self._notautopilot = gui_app.texture("images/spinner_comma.png")
    self._progress = None
    self._status = []
    self._phase = 0.0

  def set_text(self, value: str) -> None:
    if value.isdigit():
      self._progress = max(0, min(100, int(value)))
      self._status = []
    else:
      self._progress = None
      # Real startup messages are preserved, not silently discarded.
      self._status = [value] if value else []

  @staticmethod
  def _logo(texture, x: float, y: float, size: float):
    if texture.width <= 0 or texture.height <= 0:
      return
    scale = min(size / texture.width, size / texture.height)
    width, height = texture.width * scale, texture.height * scale
    rl.draw_texture_pro(texture,
                        rl.Rectangle(0, 0, texture.width, texture.height),
                        rl.Rectangle(x + (size - width) / 2.0,
                                     y + (size - height) / 2.0, width, height),
                        rl.Vector2(0, 0), 0.0, rl.WHITE)

  def _render(self, rect: rl.Rectangle):
    w, h = float(rect.width), float(rect.height)
    if w <= 1 or h <= 1:
      return
    elapsed = rl.get_frame_time()
    self._phase = (self._phase + min(max(elapsed, 0.0), 0.2) * 0.22) % 1.0

    # Cover the display without squashing the artwork. Keep the road center
    # aligned to the middle of the view.
    img_w, img_h = self._road.width, self._road.height
    scale = max(w / img_w, h / img_h)
    source_w, source_h = w / scale, h / scale
    src = rl.Rectangle((img_w - source_w) / 2.0,
                       (img_h - source_h) / 2.0, source_w, source_h)
    rl.draw_texture_pro(self._road, src, rl.Rectangle(rect.x, rect.y, w, h),
                        rl.Vector2(0, 0), 0.0, rl.WHITE)

    # A handful of animated highlights follow the perspective lane markers.
    # Pure cosmetic animation; unrelated to steering/model lane predictions.
    for side in (-1, 1):
      for i in range(5):
        travel = (self._phase + i / 5.0) % 1.0
        t = 0.06 + 0.94 * travel
        y = rect.y + h * (0.475 + 0.53 * t ** 1.6)
        x = rect.x + w * (0.5 + side * (0.004 + 0.405 * t ** 1.32))
        radius = max(2.0, min(w, h) * (0.0015 + 0.004 * t))
        opacity = int(148 * math.sin(math.pi * travel))
        if opacity > 2 and y < rect.y + h:
          rl.draw_circle_v(rl.Vector2(x, y), radius,
                           rl.Color(72, 230, 255, opacity))

    # A translucent picture-in-picture plaque showing the untouched original
    # NotAutopilot image (spinner_comma.png). No text or invented logos.
    icon_size = max(38.0, min(104.0, h * 0.095))
    pad = max(14.0, h * 0.022)
    panel_width = icon_size + pad * 2
    panel_height = icon_size + pad * 2
    panel_x = rect.x + w - panel_width - max(20.0, w * 0.025)
    panel_y = rect.y + max(20.0, h * 0.035)
    rl.draw_rectangle_rounded(rl.Rectangle(panel_x, panel_y, panel_width, panel_height),
                              0.17, 10, PANEL)
    rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(panel_x, panel_y, panel_width, panel_height),
                                       0.17, 10, 1.25, rl.Color(23, 130, 175, 120))
    self._logo(self._notautopilot, panel_x + pad, panel_y + pad, icon_size)

    # Keep the spinner's existing numeric progress vs. status-message contract.
    bar_width = min(w * 0.49, 900.0)
    bar_height = max(8.0, min(14.0, h * 0.012))
    bar_x = rect.x + (w - bar_width) / 2.0
    bar_y = rect.y + h * 0.88
    rl.draw_rectangle_rounded(rl.Rectangle(bar_x, bar_y, bar_width, bar_height),
                              0.95, 10, TRACK)
    if self._progress is not None:
      portion = self._progress / 100.0
      if portion > 0:
        rl.draw_rectangle_rounded(rl.Rectangle(bar_x, bar_y,
                                               max(2.0, bar_width * portion), bar_height),
                                  0.95, 10, CYAN)
    else:
      # Indeterminate progress never pretends to be a completion percentage.
      travel = (self._phase * 2.6) % 1.0
      sweep_width = bar_width * 0.22
      x = bar_x + (bar_width - sweep_width) * travel
      rl.draw_rectangle_rounded(rl.Rectangle(x, bar_y, sweep_width, bar_height),
                                0.95, 10, SOFT_CYAN)

    if self._status:
      font_size = max(18.0, min(36.0, h * 0.032))
      max_text_width = w * 0.88
      # Wrap messages so startup errors remain readable on narrower screens.
      lines = wrap_text(self._status[-1], font_size, max_text_width)
      for line_i, line in enumerate(lines[:2]):
        size = measure_text_cached(gui_app.font(), line, font_size)
        rl.draw_text_ex(gui_app.font(), line,
                        rl.Vector2(rect.x + (w - size.x) / 2.0,
                                   min(rect.y + h - font_size * 1.2,
                                       bar_y + 25 + line_i * font_size * 1.1)),
                        font_size, 0.0, TEXT)
