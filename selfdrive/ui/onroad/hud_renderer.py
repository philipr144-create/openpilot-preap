import pyray as rl
from dataclasses import dataclass
from openpilot.common.constants import CV
from openpilot.selfdrive.ui.onroad.exp_button import ExpButton
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.multilang import tr
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

try:
  from openpilot.common.params import Params
  _params = Params()
except Exception:
  _params = None

FOLLOW_KEY_STR = "NAPFollowDistance"
PERSONALITY_KEY = "LongitudinalPersonality"
PERSONALITIES = ["Relaxed", "Standard", "Aggressive"]

SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'

@dataclass(frozen=True)
class UIConfig:
  header_height: int = 300
  border_size: int = 30
  button_size: int = 192
  set_speed_width_metric: int = 200
  set_speed_width_imperial: int = 172
  set_speed_height: int = 204
  wheel_icon_size: int = 144

@dataclass(frozen=True)
class FontSizes:
  current_speed: int = 176
  speed_unit: int = 66
  max_speed: int = 40
  set_speed: int = 90
  ctrl_btn: int = 90
  ctrl_lbl: int = 55

@dataclass(frozen=True)
class Colors:
  WHITE = rl.WHITE
  DISENGAGED = rl.Color(145, 155, 149, 255)
  OVERRIDE = rl.Color(145, 155, 149, 255)
  ENGAGED = rl.Color(128, 216, 166, 255)
  DISENGAGED_BG = rl.Color(0, 0, 0, 153)
  OVERRIDE_BG = rl.Color(145, 155, 149, 204)
  ENGAGED_BG = rl.Color(128, 216, 166, 204)
  GREY = rl.Color(166, 166, 166, 255)
  DARK_GREY = rl.Color(114, 114, 114, 255)
  BLACK_TRANSLUCENT = rl.Color(0, 0, 0, 166)
  WHITE_TRANSLUCENT = rl.Color(255, 255, 255, 200)
  BORDER_TRANSLUCENT = rl.Color(255, 255, 255, 75)
  HEADER_GRADIENT_START = rl.Color(0, 0, 0, 114)
  HEADER_GRADIENT_END = rl.BLANK
  BTN_ACTIVE = rl.Color(128, 216, 166, 255)

UI_CONFIG = UIConfig()
FONT_SIZES = FontSizes()
COLORS = Colors()

class HudRenderer(Widget):
  def __init__(self):
    super().__init__()
    self.is_cruise_set = False
    self.is_cruise_available = True
    self.set_speed = SET_SPEED_NA
    self.speed = 0.0
    self.v_ego_cluster_seen = False
    self._font_semi_bold = gui_app.font(FontWeight.SEMI_BOLD)
    self._font_bold = gui_app.font(FontWeight.BOLD)
    self._font_medium = gui_app.font(FontWeight.MEDIUM)
    self._exp_button = ExpButton(UI_CONFIG.button_size, UI_CONFIG.wheel_icon_size)
    self.follow_dist = 3
    self.personality = 1
    self._last_param_check_frame = 0
    self._ignore_param_reads_until = 0
    self._btn_pressed = None
    self._dist_minus_rect = rl.Rectangle(0, 0, 0, 0)
    self._dist_plus_rect = rl.Rectangle(0, 0, 0, 0)
    self._pers_minus_rect = rl.Rectangle(0, 0, 0, 0)
    self._pers_plus_rect = rl.Rectangle(0, 0, 0, 0)

  def _safe_write_param(self, key: str, val: int) -> None:
    if _params is None: return
    self._ignore_param_reads_until = self._last_param_check_frame + 60
    try:
      _params.put_nonblocking(key, str(val).encode('utf8'))
    except Exception:
      try: _params.put(key, str(val).encode('utf8'))
      except Exception: pass

  def _update_params(self) -> None:
    if _params is None: return
    if self._last_param_check_frame < self._ignore_param_reads_until: return
    try:
      d_raw = _params.get(FOLLOW_KEY_STR)
      if d_raw:
        val = int(d_raw.decode('utf-8') if isinstance(d_raw, bytes) else d_raw)
        if 1 <= val <= 7: self.follow_dist = val
      p_raw = _params.get(PERSONALITY_KEY)
      if p_raw:
        val = int(p_raw.decode('utf-8') if isinstance(p_raw, bytes) else p_raw)
        if 0 <= val <= 2: self.personality = val
    except Exception: pass

  def _set_dist(self, change: int) -> None:
    new_val = max(1, min(7, self.follow_dist + change))
    if new_val != self.follow_dist:
      self.follow_dist = new_val
      self._safe_write_param(FOLLOW_KEY_STR, self.follow_dist)

  def _set_pers(self, change: int) -> None:
    new_val = max(0, min(2, int(self.personality) + int(change)))

    if new_val != int(self.personality):
      try:
        # This is the same persistent parameter used by Settings/selfdrived.
        Params().put(PERSONALITY_KEY, str(new_val))
        self.personality = new_val
        print(f"[HUD] LongitudinalPersonality -> {new_val}")
      except Exception as e:
        print(f"[HUD] Failed to write LongitudinalPersonality: {e}")

  def _update_state(self) -> None:
    self._last_param_check_frame += 1
    if self._last_param_check_frame % 15 == 0: self._update_params()
    sm = ui_state.sm
    if sm.recv_frame["carState"] < ui_state.started_frame:
      self.is_cruise_set = False; self.set_speed = SET_SPEED_NA; self.speed = 0.0; return
    controls_state = sm['controlsState']; car_state = sm['carState']
    v_cruise_cluster = car_state.vCruiseCluster
    self.set_speed = controls_state.vCruiseDEPRECATED if v_cruise_cluster == 0.0 else v_cruise_cluster
    self.is_cruise_set = 0 < self.set_speed < SET_SPEED_NA
    self.is_cruise_available = self.set_speed != -1
    if self.is_cruise_set and not ui_state.is_metric: self.set_speed *= KM_TO_MILE
    v_ego_cluster = car_state.vEgoCluster
    self.v_ego_cluster_seen = self.v_ego_cluster_seen or v_ego_cluster != 0.0
    v_ego = v_ego_cluster if self.v_ego_cluster_seen else car_state.vEgo
    speed_conversion = CV.MS_TO_KPH if ui_state.is_metric else CV.MS_TO_MPH
    self.speed = max(0.0, v_ego * speed_conversion)

  def _render(self, rect: rl.Rectangle) -> None:
    self._handle_touch_input()
    rl.draw_rectangle_gradient_v(int(rect.x), int(rect.y), int(rect.width), UI_CONFIG.header_height, COLORS.HEADER_GRADIENT_START, COLORS.HEADER_GRADIENT_END)
    if self.is_cruise_available: self._draw_set_speed(rect)
    self._draw_current_speed(rect)
    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + UI_CONFIG.border_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))
    self._draw_onscreen_controls(rect)

  def _handle_touch_input(self) -> None:
    mouse_pos = rl.get_mouse_position()
    mouse_down = rl.is_mouse_button_down(rl.MouseButton.MOUSE_BUTTON_LEFT)
    active_btn = None
    if rl.check_collision_point_rec(mouse_pos, self._dist_minus_rect): active_btn = "dist_min"
    elif rl.check_collision_point_rec(mouse_pos, self._dist_plus_rect): active_btn = "dist_plus"
    elif rl.check_collision_point_rec(mouse_pos, self._pers_minus_rect): active_btn = "pers_min"
    elif rl.check_collision_point_rec(mouse_pos, self._pers_plus_rect): active_btn = "pers_plus"
    
    if mouse_down:
      if self._btn_pressed is None and active_btn: self._btn_pressed = active_btn
    else:
      if self._btn_pressed:
        if active_btn == self._btn_pressed:
          if active_btn == "dist_min": self._set_dist(-1)
          elif active_btn == "dist_plus": self._set_dist(1)
          elif active_btn == "pers_min": self._set_pers(-1)
          elif active_btn == "pers_plus": self._set_pers(1)
        self._btn_pressed = None

  def _draw_onscreen_controls(self, rect: rl.Rectangle) -> None:
    x = rect.x + 60
    btn_w = 140; lbl_w = 320; h = 130; gap = 25
    base_y = rect.y + rect.height - h - 250
    
    y_pers = base_y - h - gap
    self._pers_minus_rect = rl.Rectangle(x, y_pers, btn_w, h)
    self._draw_btn(self._pers_minus_rect, "<", self._btn_pressed == "pers_min")
    lbl_rect1 = rl.Rectangle(x + btn_w + gap, y_pers, lbl_w, h)
    self._draw_label(lbl_rect1, PERSONALITIES[self.personality])
    self._pers_plus_rect = rl.Rectangle(x + btn_w + gap + lbl_w + gap, y_pers, btn_w, h)
    self._draw_btn(self._pers_plus_rect, ">", self._btn_pressed == "pers_plus")
    
    y_dist = base_y
    self._dist_minus_rect = rl.Rectangle(x, y_dist, btn_w, h)
    self._draw_btn(self._dist_minus_rect, "-", self._btn_pressed == "dist_min")
    lbl_rect2 = rl.Rectangle(x + btn_w + gap, y_dist, lbl_w, h)
    self._draw_label(lbl_rect2, f"DIST: {self.follow_dist}")
    self._dist_plus_rect = rl.Rectangle(x + btn_w + gap + lbl_w + gap, y_dist, btn_w, h)
    self._draw_btn(self._dist_plus_rect, "+", self._btn_pressed == "dist_plus")

  def _draw_btn(self, r: rl.Rectangle, text: str, pressed: bool) -> None:
    bg = COLORS.BLACK_TRANSLUCENT if not pressed else rl.Color(40, 40, 40, 220)
    border = COLORS.BTN_ACTIVE if pressed else COLORS.BORDER_TRANSLUCENT
    rl.draw_rectangle_rounded(r, 0.35, 10, bg)
    rl.draw_rectangle_rounded_lines_ex(r, 0.35, 10, 5, border)
    w = measure_text_cached(self._font_bold, text, FONT_SIZES.ctrl_btn).x
    offset_y = 15 if text in ["-", "+"] else 25
    rl.draw_text_ex(self._font_bold, text, rl.Vector2(r.x + (r.width - w) / 2, r.y + offset_y), FONT_SIZES.ctrl_btn, 0, COLORS.WHITE)

  def _draw_label(self, r: rl.Rectangle, text: str) -> None:
    rl.draw_rectangle_rounded(r, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(r, 0.35, 10, 4, COLORS.BORDER_TRANSLUCENT)
    w = measure_text_cached(self._font_semi_bold, text, FONT_SIZES.ctrl_lbl).x
    rl.draw_text_ex(self._font_semi_bold, text, rl.Vector2(r.x + (r.width - w) / 2, r.y + 35), FONT_SIZES.ctrl_lbl, 0, COLORS.WHITE)

  def user_interacting(self) -> bool: return self._exp_button.is_pressed or (self._btn_pressed is not None)
  def _draw_set_speed(self, rect: rl.Rectangle) -> None:
    set_speed_width = UI_CONFIG.set_speed_width_metric if ui_state.is_metric else UI_CONFIG.set_speed_width_imperial
    x = rect.x + 60 + (UI_CONFIG.set_speed_width_imperial - set_speed_width) // 2
    y = rect.y + 45
    set_speed_rect = rl.Rectangle(x, y, set_speed_width, UI_CONFIG.set_speed_height)
    rl.draw_rectangle_rounded(set_speed_rect, 0.35, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(set_speed_rect, 0.35, 10, 6, COLORS.BORDER_TRANSLUCENT)
    max_color = COLORS.GREY
    set_speed_color = COLORS.DARK_GREY
    if self.is_cruise_set:
      set_speed_color = COLORS.WHITE
      if ui_state.status == UIStatus.ENGAGED: max_color = COLORS.ENGAGED
      elif ui_state.status == UIStatus.DISENGAGED: max_color = COLORS.DISENGAGED
      elif ui_state.status == UIStatus.OVERRIDE: max_color = COLORS.OVERRIDE
    max_text = tr("MAX")
    max_text_width = measure_text_cached(self._font_semi_bold, max_text, FONT_SIZES.max_speed).x
    rl.draw_text_ex(self._font_semi_bold, max_text, rl.Vector2(x + (set_speed_width - max_text_width) / 2, y + 27), FONT_SIZES.max_speed, 0, max_color)
    set_speed_text = CRUISE_DISABLED_CHAR if not self.is_cruise_set else str(round(self.set_speed))
    speed_text_width = measure_text_cached(self._font_bold, set_speed_text, FONT_SIZES.set_speed).x
    rl.draw_text_ex(self._font_bold, set_speed_text, rl.Vector2(x + (set_speed_width - speed_text_width) / 2, y + 77), FONT_SIZES.set_speed, 0, set_speed_color)
  def _draw_current_speed(self, rect: rl.Rectangle) -> None:
    speed_text = str(round(self.speed))
    speed_text_size = measure_text_cached(self._font_bold, speed_text, FONT_SIZES.current_speed)
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, 180 - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)
    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, 290 - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
