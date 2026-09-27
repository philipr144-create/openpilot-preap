import pyray as rl
import json
import os
import time
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
PERSONALITIES = ["Aggressive", "Standard", "Relaxed"]

SET_SPEED_NA = 255
KM_TO_MILE = 0.621371
CRUISE_DISABLED_CHAR = '–'


def classify_intent(*, enabled, gas, brake, steering, left_blinker,
                    right_blinker, decision, should_stop, stopping,
                    source, accel, lead_distance=None):
  """Return glance-readable intent text without changing control behavior."""
  source = str(source).split('.')[-1].lower()
  decision = decision if isinstance(decision, dict) else {}

  if not enabled:
    return ('ASSIST READY', 'SYSTEM', 'Engage when ready', 'idle')
  if brake:
    return ('DRIVER BRAKING', 'DRIVER', 'Longitudinal control paused', 'driver')
  if gas:
    return ('DRIVER ACCEL', 'DRIVER', 'Speed control overridden', 'driver')
  if steering:
    return ('DRIVER STEERING', 'DRIVER', 'Steering assistance yielding', 'driver')

  if should_stop or stopping:
    stop_source = 'MODEL STOP' if source == 'e2e' else 'STOP CONTROL'
    if accel <= -2.2:
      return ('MAX REGEN', stop_source, 'Firm regenerative stop request', 'stop')
    return ('STOPPING', stop_source, 'Regenerative braking active', 'stop')

  one_blinker = bool(left_blinker) != bool(right_blinker)
  direction = decision.get('manual_turn_direction')
  matching = one_blinker and (
    direction == 1 and left_blinker or direction == 2 and right_blinker
  )
  if matching and decision.get('city_turns_enabled') is True:
    side = 'LEFT' if direction == 1 else 'RIGHT'
    expected = 'turnLeft' if direction == 1 else 'turnRight'
    if decision.get('final_desire') == expected:
      return (f'TURNING {side}', 'CITY TURN',
              'Physical blinker owns direction', 'turn')
    return (f'CHECKING {side}', 'CITY TURN',
            'Waiting for genuine turn geometry', 'turn')

  desire = str(decision.get('desire') or decision.get('final_desire') or '')
  # A driver-owned City Turn may only be displayed when its direction still
  # matches the live physical blinker. Do not relabel stale manual intent as
  # an automatic navigation maneuver.
  manual_decision = decision.get('manual_turn_direction') in (1, 2)
  if not manual_decision and desire in ('turnLeft', 'turnRight'):
    side = 'LEFT' if desire == 'turnLeft' else 'RIGHT'
    return (f'NAV TURN {side}', 'NAVIGATION', 'Route maneuver confirmed', 'turn')

  if source in ('lead0', 'lead1', 'lead2'):
    detail = ('Lead vehicle controls speed' if lead_distance is None
              else f'Lead vehicle {lead_distance:.0f} ft ahead')
    return ('FOLLOWING', 'LEAD', detail, 'lead')
  if source == 'e2e':
    return ('END-TO-END', 'MODEL', 'Model controls speed', 'model')
  if accel < -0.15:
    return ('SLOWING', 'CRUISE PLAN', 'Speed target is decreasing', 'slow')
  if accel > 0.15:
    return ('ACCELERATING', 'CRUISE PLAN', 'Returning to target speed', 'cruise')
  return ('CRUISING', 'CRUISE PLAN', 'Holding target speed', 'cruise')

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
    self._nav_checked = 0.0
    self._nav_view = None
    self._nav_map = ''
    self._nav_debug_written = 0.0
    self._intent_title = 'ASSIST READY'
    self._intent_source = 'SYSTEM'
    self._intent_detail = 'Waiting for live controls'
    self._intent_kind = 'idle'
    self._intent_target = None
    self._intent_accel = 0.0
    self._intent_lead = None

  def _read_nav_file(self, path, now, stamp, max_age=3.0):
    try:
      with open(path) as stream:
        raw = stream.read(2049)
      data = json.loads(raw) if len(raw) <= 2048 else None
      age = now - data[stamp]
      return data if 0 <= age <= max_age else None
    except (OSError, ValueError, TypeError, KeyError):
      return None

  def _update_nav_view(self):
    now = time.monotonic()
    if now - self._nav_checked < .25:
      return
    self._nav_checked = now
    nav = self._read_nav_file('/dev/shm/nap_navigation_desire.json', now, 'received_mono')
    if not nav or not nav.get('route_active'):
      self._nav_view = ('NAVIGATION', 'NO ACTIVE ROUTE', '', 'Set a destination on the phone')
      self._nav_map = ''
      return
    if nav.get('route_state') != 'active' or now >= nav.get('expires_mono', 0):
      self._nav_view = ('NAV PAUSED', 'WAITING', '',
                        str(nav.get('pause_reason') or 'Waiting for comma GPS')[:65])
      self._nav_map = ''
      return
    maneuver = nav.get('maneuver') or {}
    distance = maneuver.get('distance_m')
    if type(distance) not in (int, float):
      self._nav_view = ('NAV PAUSED', 'WAITING', '',
                        'Invalid maneuver · waiting for route update')
      self._nav_map = ''
      return
    feet = max(0, distance * 3.28084)
    away = f'{round(feet / 50) * 50:.0f} ft' if feet < 1000 else f'{feet / 5280:.1f} mi'
    title = str(maneuver.get('primary_text') or 'Route guidance')[:42]

    modifier = str(maneuver.get('modifier') or '').lower()
    kind = str(maneuver.get('type') or '').lower()

    if 'left' in modifier:
      direction = 'LEFT'
    elif 'right' in modifier:
      direction = 'RIGHT'
    elif kind in ('arrive', 'destination'):
      direction = 'ARRIVE'
    elif kind in ('turn', 'end of road', 'fork', 'off ramp'):
      direction = 'AHEAD'
    else:
      direction = 'ROUTE'

    decision = self._read_nav_file('/dev/shm/nap_navigation_decision.json', now, 'received_mono', 2)
    signal = self._read_nav_file('/dev/shm/nap_navigation_signal_status.json', now, 'time', 1)
    if nav.get('enabled') is not True:
      detail = 'CACHED ROUTE  ·  NAV ONLY'
    elif signal and signal.get('active') and signal.get('sent_recent'):
      detail = 'CACHED ROUTE  ·  SIGNAL SENT'
    elif decision and decision.get('synthetic_indicator_request') in ('left', 'right'):
      reason = signal.get('block_reason', 'signal pending') if signal else 'signal status unavailable'
      detail = 'SIGNAL WAITING  ·  ' + reason[:37]
    else:
      detail = 'CACHED ROUTE  ·  ' + str((decision or {}).get('reason') or 'signal not requested')[:38]
    stop = nav.get('mapped_stop_sign')
    if isinstance(stop, dict) and type(stop.get('distance_m')) in (int, float):
      self._nav_map = f"Mapped stop {max(0, round(stop['distance_m'] * 3.28084 / 50) * 50):.0f} ft ahead"
    else:
      lookup = nav.get('map_lookup') or {}
      state = lookup.get('map_lookup_state')
      count = lookup.get('map_loaded_ahead', 0)
      self._nav_map = ('Mapped stops off' if state == 'off' else
                       'Map lookup unavailable' if state in ('unavailable', 'partial') else
                       f'{count} mapped stops ahead' if state == 'ready' else 'Mapped stops: waiting for lookup')
    self._nav_view = (direction, away, title[:42], detail[:52])

  def _draw_nav_view(self, rect):
    self._update_nav_view()
    if self._nav_view is None:
      return

    direction, away, title, detail = self._nav_view

    # Navigation occupies the right-hand card position.
    width = min(680, rect.width - 120)
    height = 275
    box = rl.Rectangle(
      rect.x + rect.width - width - 60,
      rect.y + 260,
      width,
      height,
    )

    rl.draw_rectangle_rounded(box, .12, 8, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(
      box, .12, 8, 4, COLORS.BORDER_TRANSLUCENT
    )

    # Large direction and distance are the two glance-readable elements.
    rl.draw_text_ex(
      self._font_bold,
      direction[:12],
      rl.Vector2(box.x + 24, box.y + 14),
      58, 0, COLORS.WHITE,
    )

    rl.draw_text_ex(
      self._font_bold,
      away[:20],
      rl.Vector2(box.x + 24, box.y + 78),
      56, 0, COLORS.BTN_ACTIVE,
    )

    # Street/maneuver description.
    rl.draw_text_ex(
      self._font_semi_bold,
      title[:38],
      rl.Vector2(box.x + 24, box.y + 145),
      30, 0, COLORS.WHITE,
    )

    # Navigation/signal state.
    rl.draw_text_ex(
      self._font_medium,
      detail[:48],
      rl.Vector2(box.x + 24, box.y + 191),
      25, 0, COLORS.GREY,
    )

    # Map/stop information.
    rl.draw_text_ex(
      self._font_medium,
      self._nav_map[:48],
      rl.Vector2(box.x + 24, box.y + 229),
      23, 0, COLORS.GREY,
    )

    now = time.monotonic()
    if now - self._nav_debug_written >= 2:
      self._nav_debug_written = now
      try:
        path = '/dev/shm/nap_nav_ui_status.json'
        with open(path + '.tmp', 'w') as stream:
          json.dump({
            'time': now,
            'state': direction,
            'route_visible': direction not in ('NAVIGATION', 'NAV PAUSED'),
          }, stream)
        os.replace(path + '.tmp', path)
      except OSError:
        pass

  def _safe_write_param(self, key: str, val: int) -> None:
    if _params is None: return
    self._ignore_param_reads_until = self._last_param_check_frame + 60
    try:
      _params.put_nonblocking(key, int(val))
    except Exception:
      try: _params.put(key, int(val))
      except Exception: pass

  def _update_params(self) -> None:
    if _params is None: return
    if self._last_param_check_frame < self._ignore_param_reads_until: return
    try:
      d_raw = _params.get(FOLLOW_KEY_STR)
      if d_raw is not None:
        val = int(d_raw)
        if 1 <= val <= 7: self.follow_dist = val
      p_raw = _params.get(PERSONALITY_KEY)
      if p_raw is not None:
        val = int(p_raw)
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
        Params().put(PERSONALITY_KEY, int(new_val))
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
    self._update_intent(sm, speed_conversion)

  def _update_intent(self, sm, speed_conversion) -> None:
    try:
      plan_fresh = (sm.valid['longitudinalPlan'] and sm.alive['longitudinalPlan']
                    and sm.recv_frame['longitudinalPlan'] >= ui_state.started_frame)
      control_fresh = (sm.valid['carControl'] and sm.alive['carControl']
                       and sm.recv_frame['carControl'] >= ui_state.started_frame)
      radar_fresh = (sm.valid['radarState'] and sm.alive['radarState']
                     and sm.recv_frame['radarState'] >= ui_state.started_frame)

      plan = sm['longitudinalPlan'] if plan_fresh else None
      control = sm['carControl'] if control_fresh else None
      car_state = sm['carState']
      now = time.monotonic()
      decision = self._read_nav_file(
        '/dev/shm/nap_navigation_decision.json', now, 'received_mono', .6
      ) or {}

      source = str(plan.longitudinalPlanSource) if plan is not None else 'cruise'
      should_stop = bool(plan.shouldStop) if plan is not None else False
      accel = float(control.actuators.accel) if control is not None else (
        float(plan.aTarget) if plan is not None else 0.0
      )
      stopping = str(sm['controlsState'].longControlState) == 'stopping'

      self._intent_target = None
      if plan is not None:
        speeds = [float(v) for v in plan.speeds if float(v) >= 0.0]
        if speeds:
          # End of the published horizon communicates where the plan is
          # heading without replacing the existing cruise/set-speed display.
          self._intent_target = speeds[-1] * speed_conversion

      self._intent_lead = None
      if radar_fresh:
        radar = sm['radarState']
        lead = radar.leadOne
        if lead.status and float(lead.dRel) >= 0.0:
          self._intent_lead = float(lead.dRel) * 3.28084

      self._intent_title, self._intent_source, self._intent_detail, self._intent_kind = classify_intent(
        enabled=ui_state.engaged,
        gas=bool(car_state.gasPressed),
        brake=bool(car_state.brakePressed),
        steering=bool(car_state.steeringPressed),
        left_blinker=bool(car_state.leftBlinker),
        right_blinker=bool(car_state.rightBlinker),
        decision=decision,
        should_stop=should_stop,
        stopping=stopping,
        source=source,
        accel=accel,
        lead_distance=self._intent_lead,
      )
      self._intent_accel = accel
    except (AttributeError, KeyError, TypeError, ValueError, OverflowError):
      self._intent_title = 'INTENT UNAVAILABLE'
      self._intent_source = 'WAITING'
      self._intent_detail = 'Waiting for fresh planner data'
      self._intent_kind = 'idle'
      self._intent_target = None
      self._intent_accel = 0.0
      self._intent_lead = None

  def _draw_intent_view(self, rect: rl.Rectangle) -> None:
    colors = {
      'idle': rl.Color(145, 155, 149, 255),
      'driver': rl.Color(255, 190, 80, 255),
      'stop': rl.Color(255, 105, 95, 255),
      'turn': rl.Color(190, 125, 255, 255),
      'lead': rl.Color(80, 205, 255, 255),
      'model': rl.Color(70, 220, 225, 255),
      'slow': rl.Color(255, 190, 80, 255),
      'cruise': COLORS.BTN_ACTIVE,
    }
    accent = colors.get(self._intent_kind, COLORS.WHITE)
    width = min(980, rect.width - 120)
    height = 250
    # Driving intentions occupy the driver's visible left-hand position.
    box = rl.Rectangle(rect.x + 60, rect.y + 235, width, height)

    rl.draw_rectangle_rounded(box, .10, 10, COLORS.BLACK_TRANSLUCENT)
    rl.draw_rectangle_rounded_lines_ex(box, .10, 10, 4, COLORS.BORDER_TRANSLUCENT)
    rl.draw_rectangle_rounded(
      rl.Rectangle(box.x + 16, box.y + 18, 10, box.height - 36),
      .8, 8, accent,
    )

    rl.draw_text_ex(self._font_bold, self._intent_title[:24],
                    rl.Vector2(box.x + 48, box.y + 20), 58, 0, COLORS.WHITE)

    source_text = self._intent_source[:22]
    source_w = measure_text_cached(self._font_semi_bold, source_text, 28).x + 34
    source_box = rl.Rectangle(box.x + 48, box.y + 91, source_w, 44)
    rl.draw_rectangle_rounded(source_box, .45, 8, rl.Color(accent.r, accent.g, accent.b, 95))
    rl.draw_text_ex(self._font_semi_bold, source_text,
                    rl.Vector2(source_box.x + 17, source_box.y + 6), 28, 0, accent)

    rl.draw_text_ex(self._font_medium, self._intent_detail[:48],
                    rl.Vector2(box.x + 48, box.y + 147), 28, 0, COLORS.GREY)

    metrics = []
    if self._intent_target is not None:
      unit = 'KM/H' if ui_state.is_metric else 'MPH'
      metrics.append(f'PLAN {round(self._intent_target):.0f} {unit}')
    if self._intent_accel <= -0.05:
      pct = min(100, round(-self._intent_accel / 2.5 * 100))
      metrics.append(f'REGEN REQUEST {pct}%')
    elif self._intent_accel >= 0.05:
      metrics.append(f'ACCEL {self._intent_accel:.1f} m/s²')
    else:
      metrics.append('HOLDING SPEED')
    if self._intent_lead is not None and self._intent_source != 'LEAD':
      metrics.append(f'LEAD {self._intent_lead:.0f} FT')
    rl.draw_text_ex(self._font_semi_bold, '   ·   '.join(metrics)[:64],
                    rl.Vector2(box.x + 48, box.y + 199), 27, 0, COLORS.WHITE)

  def _render(self, rect: rl.Rectangle) -> None:
    self._handle_touch_input()
    footer_y = rect.y + rect.height - UI_CONFIG.header_height
    rl.draw_rectangle_gradient_v(int(rect.x), int(footer_y), int(rect.width), UI_CONFIG.header_height,
                                 COLORS.HEADER_GRADIENT_END, COLORS.HEADER_GRADIENT_START)
    if self.is_cruise_available: self._draw_set_speed(rect)
    self._draw_current_speed(rect)
    # NAP HUD: driver monitoring now lives at the top, so reclaim the
    # bottom-right corner for the experimental-mode button.
    button_x = rect.x + rect.width - UI_CONFIG.border_size - UI_CONFIG.button_size
    button_y = rect.y + rect.height - UI_CONFIG.border_size - UI_CONFIG.button_size
    self._exp_button.render(rl.Rectangle(button_x, button_y, UI_CONFIG.button_size, UI_CONFIG.button_size))
    self._draw_onscreen_controls(rect)
    self._draw_nav_view(rect)
    self._draw_intent_view(rect)

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
    # NAP HUD: controls form a visible left-side stack below driving intentions.
    base_y = rect.y + rect.height - h - 105
    
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
    x = rect.x + UI_CONFIG.border_size + UI_CONFIG.button_size + 40
    y = rect.y + rect.height - UI_CONFIG.border_size - UI_CONFIG.set_speed_height
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
    speed_y = rect.y + rect.height - 190
    speed_pos = rl.Vector2(rect.x + rect.width / 2 - speed_text_size.x / 2, speed_y - speed_text_size.y / 2)
    rl.draw_text_ex(self._font_bold, speed_text, speed_pos, FONT_SIZES.current_speed, 0, COLORS.WHITE)
    unit_text = tr("km/h") if ui_state.is_metric else tr("mph")
    unit_text_size = measure_text_cached(self._font_medium, unit_text, FONT_SIZES.speed_unit)
    unit_y = rect.y + rect.height - 80
    unit_pos = rl.Vector2(rect.x + rect.width / 2 - unit_text_size.x / 2, unit_y - unit_text_size.y / 2)
    rl.draw_text_ex(self._font_medium, unit_text, unit_pos, FONT_SIZES.speed_unit, 0, COLORS.WHITE_TRANSLUCENT)
