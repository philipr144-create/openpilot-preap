import time
import json
import threading
from pathlib import Path
from urllib.request import Request, urlopen
import pyray as rl
from collections.abc import Callable
from enum import IntEnum
from openpilot.common.params import Params
from openpilot.selfdrive.ui.widgets.offroad_alerts import UpdateAlert, OffroadAlert
from openpilot.selfdrive.ui.widgets.exp_mode_button import ExperimentalModeButton
from openpilot.selfdrive.ui.widgets.prime import PrimeWidget
from openpilot.selfdrive.ui.widgets.setup import SetupWidget
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.lib.application import gui_app, FontWeight, MousePos
from openpilot.system.ui.lib.multilang import tr, trn
from openpilot.system.ui.widgets.label import gui_label
from openpilot.system.ui.widgets import Widget

HEADER_HEIGHT = 80
HEAD_BUTTON_FONT_SIZE = 40
CONTENT_MARGIN = 40
SPACING = 25
RIGHT_COLUMN_WIDTH = 750
REFRESH_INTERVAL = 10.0
PRESETS_PATH = Path('/data/nap_destination_presets.json')
PAIRING_KEY_PATH = Path('/data/openpilot/server/phone_navigation/.pairing-key')


class HomeLayoutState(IntEnum):
  HOME = 0
  UPDATE = 1
  ALERTS = 2
  ROUTES = 3


class HomeLayout(Widget):
  def __init__(self):
    super().__init__()
    self.params = Params()

    self.update_alert = UpdateAlert()
    self.offroad_alert = OffroadAlert()

    self._layout_widgets = {HomeLayoutState.UPDATE: self.update_alert, HomeLayoutState.ALERTS: self.offroad_alert}

    self.current_state = HomeLayoutState.HOME
    self.last_refresh = 0
    self.settings_callback: Callable[[], None] | None = None

    self.update_available = False
    self.alert_count = 0
    self._version_text = ""
    self._prev_update_available = False
    self._prev_alerts_present = False

    self.header_rect = rl.Rectangle(0, 0, 0, 0)
    self.content_rect = rl.Rectangle(0, 0, 0, 0)
    self.left_column_rect = rl.Rectangle(0, 0, 0, 0)
    self.right_column_rect = rl.Rectangle(0, 0, 0, 0)

    self.update_notif_rect = rl.Rectangle(0, 0, 200, HEADER_HEIGHT - 10)
    self.alert_notif_rect = rl.Rectangle(0, 0, 220, HEADER_HEIGHT - 10)
    self.routes_rect = rl.Rectangle(0, 0, 220, HEADER_HEIGHT - 10)
    self.route_rows = []
    self.route_presets = []
    self.route_message = ''
    self.route_busy = False

    self._prime_widget = PrimeWidget()
    self._setup_widget = SetupWidget()

    self._exp_mode_button = ExperimentalModeButton()
    self._setup_callbacks()

  def show_event(self):
    super().show_event()
    self._exp_mode_button.show_event()
    self.last_refresh = time.monotonic()
    self._refresh()
    self._load_routes()

  def _load_routes(self):
    try:
      data = json.loads(PRESETS_PATH.read_text())
      self.route_presets = data.get('presets', [])[:6] if isinstance(data, dict) else []
    except (OSError, ValueError):
      self.route_presets = []

  def _start_route(self, preset):
    if self.route_busy:
      return
    self.route_busy = True
    self.route_message = 'Sending destination…'

    def work():
      try:
        key = PAIRING_KEY_PATH.read_text().strip()
        body = json.dumps({'destination': {'label': preset['label'], 'location': preset['location']}}).encode()
        request = Request('http://127.0.0.1:7070/phone-nav/api/start', data=body,
                          headers={'Content-Type': 'application/json', 'X-NAP-Phone-Key': key})
        with urlopen(request, timeout=4) as response:
          result = json.load(response)
        self.route_message = 'Destination set. Route will calculate when GPS and internet are ready.' if result.get('active') else 'Route service did not start.'
      except Exception:
        self.route_message = 'Could not reach navigation service. Check the comma connection.'
      finally:
        self.route_busy = False

    threading.Thread(target=work, name='nap-preset-route', daemon=True).start()

  def _setup_callbacks(self):
    self.update_alert.set_dismiss_callback(lambda: self._set_state(HomeLayoutState.HOME))
    self.offroad_alert.set_dismiss_callback(lambda: self._set_state(HomeLayoutState.HOME))
    self._exp_mode_button.set_click_callback(lambda: self.settings_callback() if self.settings_callback else None)

  def set_settings_callback(self, callback: Callable):
    self.settings_callback = callback

  def _set_state(self, state: HomeLayoutState):
    # propagate show/hide events
    if state != self.current_state:
      if state == HomeLayoutState.HOME:
        self._exp_mode_button.show_event()

      if state in self._layout_widgets:
        self._layout_widgets[state].show_event()
      if self.current_state in self._layout_widgets:
        self._layout_widgets[self.current_state].hide_event()

    self.current_state = state

  def _render(self, rect: rl.Rectangle):
    current_time = time.monotonic()
    if current_time - self.last_refresh >= REFRESH_INTERVAL:
      self._refresh()
      self.last_refresh = current_time

    self._render_header()

    # Render content based on current state
    if self.current_state == HomeLayoutState.HOME:
      self._render_home_content()
    elif self.current_state == HomeLayoutState.UPDATE:
      self._render_update_view()
    elif self.current_state == HomeLayoutState.ALERTS:
      self._render_alerts_view()
    elif self.current_state == HomeLayoutState.ROUTES:
      self._render_routes_view()

  def _update_state(self):
    self.header_rect = rl.Rectangle(
      self._rect.x + CONTENT_MARGIN, self._rect.y + CONTENT_MARGIN, self._rect.width - 2 * CONTENT_MARGIN, HEADER_HEIGHT
    )

    content_y = self._rect.y + CONTENT_MARGIN + HEADER_HEIGHT + SPACING
    content_height = self._rect.height - CONTENT_MARGIN - HEADER_HEIGHT - SPACING - CONTENT_MARGIN

    self.content_rect = rl.Rectangle(
      self._rect.x + CONTENT_MARGIN, content_y, self._rect.width - 2 * CONTENT_MARGIN, content_height
    )

    left_width = self.content_rect.width - RIGHT_COLUMN_WIDTH - SPACING

    self.left_column_rect = rl.Rectangle(self.content_rect.x, self.content_rect.y, left_width, self.content_rect.height)

    self.right_column_rect = rl.Rectangle(
      self.content_rect.x + left_width + SPACING, self.content_rect.y, RIGHT_COLUMN_WIDTH, self.content_rect.height
    )

    self.update_notif_rect.x = self.header_rect.x
    self.update_notif_rect.y = self.header_rect.y + (self.header_rect.height - 60) // 2

    notif_x = self.header_rect.x + (220 if self.update_available else 0)
    self.alert_notif_rect.x = notif_x
    self.alert_notif_rect.y = self.header_rect.y + (self.header_rect.height - 60) // 2
    self.routes_rect.x = self.header_rect.x + self.header_rect.width - self.routes_rect.width - 420
    self.routes_rect.y = self.header_rect.y + (self.header_rect.height - 60) // 2

  def _handle_mouse_release(self, mouse_pos: MousePos):
    super()._handle_mouse_release(mouse_pos)

    if self.update_available and rl.check_collision_point_rec(mouse_pos, self.update_notif_rect):
      self._set_state(HomeLayoutState.UPDATE)
    elif self.alert_count > 0 and rl.check_collision_point_rec(mouse_pos, self.alert_notif_rect):
      self._set_state(HomeLayoutState.ALERTS)
    elif rl.check_collision_point_rec(mouse_pos, self.routes_rect):
      self._load_routes()
      self._set_state(HomeLayoutState.HOME if self.current_state == HomeLayoutState.ROUTES else HomeLayoutState.ROUTES)
    elif self.current_state == HomeLayoutState.ROUTES and not self.route_busy:
      for index, row in enumerate(self.route_rows):
        if rl.check_collision_point_rec(mouse_pos, row) and index < len(self.route_presets):
          self._start_route(self.route_presets[index])
          break

  def _render_header(self):
    font = gui_app.font(FontWeight.MEDIUM)
    rl.draw_rectangle_rounded(self.routes_rect, 0.3, 10, rl.Color(75, 95, 255, 255) if self.current_state == HomeLayoutState.ROUTES else rl.Color(54, 77, 239, 255))
    gui_label(self.routes_rect, 'ROUTES', 34, rl.WHITE)

    version_text_width = 400

    # Update notification button
    if self.update_available:
      version_text_width -= self.update_notif_rect.width

      # Highlight if currently viewing updates
      highlight_color = rl.Color(75, 95, 255, 255) if self.current_state == HomeLayoutState.UPDATE else rl.Color(54, 77, 239, 255)
      rl.draw_rectangle_rounded(self.update_notif_rect, 0.3, 10, highlight_color)

      text = tr("UPDATE")
      text_size = measure_text_cached(font, text, HEAD_BUTTON_FONT_SIZE)
      text_x = self.update_notif_rect.x + (self.update_notif_rect.width - text_size.x) // 2
      text_y = self.update_notif_rect.y + (self.update_notif_rect.height - text_size.y) // 2
      rl.draw_text_ex(font, text, rl.Vector2(int(text_x), int(text_y)), HEAD_BUTTON_FONT_SIZE, 0, rl.WHITE)

    # Alert notification button
    if self.alert_count > 0:
      version_text_width -= self.alert_notif_rect.width

      # Highlight if currently viewing alerts
      highlight_color = rl.Color(255, 70, 70, 255) if self.current_state == HomeLayoutState.ALERTS else rl.Color(226, 44, 44, 255)
      rl.draw_rectangle_rounded(self.alert_notif_rect, 0.3, 10, highlight_color)

      alert_text = trn("{} ALERT", "{} ALERTS", self.alert_count).format(self.alert_count)
      text_size = measure_text_cached(font, alert_text, HEAD_BUTTON_FONT_SIZE)
      text_x = self.alert_notif_rect.x + (self.alert_notif_rect.width - text_size.x) // 2
      text_y = self.alert_notif_rect.y + (self.alert_notif_rect.height - text_size.y) // 2
      rl.draw_text_ex(font, alert_text, rl.Vector2(int(text_x), int(text_y)), HEAD_BUTTON_FONT_SIZE, 0, rl.WHITE)

    # Version text (right aligned)
    if self.update_available or self.alert_count > 0:
      version_text_width -= SPACING * 1.5

    version_rect = rl.Rectangle(self.header_rect.x + self.header_rect.width - version_text_width, self.header_rect.y,
                                version_text_width, self.header_rect.height)
    gui_label(version_rect, self._version_text, 48, rl.WHITE, alignment=rl.GuiTextAlignment.TEXT_ALIGN_RIGHT)

  def _render_home_content(self):
    self._render_left_column()
    self._render_right_column()

  def _render_update_view(self):
    self.update_alert.render(self.content_rect)

  def _render_alerts_view(self):
    self.offroad_alert.render(self.content_rect)

  def _render_routes_view(self):
    self.route_rows = []
    title = rl.Rectangle(self.content_rect.x, self.content_rect.y, self.content_rect.width, 74)
    gui_label(title, 'Saved destinations', 48, rl.WHITE)
    if not self.route_presets:
      hint = rl.Rectangle(self.content_rect.x, self.content_rect.y + 110, self.content_rect.width, 100)
      gui_label(hint, 'No presets yet. Save a destination from the navigation page.', 32, rl.WHITE)
    for index, preset in enumerate(self.route_presets):
      row = rl.Rectangle(self.content_rect.x, self.content_rect.y + 110 + index * 105, self.content_rect.width, 88)
      self.route_rows.append(row)
      rl.draw_rectangle_rounded(row, 0.12, 10, rl.Color(40, 47, 59, 255))
      gui_label(row, str(preset.get('name', 'Destination'))[:48], 38, rl.WHITE)
    note = rl.Rectangle(self.content_rect.x, self.content_rect.y + self.content_rect.height - 88, self.content_rect.width, 80)
    gui_label(note, self.route_message or 'Tap a destination. New routes need internet; GPS tracks a loaded route locally.', 27, rl.WHITE)

  def _render_left_column(self):
    self._prime_widget.render(self.left_column_rect)

  def _render_right_column(self):
    exp_height = 125
    exp_rect = rl.Rectangle(
      self.right_column_rect.x, self.right_column_rect.y, self.right_column_rect.width, exp_height
    )
    self._exp_mode_button.render(exp_rect)

    setup_rect = rl.Rectangle(
      self.right_column_rect.x,
      self.right_column_rect.y + exp_height + SPACING,
      self.right_column_rect.width,
      self.right_column_rect.height - exp_height - SPACING,
    )
    self._setup_widget.render(setup_rect)

  def _refresh(self):
    self._version_text = self._get_version_text()
    update_available = self.update_alert.refresh()
    alert_count = self.offroad_alert.refresh()
    alerts_present = alert_count > 0

    # Show panels on transition from no alert/update to any alerts/update
    if not update_available and not alerts_present and self.current_state != HomeLayoutState.ROUTES:
      self._set_state(HomeLayoutState.HOME)
    elif update_available and ((not self._prev_update_available) or (not alerts_present and self.current_state == HomeLayoutState.ALERTS)):
      self._set_state(HomeLayoutState.UPDATE)
    elif alerts_present and ((not self._prev_alerts_present) or (not update_available and self.current_state == HomeLayoutState.UPDATE)):
      self._set_state(HomeLayoutState.ALERTS)

    self.update_available = update_available
    self.alert_count = alert_count
    self._prev_update_available = update_available
    self._prev_alerts_present = alerts_present

  def _get_version_text(self) -> str:
    brand = "openpilot"
    description = self.params.get("UpdaterCurrentDescription")
    return f"{brand} {description}" if description else brand
