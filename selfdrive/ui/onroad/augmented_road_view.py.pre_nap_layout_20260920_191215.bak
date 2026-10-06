import time
import json
import threading
from pathlib import Path
from urllib.request import Request, urlopen
import numpy as np
import pyray as rl
from cereal import log, messaging
from msgq.visionipc import VisionStreamType
from openpilot.selfdrive.ui import UI_BORDER_SIZE
from openpilot.selfdrive.ui.ui_state import ui_state, UIStatus
from openpilot.selfdrive.ui.onroad.alert_renderer import AlertRenderer
from openpilot.selfdrive.ui.onroad.driver_state import DriverStateRenderer
from openpilot.selfdrive.ui.onroad.hud_renderer import HudRenderer
from openpilot.selfdrive.ui.onroad.model_renderer import ModelRenderer
from openpilot.selfdrive.ui.onroad.cameraview import CameraView
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.widgets.label import gui_label
from openpilot.common.transformations.camera import DEVICE_CAMERAS, DeviceCameraConfig, view_frame_from_device_frame
from openpilot.common.transformations.orientation import rot_from_euler

OpState = log.SelfdriveState.OpenpilotState
CALIBRATED = log.LiveCalibrationData.Status.calibrated
ROAD_CAM = VisionStreamType.VISION_STREAM_ROAD
WIDE_CAM = VisionStreamType.VISION_STREAM_WIDE_ROAD
DEFAULT_DEVICE_CAMERA = DEVICE_CAMERAS["tici", "ar0231"]

BORDER_COLORS = {
  UIStatus.DISENGAGED: rl.Color(0x12, 0x28, 0x39, 0xFF),  # Blue for disengaged state
  UIStatus.OVERRIDE: rl.Color(0x89, 0x92, 0x8D, 0xFF),  # Gray for override state
  UIStatus.ENGAGED: rl.Color(0x16, 0x7F, 0x40, 0xFF),  # Green for engaged state
}

WIDE_CAM_MAX_SPEED = 10.0  # m/s (22 mph)
ROAD_CAM_MIN_SPEED = 15.0  # m/s (34 mph)
INF_POINT = np.array([1000.0, 0.0, 0.0])
PRESETS_PATH = Path('/data/nap_destination_presets.json')
PAIRING_KEY_PATH = Path('/data/openpilot/server/phone_navigation/.pairing-key')


class AugmentedRoadView(CameraView):
  def __init__(self, stream_type: VisionStreamType = VisionStreamType.VISION_STREAM_ROAD):
    super().__init__("camerad", stream_type)
    self._set_placeholder_color(BORDER_COLORS[UIStatus.DISENGAGED])

    self.device_camera: DeviceCameraConfig | None = None
    self.view_from_calib = view_frame_from_device_frame.copy()
    self.view_from_wide_calib = view_frame_from_device_frame.copy()

    self._matrix_cache_key = (0, 0.0, 0.0, stream_type)
    self._cached_matrix: np.ndarray | None = None
    self._content_rect = rl.Rectangle()

    self.model_renderer = ModelRenderer()
    self._hud_renderer = HudRenderer()
    self.alert_renderer = AlertRenderer()
    self.driver_state_renderer = DriverStateRenderer()
    self._route_button_rect = rl.Rectangle(0, 0, 0, 0)
    self._route_rows = []
    self._route_modal_rect = rl.Rectangle(0, 0, 0, 0)
    self._route_picker_open = False
    self._route_presets = []
    self._route_message = ''
    self._route_busy = False

    # debug
    self._pm = messaging.PubMaster(['uiDebug'])

  def _render(self, rect):
    # Only render when system is started to avoid invalid data access
    start_draw = time.monotonic()
    if not ui_state.started:
      return

    self._switch_stream_if_needed(ui_state.sm)

    # Update calibration before rendering
    self._update_calibration()

    # Create inner content area with border padding
    self._content_rect = rl.Rectangle(
      rect.x + UI_BORDER_SIZE,
      rect.y + UI_BORDER_SIZE,
      rect.width - 2 * UI_BORDER_SIZE,
      rect.height - 2 * UI_BORDER_SIZE,
    )

    # Enable scissor mode to clip all rendering within content rectangle boundaries
    # This creates a rendering viewport that prevents graphics from drawing outside the border
    rl.begin_scissor_mode(
      int(self._content_rect.x),
      int(self._content_rect.y),
      int(self._content_rect.width),
      int(self._content_rect.height)
    )

    # Render the base camera view
    super()._render(rect)

    # Draw all UI overlays
    self.model_renderer.render(self._content_rect)
    self._hud_renderer.render(self._content_rect)
    self.alert_renderer.render(self._content_rect)
    self.driver_state_renderer.render(self._content_rect)
    if self._parked_for_routes():
      self._render_route_picker()
    else:
      self._route_picker_open = False
      self._route_rows = []

    # Custom UI extension point - add custom overlays here
    # Use self._content_rect for positioning within camera bounds

    # End clipping region
    rl.end_scissor_mode()

    # Draw colored border based on driving state
    self._draw_border(rect)

    # publish uiDebug
    msg = messaging.new_message('uiDebug')
    msg.uiDebug.drawTimeMillis = (time.monotonic() - start_draw) * 1000
    self._pm.send('uiDebug', msg)

  def _parked_for_routes(self):
    sm = ui_state.sm
    return (ui_state.started and sm.valid['carState'] and sm.alive['carState']
            and sm.valid['carControl'] and sm.alive['carControl']
            and sm.recv_frame['carState'] >= ui_state.started_frame
            and str(sm['carState'].gearShifter) == 'park'
            and sm['carState'].canValid and abs(sm['carState'].vEgo) < .1
            and not sm['carControl'].enabled and not sm['carControl'].latActive
            and not sm['carControl'].longActive)

  def _load_route_presets(self):
    try:
      data = json.loads(PRESETS_PATH.read_text())
      self._route_presets = data.get('presets', [])[:6] if isinstance(data, dict) else []
    except (OSError, ValueError):
      self._route_presets = []

  def _start_preset(self, preset):
    if self._route_busy or not self._parked_for_routes():
      return
    self._route_busy = True
    self._route_message = 'Sending destination…'

    def work():
      try:
        key = PAIRING_KEY_PATH.read_text().strip()
        body = json.dumps({'destination': {'label': preset['label'], 'location': preset['location']}}).encode()
        request = Request('http://127.0.0.1:7070/phone-nav/api/start', data=body,
                          headers={'Content-Type': 'application/json', 'X-NAP-Phone-Key': key})
        with urlopen(request, timeout=4) as response:
          result = json.load(response)
        self._route_message = 'Destination set. Route will calculate with GPS and internet.' if result.get('active') else 'Navigation did not start.'
      except Exception:
        self._route_message = 'Navigation service unavailable. Try again when connected.'
      finally:
        self._route_busy = False

    threading.Thread(target=work, name='nap-parked-route', daemon=True).start()

  def _render_route_picker(self):
    rect = self._content_rect
    self._route_button_rect = rl.Rectangle(rect.x + 40, rect.y + rect.height - 125, 450, 90)
    rl.draw_rectangle_rounded(self._route_button_rect, .18, 10, rl.Color(54, 77, 239, 240))
    gui_label(rl.Rectangle(self._route_button_rect.x + 24, self._route_button_rect.y,
                           self._route_button_rect.width - 48, self._route_button_rect.height),
              'SAVED DESTINATIONS', 31, rl.WHITE, font_weight=FontWeight.BOLD)
    if not self._route_picker_open:
      return
    width = min(1250, rect.width - 120)
    height = min(820, rect.height - 130)
    panel = rl.Rectangle(rect.x + (rect.width-width)/2, rect.y + (rect.height-height)/2, width, height)
    self._route_modal_rect = panel
    rl.draw_rectangle_rounded(panel, .035, 12, rl.Color(17, 27, 41, 250))
    title = rl.Rectangle(panel.x + 48, panel.y + 25, panel.width - 96, 70)
    gui_label(title, 'Saved destinations · tap a place', 43, rl.WHITE, font_weight=FontWeight.BOLD)
    self._route_rows = []
    for index, preset in enumerate(self._route_presets):
      row = rl.Rectangle(panel.x + 45, panel.y + 105 + index*98, panel.width - 90, 84)
      self._route_rows.append(row)
      rl.draw_rectangle_rounded(row, .1, 10, rl.Color(67, 88, 126, 255))
      label = rl.Rectangle(row.x + 24, row.y, row.width - 48, row.height)
      gui_label(label, str(preset.get('name', 'Destination'))[:40], 38, rl.WHITE)
    if not self._route_presets:
      empty = rl.Rectangle(panel.x + 50, panel.y + 130, panel.width - 100, 100)
      gui_label(empty, 'No places saved. Add them from the navigation page.', 30, rl.WHITE)
    note = rl.Rectangle(panel.x + 45, panel.y + panel.height - 88, panel.width - 90, 70)
    gui_label(note, self._route_message or 'Select while parked. New routes need internet.', 27, rl.WHITE)

  def _handle_mouse_press(self, mouse_pos):
    if self._parked_for_routes():
      if self._route_picker_open:
        if not self._route_busy:
          for index, row in enumerate(self._route_rows):
            if rl.check_collision_point_rec(mouse_pos, row) and index < len(self._route_presets):
              self._start_preset(self._route_presets[index])
              return
        if not rl.check_collision_point_rec(mouse_pos, self._route_modal_rect):
          self._route_picker_open = False
        return
      if rl.check_collision_point_rec(mouse_pos, self._route_button_rect):
        self._load_route_presets()
        self._route_picker_open = True
        return
    if not self._hud_renderer.user_interacting() and self._click_callback is not None:
      self._click_callback()

  def _handle_mouse_release(self, _):
    # We only call click callback on press if not interacting with HUD
    pass

  def _draw_border(self, rect: rl.Rectangle):
    rl.draw_rectangle_lines_ex(rect, UI_BORDER_SIZE, rl.BLACK)
    border_roundness = 0.12
    border_color = BORDER_COLORS.get(ui_state.status, BORDER_COLORS[UIStatus.DISENGAGED])
    border_rect = rl.Rectangle(rect.x + UI_BORDER_SIZE, rect.y + UI_BORDER_SIZE,
                               rect.width - 2 * UI_BORDER_SIZE, rect.height - 2 * UI_BORDER_SIZE)
    rl.draw_rectangle_rounded_lines_ex(border_rect, border_roundness, 10, UI_BORDER_SIZE, border_color)

  def _switch_stream_if_needed(self, sm):
    if sm['selfdriveState'].experimentalMode and WIDE_CAM in self.available_streams:
      v_ego = sm['carState'].vEgo
      if v_ego < WIDE_CAM_MAX_SPEED:
        target = WIDE_CAM
      elif v_ego > ROAD_CAM_MIN_SPEED:
        target = ROAD_CAM
      else:
        # Hysteresis zone - keep current stream
        target = self.stream_type
    else:
      target = ROAD_CAM

    if self.stream_type != target:
      self.switch_stream(target)

  def _update_calibration(self):
    # Update device camera if not already set
    sm = ui_state.sm
    if not self.device_camera and sm.seen['roadCameraState'] and sm.seen['deviceState']:
      self.device_camera = DEVICE_CAMERAS[(str(sm['deviceState'].deviceType), str(sm['roadCameraState'].sensor))]

    # Check if live calibration data is available and valid
    if not (sm.updated["liveCalibration"] and sm.valid['liveCalibration']):
      return

    calib = sm['liveCalibration']
    if len(calib.rpyCalib) != 3 or calib.calStatus != CALIBRATED:
      return

    # Update view_from_calib matrix
    device_from_calib = rot_from_euler(calib.rpyCalib)
    self.view_from_calib = view_frame_from_device_frame @ device_from_calib

    # Update wide calibration if available
    if hasattr(calib, 'wideFromDeviceEuler') and len(calib.wideFromDeviceEuler) == 3:
      wide_from_device = rot_from_euler(calib.wideFromDeviceEuler)
      self.view_from_wide_calib = view_frame_from_device_frame @ wide_from_device @ device_from_calib

  def _calc_frame_matrix(self, rect: rl.Rectangle) -> np.ndarray:
    # Check if we can use cached matrix
    cache_key = (
      ui_state.sm.recv_frame['liveCalibration'],
      self._content_rect.width,
      self._content_rect.height,
      self.stream_type
    )
    if cache_key == self._matrix_cache_key and self._cached_matrix is not None:
      return self._cached_matrix

    # Get camera configuration
    device_camera = self.device_camera or DEFAULT_DEVICE_CAMERA
    is_wide_camera = self.stream_type == WIDE_CAM
    intrinsic = device_camera.ecam.intrinsics if is_wide_camera else device_camera.fcam.intrinsics
    calibration = self.view_from_wide_calib if is_wide_camera else self.view_from_calib
    zoom = 2.0 if is_wide_camera else 1.1

    # Calculate transforms for vanishing point
    calib_transform = intrinsic @ calibration
    kep = calib_transform @ INF_POINT

    # Calculate center points and dimensions
    x, y = self._content_rect.x, self._content_rect.y
    w, h = self._content_rect.width, self._content_rect.height
    cx, cy = intrinsic[0, 2], intrinsic[1, 2]

    # Calculate max allowed offsets with margins
    margin = 5
    max_x_offset = cx * zoom - w / 2 - margin
    max_y_offset = cy * zoom - h / 2 - margin

    # Calculate and clamp offsets to prevent out-of-bounds issues
    try:
      if abs(kep[2]) > 1e-6:
        x_offset = np.clip((kep[0] / kep[2] - cx) * zoom, -max_x_offset, max_x_offset)
        y_offset = np.clip((kep[1] / kep[2] - cy) * zoom, -max_y_offset, max_y_offset)
      else:
        x_offset, y_offset = 0, 0
    except (ZeroDivisionError, OverflowError):
      x_offset, y_offset = 0, 0

    # Cache the computed transformation matrix to avoid recalculations
    self._matrix_cache_key = cache_key
    self._cached_matrix = np.array([
      [zoom * 2 * cx / w, 0, -x_offset / w * 2],
      [0, zoom * 2 * cy / h, -y_offset / h * 2],
      [0, 0, 1.0]
    ])

    video_transform = np.array([
      [zoom, 0.0, (w / 2 + x - x_offset) - (cx * zoom)],
      [0.0, zoom, (h / 2 + y - y_offset) - (cy * zoom)],
      [0.0, 0.0, 1.0]
    ])
    self.model_renderer.set_transform(video_transform @ calib_transform)

    return self._cached_matrix


if __name__ == "__main__":
  gui_app.init_window("OnRoad Camera View")
  road_camera_view = AugmentedRoadView(ROAD_CAM)
  gui_app.push_widget(road_camera_view)
  print("***press space to switch camera view***")
  try:
    for _ in gui_app.render():
      ui_state.update()
      if rl.is_key_released(rl.KeyboardKey.KEY_SPACE):
        if WIDE_CAM in road_camera_view.available_streams:
          stream = ROAD_CAM if road_camera_view.stream_type == WIDE_CAM else WIDE_CAM
          road_camera_view.switch_stream(stream)
  finally:
    road_camera_view.close()
