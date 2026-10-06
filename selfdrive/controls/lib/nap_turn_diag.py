#!/usr/bin/env python3

import math
import time

try:
  from openpilot.common.swaglog import cloudlog
except Exception:
  cloudlog = None


class NAPTurnDiag:
  """
  Diagnostic-only helper.

  Collects snapshots of:
    - vehicle speed
    - nav desire
    - model path curvature estimate
    - planner desired curvature
    - committed/output curvature
    - steering angle

  It does NOT modify any control values.
  """

  def __init__(self):
    self.last_log_t = 0.0
    self.log_interval = 0.10   # 10 Hz while turning
    self.seq = 0

    # Temporary diagnostic capture. Line-buffered so a reboot does not lose
    # the useful part of a drive.
    self._diag_file = None
    try:
      self._diag_file = open("/data/nap_turn_diag.log", "a", buffering=1)
      self._diag_file.write("\n===== NAP TURN DIAG SESSION =====\n")
    except Exception:
      self._diag_file = None

  @staticmethod
  def curvature_from_xyz(x, y, max_points=33):
    """
    Estimate signed curvature from model position x/y.

    Uses 3-point circumcircle geometry over several points and returns
    the median-ish central value.

    Positive/negative sign follows lateral y direction.
    """

    try:
      n = min(len(x), len(y), max_points)
    except Exception:
      return None

    if n < 7:
      return None

    vals = []

    # Skip the very nearest points because they can be noisy.
    for i in range(3, n - 3, 3):
      i0 = max(0, i - 3)
      i1 = i
      i2 = min(n - 1, i + 3)

      x1, y1 = float(x[i0]), float(y[i0])
      x2, y2 = float(x[i1]), float(y[i1])
      x3, y3 = float(x[i2]), float(y[i2])

      a = math.hypot(x2 - x1, y2 - y1)
      b = math.hypot(x3 - x2, y3 - y2)
      c = math.hypot(x3 - x1, y3 - y1)

      denom = a * b * c
      if denom < 1e-6:
        continue

      cross = ((x2 - x1) * (y3 - y1)) - ((y2 - y1) * (x3 - x1))

      # k = 4*area/(abc), where cross = 2*area
      k = (2.0 * cross) / denom

      if math.isfinite(k) and abs(k) < 1.0:
        vals.append(k)

    if not vals:
      return None

    vals.sort()
    return vals[len(vals) // 2]

  @staticmethod
  def lateral_at_distance(x, y, target_x):
    """
    Approximate model path lateral offset at a forward distance.
    """
    try:
      if len(x) == 0 or len(y) == 0:
        return None

      best_i = min(
        range(min(len(x), len(y))),
        key=lambda i: abs(float(x[i]) - target_x)
      )

      return float(y[best_i])

    except Exception:
      return None

  def should_log(self, active=True):
    if not active:
      return False

    now = time.monotonic()

    if now - self.last_log_t < self.log_interval:
      return False

    self.last_log_t = now
    return True

  def log(self,
          *,
          speed_mps=None,
          nav_desire=None,
          maneuver=None,
          distance_to_maneuver=None,
          model_x=None,
          model_y=None,
          planner_curvature=None,
          committed_curvature=None,
          output_curvature=None,
          steering_angle_deg=None,
          lat_active=None,
          note=""):

    model_k = None
    y10 = None
    y20 = None
    y30 = None

    if model_x is not None and model_y is not None:
      model_k = self.curvature_from_xyz(model_x, model_y)
      y10 = self.lateral_at_distance(model_x, model_y, 10.0)
      y20 = self.lateral_at_distance(model_x, model_y, 20.0)
      y30 = self.lateral_at_distance(model_x, model_y, 30.0)

    self.seq += 1

    msg = (
      "NAP_TURN_DIAG "
      f"seq={self.seq} "
      f"v={self._f(speed_mps, 2)} "
      f"nav={nav_desire} "
      f"maneuver={maneuver} "
      f"dist={self._f(distance_to_maneuver, 1)} "
      f"model_k={self._f(model_k, 5)} "
      f"y10={self._f(y10, 2)} "
      f"y20={self._f(y20, 2)} "
      f"y30={self._f(y30, 2)} "
      f"planner_k={self._f(planner_curvature, 5)} "
      f"commit_k={self._f(committed_curvature, 5)} "
      f"output_k={self._f(output_curvature, 5)} "
      f"steer_deg={self._f(steering_angle_deg, 2)} "
      f"latActive={lat_active} "
      f"note={note}"
    )

    print(msg, flush=True)

    if self._diag_file is not None:
      try:
        self._diag_file.write(msg + "\n")
      except Exception:
        pass

    if cloudlog is not None:
      try:
        cloudlog.info(msg)
      except Exception:
        pass

  @staticmethod
  def _f(v, digits):
    try:
      if v is None:
        return "NA"
      v = float(v)
      if not math.isfinite(v):
        return "NA"
      return f"{v:.{digits}f}"
    except Exception:
      return "NA"
