import unittest

from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature


class TestNAPTurnRate(unittest.TestCase):

  def test_stock_behavior_unchanged_without_override(self):
    # At 4 m/s, stock rate is 5/16 = 0.3125 curvature/s.
    # One 100 Hz control frame therefore permits ~0.003125.
    out, _ = clip_curvature(
      4.0,
      0.0,
      0.10,
      0.0,
    )

    self.assertGreater(out, 0.002)
    self.assertLess(out, 0.004)


  def test_low_speed_rate_override_turn_in(self):
    out, _ = clip_curvature(
      4.0,
      0.0,
      0.10,
      0.0,
      max_curvature_rate_override=0.80,
    )

    # 0.80 curvature/s * 0.01 s = about 0.008/frame.
    self.assertGreater(out, 0.007)
    self.assertLess(out, 0.009)


  def test_low_speed_rate_override_unwind(self):
    out, _ = clip_curvature(
      4.0,
      0.10,
      0.0,
      0.0,
      max_curvature_rate_override=1.20,
    )

    # 1.20 curvature/s * 0.01 s = about 0.012/frame.
    self.assertGreater(out, 0.087)
    self.assertLess(out, 0.089)


  def test_acceleration_limit_still_applies(self):
    # Override must NOT bypass lateral-acceleration protection.
    out, _ = clip_curvature(
      10.0,
      0.0,
      1.0,
      0.0,
      max_curvature_rate_override=100.0,
    )

    # 3 m/s^2 / 10^2 = 0.03 curvature.
    self.assertAlmostEqual(out, 0.03, places=4)


  def test_max_curvature_still_applies(self):
    out, _ = clip_curvature(
      1.0,
      0.0,
      1.0,
      0.0,
      max_curvature_rate_override=100.0,
    )

    self.assertAlmostEqual(out, 0.2, places=4)


if __name__ == "__main__":
  unittest.main()
