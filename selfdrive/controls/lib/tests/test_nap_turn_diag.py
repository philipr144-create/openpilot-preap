import math
import unittest

from openpilot.selfdrive.controls.lib.nap_turn_diag import NAPTurnDiag


class TestNAPTurnDiag(unittest.TestCase):

  def test_straight_path(self):
    x = [float(i) for i in range(50)]
    y = [0.0 for _ in x]

    k = NAPTurnDiag.curvature_from_xyz(x, y)

    self.assertIsNotNone(k)
    self.assertAlmostEqual(k, 0.0, places=5)

  def test_left_arc(self):
    # Circle radius 25 m.
    r = 25.0

    theta = [i * 0.02 for i in range(50)]

    x = [r * math.sin(t) for t in theta]
    y = [r * (1.0 - math.cos(t)) for t in theta]

    k = NAPTurnDiag.curvature_from_xyz(x, y)

    self.assertIsNotNone(k)

    # Magnitude should be approximately 1/r = 0.04.
    self.assertAlmostEqual(abs(k), 1.0 / r, delta=0.005)

  def test_right_arc(self):
    r = 20.0

    theta = [i * 0.02 for i in range(50)]

    x = [r * math.sin(t) for t in theta]
    y = [-r * (1.0 - math.cos(t)) for t in theta]

    k = NAPTurnDiag.curvature_from_xyz(x, y)

    self.assertIsNotNone(k)
    self.assertAlmostEqual(abs(k), 1.0 / r, delta=0.006)

  def test_lateral_sampling(self):
    x = [float(i) for i in range(40)]
    y = [0.1 * i for i in range(40)]

    self.assertAlmostEqual(
      NAPTurnDiag.lateral_at_distance(x, y, 10.0),
      1.0,
      places=3,
    )

    self.assertAlmostEqual(
      NAPTurnDiag.lateral_at_distance(x, y, 20.0),
      2.0,
      places=3,
    )


if __name__ == "__main__":
  unittest.main()
