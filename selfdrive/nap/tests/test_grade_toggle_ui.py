"""V9e UI switch tests; all use isolated temp files, never vehicle config."""
import json
from pathlib import Path
import tempfile
import unittest

from openpilot.selfdrive.nap.grade_toggle_ui import read_grade_enabled, write_grade_enabled


class GradeToggleUiTests(unittest.TestCase):
  def setUp(self):
    self.dir = tempfile.TemporaryDirectory()
    self.addCleanup(self.dir.cleanup)
    self.path = Path(self.dir.name) / 'v9e_predictive_grade.json'

  def install_off(self):
    self.path.write_text('{"enabled": false}\n')

  def test_fail_closed_if_missing_bad_or_symlinked(self):
    self.assertFalse(read_grade_enabled(self.path))
    self.assertFalse(write_grade_enabled(True, offroad=True, path=self.path, process_checker=lambda: False))
    self.path.write_text('{not json')
    self.assertFalse(read_grade_enabled(self.path))
    self.path.unlink()
    target = Path(self.dir.name) / 'target.json'
    target.write_text('{"enabled": true}\n')
    self.path.symlink_to(target)
    self.assertFalse(read_grade_enabled(self.path))
    self.assertFalse(write_grade_enabled(False, offroad=True, path=self.path))
    self.assertEqual(json.loads(target.read_text())['enabled'], True)

  def test_enable_requires_offroad_and_process_clear(self):
    self.install_off()
    self.assertFalse(write_grade_enabled(True, offroad=False, path=self.path, process_checker=lambda: False))
    self.assertFalse(write_grade_enabled(True, offroad=True, path=self.path, process_checker=lambda: True))
    self.assertFalse(read_grade_enabled(self.path))
    self.assertTrue(write_grade_enabled(True, offroad=True, path=self.path, process_checker=lambda: False))
    self.assertTrue(read_grade_enabled(self.path))

  def test_disable_allowed_while_onroad(self):
    self.install_off()
    self.assertTrue(write_grade_enabled(True, offroad=True, path=self.path, process_checker=lambda: False))
    self.assertTrue(write_grade_enabled(False, offroad=False, path=self.path))
    self.assertFalse(read_grade_enabled(self.path))

  def test_reject_nonboolean_and_preserve_state(self):
    self.install_off()
    self.assertFalse(write_grade_enabled(1, offroad=True, path=self.path, process_checker=lambda: False))
    self.assertFalse(read_grade_enabled(self.path))


if __name__ == '__main__':
  unittest.main()
