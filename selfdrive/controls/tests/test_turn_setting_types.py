"""Regression coverage for Params.get returning typed booleans."""
import tempfile
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch

from cereal import car, log
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist


class TurnSettingTypes(unittest.TestCase):
  def test_real_params_boolean_contract(self):
    with tempfile.TemporaryDirectory() as directory:
      params = Params(directory)
      helper = DesireHelper()
      helper.params = params
      corner = CornerAssist()
      corner.params = params
      cs = car.CarState.new_message(vEgo=5., leftBlinker=True)
      with patch('openpilot.selfdrive.controls.lib.corner_assist.settings',
                 return_value={'corner_assist': True, 'blinker_braking': False}):
        for enabled in (True, False, True):
          params.put_bool('NAPCityTurns', enabled)
          params.put_bool('NAPCornerAssist', enabled)
          self.assertIs(params.get('NAPCityTurns'), enabled)
          helper.manual_turn_poll = 0
          helper.update(cs, True, 1.)
          self.assertIs(helper.manual_turns_enabled, enabled)
          self.assertEqual(helper.desire, log.Desire.none)
          corner.last_config = -float('inf')
          corner.refresh(100.)
          self.assertIs(corner.config['corner_assist'], enabled)
          corner.cap = None
          corner.last_time = None
          output = corner.update(0., 8., .04, -.2, True, 100.)
          self.assertLess(output, 0.) if enabled else self.assertEqual(output, 0.)

  def test_missing_corner_setting_uses_config_fallback(self):
    corner = CornerAssist()
    corner.params = N(get=lambda _: None)
    with patch('openpilot.selfdrive.controls.lib.corner_assist.settings',
               return_value={'corner_assist': True, 'blinker_braking': False}):
      corner.refresh(100.)
    self.assertTrue(corner.config['corner_assist'])


if __name__ == '__main__':
  unittest.main()
