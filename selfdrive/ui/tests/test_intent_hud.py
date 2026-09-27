import unittest

from openpilot.selfdrive.ui.onroad.hud_renderer import classify_intent


class IntentHud(unittest.TestCase):
  def state(self, **updates):
    values = dict(enabled=True, gas=False, brake=False, steering=False,
                  left_blinker=False, right_blinker=False, decision={},
                  should_stop=False, stopping=False, source='cruise',
                  accel=0., lead_distance=None)
    values.update(updates)
    return classify_intent(**values)

  def test_priority_and_longitudinal_sources(self):
    self.assertEqual(self.state(enabled=False)[0], 'ASSIST READY')
    self.assertEqual(self.state(brake=True, should_stop=True)[0], 'DRIVER BRAKING')
    self.assertEqual(self.state(should_stop=True, source='e2e', accel=-2.5)[:2],
                     ('MAX REGEN', 'MODEL STOP'))
    self.assertEqual(self.state(source='lead0', lead_distance=72.)[:2],
                     ('FOLLOWING', 'LEAD'))
    self.assertEqual(self.state(source='e2e')[:2], ('END-TO-END', 'MODEL'))
    self.assertEqual(self.state(accel=-.4)[0], 'SLOWING')
    self.assertEqual(self.state(accel=.4)[0], 'ACCELERATING')

  def test_city_turn_direction_and_geometry_wait(self):
    decision = dict(city_turns_enabled=True, manual_turn_direction=1,
                    final_desire='none')
    self.assertEqual(self.state(left_blinker=True, decision=decision)[0],
                     'CHECKING LEFT')
    decision['final_desire'] = 'turnLeft'
    self.assertEqual(self.state(left_blinker=True, decision=decision)[:2],
                     ('TURNING LEFT', 'CITY TURN'))
    # A stale/mismatched direction cannot be shown as an active City Turn.
    self.assertEqual(self.state(right_blinker=True, decision=decision)[0],
                     'CRUISING')


if __name__ == '__main__':
  unittest.main()
