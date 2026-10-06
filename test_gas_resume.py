import unittest
import sys
from pathlib import Path

# Ensure openpilot is in the path
sys.path.append('/data/openpilot')
from opendbc.car.tesla.preap.engagement import PreAPEngagement, RESUME_MAX_SPEED_MS

class TestGasResume(unittest.TestCase):
    def setUp(self):
        # Instantiate without running __init__ to avoid CAN dependencies
        self.eng = PreAPEngagement.__new__(PreAPEngagement)
        self.eng.resume_prev_gas_pressed = False
        self.eng.cruiseEnabled = True
        self.eng.enableLongControl = False
        self.eng.enableJustCC = True
        self.eng.resume_target_kph = 45.0
        self.eng.pedal_speed_kph = 0.0
        self.eng.longCtrlEvent = None
        self.eng.clear_brake_resume = lambda: None # mock the clear function

    def test_standard_standstill_resume(self):
        # Valid gas tap at 0 mph
        res = self.eng.process_brake_resume(
            curr_time_ms=1000, v_ego=0.0, real_brake_pressed=False,
            gas_pressed=True, pedal_available=True, pedal_long_allowed=True,
            can_engage=True, enabled=True
        )
        self.assertTrue(res)
        self.assertTrue(self.eng.enableLongControl)
        self.assertFalse(self.eng.enableJustCC)
        self.assertEqual(self.eng.pedal_speed_kph, 45.0)
        self.assertEqual(self.eng.longCtrlEvent, 'pccEnabled')

    def test_speed_limit_blocks_resume(self):
        # Attempt to resume while already rolling above threshold
        res = self.eng.process_brake_resume(
            curr_time_ms=1000, v_ego=RESUME_MAX_SPEED_MS + 1.0, real_brake_pressed=False,
            gas_pressed=True, pedal_available=True, pedal_long_allowed=True,
            can_engage=True, enabled=True
        )
        self.assertFalse(res)
        self.assertFalse(self.eng.enableLongControl)

    def test_brake_blocks_resume(self):
        # Driver is stepping on the brake and gas simultaneously
        res = self.eng.process_brake_resume(
            curr_time_ms=1000, v_ego=0.0, real_brake_pressed=True,
            gas_pressed=True, pedal_available=True, pedal_long_allowed=True,
            can_engage=True, enabled=True
        )
        self.assertFalse(res)
        self.assertFalse(self.eng.enableLongControl)

    def test_held_gas_prevents_multiple_triggers(self):
        # Initial press
        self.eng.process_brake_resume(
            curr_time_ms=1000, v_ego=0.0, real_brake_pressed=False,
            gas_pressed=True, pedal_available=True, pedal_long_allowed=True,
            can_engage=True, enabled=True
        )
        
        # Second frame, gas is still held down
        res2 = self.eng.process_brake_resume(
            curr_time_ms=1050, v_ego=0.5, real_brake_pressed=False,
            gas_pressed=True, pedal_available=True, pedal_long_allowed=True,
            can_engage=True, enabled=True
        )
        # Should return False because it is not a fresh rising edge
        self.assertFalse(res2)

if __name__ == '__main__':
    unittest.main()
