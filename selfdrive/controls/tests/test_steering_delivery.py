"""Exercise model pulse delivery and actual Tesla controller routing without CAN I/O."""
import unittest
import json
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import Mock, patch
import numpy as np
from openpilot.selfdrive.modeld.modeld import ModelState
from opendbc.car.tesla.carcontroller import CarController

class PolicyReached(Exception):
  pass

class PulseDelivery(unittest.TestCase):
  def setUp(self):
    self.model = object.__new__(ModelState)
    m = self.model
    m.prev_desire = np.zeros(8, dtype=np.float32)
    tensor = Mock()
    tensor.realize.return_value = tensor
    m.update_imgs = Mock(return_value=[tensor] * 4)
    m.img_queues = dict(img=tensor, big_img=tensor)
    m.full_frames = dict(img=tensor, big_img=tensor)
    m.transforms = dict(img=tensor, big_img=tensor)
    m.vision_run = Mock()
    m.vision_run.return_value.contiguous.return_value.realize.return_value.uop.base.buffer.numpy.return_value = np.zeros(1)
    m.vision_output_slices = {}
    m.slice_outputs = Mock(return_value={})
    m.parser = Mock()
    m.parser.parse_vision_outputs.return_value = dict(hidden_state=np.zeros(1))
    m.full_input_queues = Mock()
    m.full_input_queues.get.side_effect = lambda k: {k:np.zeros(8 if k=='desire_pulse' else 1)}
    m.numpy_inputs = dict(desire_pulse=np.zeros(8), features_buffer=np.zeros(1), traffic_convention=np.zeros(2))
    m.policy_inputs = {}
    m.policy_run = Mock(side_effect=PolicyReached)

  def run_frame(self, desire, skip=False):
    vector = np.zeros(8, dtype=np.float32)
    vector[desire] = 1
    args = ({}, {}, dict(desire_pulse=vector, traffic_convention=np.zeros(2)), skip)
    if skip:
      self.assertIsNone(self.model.run(*args))
      return None
    with self.assertRaises(PolicyReached):
      self.model.run(*args)
    return self.model.full_input_queues.enqueue.call_args.args[0]['desire_pulse']

  def test_skipped_first_turn_frame_cannot_swallow_intent(self):
    for side in (1,2):
      self.setUp()
      self.run_frame(side, True)
      self.run_frame(side, True)
      self.assertEqual(self.run_frame(side)[side], 1.)
      self.assertEqual(self.run_frame(side)[side], 0.)

  def test_cancellation_before_evaluation_cannot_replay_stale_turn(self):
    self.run_frame(1, True)
    self.assertFalse(self.run_frame(0).any())
    self.assertEqual(self.run_frame(2)[2], 1.)

  def test_skipped_neutral_cannot_retrigger_same_held_intent(self):
    self.assertEqual(self.run_frame(1)[1], 1.)
    self.run_frame(0, True)
    self.assertFalse(self.run_frame(1).any())

class TeslaTakeover(unittest.TestCase):
  def setUp(self):
    self.c = object.__new__(CarController)
    c = self.c
    c.CP = N(openpilotLongitudinalControl=False)
    c.frame = 0
    c.apply_angle_last = 0.
    c.VM = N()
    c.params = N(get_bool=lambda k:True)
    c.low_speed_steering_rate_enabled = True
    c.tap_params = None
    c.tesla_can = Mock()
    c.stock_cc = N(update=Mock(return_value=[]), pcc_event=None)
    c.parked_signal_test = N(update=Mock(return_value=[]))
    c.tap_lane_change = N(update=Mock(return_value=[]))
    c.navigation_signal = N(update=Mock(return_value=[]))
    self.cs = N(hands_on_level=0, tap_stalk=N(), out=N(vEgo=3., vEgoRaw=3.,
                   leftBlinker=True, rightBlinker=False, steeringPressed=False, steeringAngleDeg=0.))
    act = N(steeringAngleDeg=0., as_builder=lambda:N(steeringAngleDeg=0.))
    self.cc = N(latActive=True, actuators=act)
    self.limiter = self.enterContext(patch('opendbc.car.tesla.carcontroller.apply_steer_angle_limits_vm',
                                        side_effect=lambda target,*args:target))

  def tick(self, count=1):
    for _ in range(count): self.c._update_preap(self.cc,self.cs)

  def test_signal_cannot_keep_centered_takeover_latched(self):
    self.cs.out.steeringPressed=True
    self.tick()
    self.assertTrue(self.c.low_speed_turn_takeover_latched)
    self.cs.out.steeringPressed=False
    self.tick(49)
    self.assertTrue(self.c.low_speed_turn_takeover_latched)
    self.tick(2)
    self.assertFalse(self.c.low_speed_turn_takeover_latched)
    # Assert actual controller enables its steering CAN builder after handoff.
    self.assertTrue(self.c.tesla_can.create_steering_control.call_args.args[2])

  def test_curved_or_misaligned_path_cannot_reacquire(self):
    self.cs.hands_on_level=2
    self.tick()
    self.cs.hands_on_level=0
    self.cc.actuators.steeringAngleDeg=30.
    self.tick(100)
    self.assertTrue(self.c.low_speed_turn_takeover_latched)
    self.assertFalse(self.c.tesla_can.create_steering_control.call_args.args[2])
    self.cc.actuators.steeringAngleDeg=8.
    self.cs.out.steeringAngleDeg=-8.
    self.tick(100)
    self.assertTrue(self.c.low_speed_turn_takeover_latched)

  def test_renewed_driver_input_restarts_quiet_interval(self):
    self.cs.hands_on_level=2
    self.tick()
    self.cs.hands_on_level=0
    self.tick(30)
    self.cs.out.steeringPressed=True
    self.tick()
    self.cs.out.steeringPressed=False
    self.tick(30)
    self.assertTrue(self.c.low_speed_turn_takeover_latched)
    self.tick(21)
    self.assertFalse(self.c.low_speed_turn_takeover_latched)

  def test_disengagement_never_sends_active_steering(self):
    self.cc.latActive=False
    self.tick(100)
    self.assertFalse(self.c.tesla_can.create_steering_control.call_args.args[2])

  def test_recorded_approach_releases_after_input_ends_despite_held_signal(self):
    samples = json.loads((Path(__file__).parent / "fixtures/steering_takeover_291.json").read_text())
    was_latched = False
    released = False
    for row in samples:
      self.cs.hands_on_level = row["hands"]
      self.cs.out.steeringPressed = row["pressed"]
      self.cs.out.steeringAngleDeg = row["angle"]
      self.cs.out.vEgo = self.cs.out.vEgoRaw = row["speed"]
      self.cs.out.leftBlinker, self.cs.out.rightBlinker = row["left"], row["right"]
      self.cc.latActive = row["lat"]
      self.cc.actuators.steeringAngleDeg = row["target"]
      self.tick()  # full-rate recorded EPAS samples, approximately 100 Hz
      was_latched |= self.c.low_speed_turn_takeover_latched
      if row["t"] >= 158.8:
        released |= not self.c.low_speed_turn_takeover_latched and row["left"]
    self.assertTrue(was_latched)
    self.assertTrue(released)
