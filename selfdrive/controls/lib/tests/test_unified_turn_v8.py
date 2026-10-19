"""Shared city-turn completion and direction regression cases; no vehicle I/O."""
import math
import unittest
from types import SimpleNamespace as N
from cereal import log
from openpilot.selfdrive.controls.tests import test_city_turn_gates as gates
from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnEntry
from openpilot.selfdrive.controls.lib.turn_context import manual_turn_intent

class Lifecycle(unittest.TestCase):
  def model(self, frame, straight=False, side=-1):
    return N(frameId=frame,position=N(t=[0.,.5,1.,2.,3.],x=[0.,2.,4.,8.,10.],
      y=([0.,0.,0.,side*2.,side*4.] if straight else [0.,side*.25,side*.5,side*2.,side*4.])),
      orientation=N(z=([0.,0.,0.,side*.3,side*.6] if straight else [0.,side*.05,side*.1,side*.3,side*.6])),
      action=N(desiredCurvature=0. if straight else side*.03))
  def test_near_exit_clears_intent_despite_far_corner_and_held_signal(self):
    for direction,sign in [(1,-1),(2,1)]:
      with self.subTest(direction=direction):
        e=ManualTurnEntry()
        for i in range(25):e.update(self.model(i,side=sign),direction,5.,True,100+i*.05,actual_curvature=sign*.06)
        self.assertEqual(e.phase,'TURN');self.assertGreater(e.heading,math.radians(12))
        for i in range(25,32):e.update(self.model(i,True,sign),direction,5.,True,100+i*.05,actual_curvature=0.)
        self.assertEqual(e.phase,'COMPLETE');self.assertTrue(e.blocked)
        for i in range(32,50):
          self.assertFalse(e.update(self.model(i,side=sign),direction,5.,True,100+i*.05,actual_curvature=sign*.06))
          self.assertEqual(e.phase,'COMPLETE')
        e.reset()
        for i in range(50,60):e.update(self.model(i,side=sign),direction,5.,True,100+i*.05,actual_curvature=sign*.06)
        self.assertEqual(e.phase,'TURN')
  def test_prediction_alone_does_not_count_as_observed_heading(self):
    e=ManualTurnEntry()
    for i in range(25):e.update(self.model(i),1,5.,True,100+i*.05,actual_curvature=0.)
    for i in range(25,32):e.update(self.model(i,True),1,5.,True,100+i*.05,actual_curvature=0.)
    self.assertEqual(e.heading,0.);self.assertNotEqual(e.phase,'COMPLETE')
  def test_stale_pause_does_not_latch_completed_turn(self):
    e=ManualTurnEntry()
    for i in range(10):e.update(self.model(i),1,5.,True,100+i*.05)
    self.assertFalse(e.update(self.model(10),1,5.,True,100.5,transient_unavailable=True))
    self.assertEqual(e.phase,'WAIT');self.assertFalse(e.blocked)
    for i in range(11,20):e.update(self.model(i),1,5.,True,100+i*.05)
    self.assertEqual(e.phase,'TURN')

class Direction(unittest.TestCase):
  setUp=gates.CityTurnGates.setUp
  prepare=gates.CityTurnGates.prepare
  tick=gates.CityTurnGates.tick
  def test_left_signal_never_selects_right_when_model_predicts_right(self):
    self.prepare()
    self.model.position.y=[-v for v in self.model.position.y]
    self.model.orientation.z=[-v for v in self.model.orientation.z]
    self.model.action.desiredCurvature=.03
    for _ in range(10):self.assertIn(self.tick(),(log.Desire.none,log.Desire.turnLeft))
    self.assertEqual(self.helper.desire,log.Desire.turnLeft)
  def test_mismatched_classifier_cannot_select_opposite_side(self):
    self.prepare()
    for _ in range(20):self.assertEqual(self.tick(direction=2),log.Desire.none)

class SharedConsumer(unittest.TestCase):
  def test_completed_and_waiting_decisions_have_no_turn_speed_authority(self):
    cs=N(vEgo=5.,leftBlinker=True,rightBlinker=False,steeringPressed=False,canValid=True,
         leftBlindspot=False,rightBlindspot=False,gearShifter='drive')
    m=N(frameId=1)
    d=dict(received_mono=100.,frame_id=1,model_evaluated=True,manual_turn_direction=1,
      city_turns_enabled=True,signal_owner='driver',tap_signal_active=False,synthetic_indicator_request='none',
      source='driver_desire_helper',final_desire='turnLeft',manual_turn_blocked=False)
    for phase in ['IDLE','WAIT','COMPLETE','CANCELLED']:
      d['manual_turn_phase']=phase;self.assertFalse(manual_turn_intent(cs,d,m,100.))
    for phase in ['ARMED','TURN','EXIT']:
      d['manual_turn_phase']=phase;self.assertTrue(manual_turn_intent(cs,d,m,100.))

class SteeringCompletion(unittest.TestCase):
  from openpilot.selfdrive.controls.tests import test_manual_turn_commit as commit
  setUp=commit.ManualTests.setUp
  tick=commit.ManualTests.tick
  start=commit.ManualTests.start
  def test_lifecycle_completion_releases_floor_with_blinker_held(self):
    self.start()
    self.d['manual_turn_phase']='COMPLETE'
    self.assertEqual(self.tick(),self.curvature)
    self.assertFalse(self.helper.active)

class Publication(unittest.TestCase):
  def test_one_decision_publishes_direction_phase_and_completion(self):
    from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire
    n=NavigationDesire.__new__(NavigationDesire);n.decision={}
    n.record_model_selection('none','none',True,123,signal_owner='driver',manual_turn_direction=1,
                            city_turns_enabled=True,manual_turn_blocked=True,
                            manual_turn_phase='COMPLETE',manual_turn_reason='NEAR_PATH_STRAIGHT')
    self.assertEqual(n.decision['manual_turn_direction'],1)
    self.assertEqual(n.decision['manual_turn_phase'],'COMPLETE')
    self.assertTrue(n.decision['manual_turn_blocked'])
    self.assertEqual(n.decision['final_desire'],'none')

class SpeedCompletion(unittest.TestCase):
  def test_completed_turn_releases_existing_ceiling_gradually(self):
    from unittest.mock import patch
    from openpilot.selfdrive.controls.lib import corner_assist as ca
    cs=N(vEgo=5.,leftBlinker=True,rightBlinker=False,steeringPressed=False,brakePressed=False,brake=0.,gasPressed=False,
         canValid=True,leftBlindspot=False,rightBlindspot=False,gearShifter='drive',steeringAngleDeg=60.)
    model=N(frameId=10,position=N(x=[0.,4.,8.,10.],y=[0.,-.5,-2.,-4.]))
    cc=N(latActive=True,longActive=True)
    class SM(dict):pass
    sm=SM(carState=cs,carControl=cc,modelV2=model);sm.valid={k:True for k in sm};sm.alive=sm.valid.copy();sm.logMonoTime={k:int(100e9) for k in sm}
    d=dict(received_mono=100.,frame_id=10,model_evaluated=True,manual_turn_direction=1,city_turns_enabled=True,
           signal_owner='driver',tap_signal_active=False,synthetic_indicator_request='none',source='driver_desire_helper',final_desire='turnLeft',manual_turn_phase='TURN')
    c=ca.CornerAssist();c.config['corner_assist']=True;cp=N(steerRatio=15.,wheelbase=2.96)
    with patch.object(ca.time,'monotonic',return_value=100.):first=c.prepare(sm,cp,False,12.,d)
    self.assertIsNotNone(first);self.assertLess(first,12.)
    cs.vEgo=first;d.update(manual_turn_phase='COMPLETE',manual_turn_blocked=True,final_desire='none')
    with patch.object(ca.time,'monotonic',return_value=100.05):released=c.prepare(sm,cp,False,12.,d)
    self.assertAlmostEqual(released,first+.03)
    self.assertEqual(c.turn_plan.phase,'EXIT')
    cs.gasPressed=True
    with patch.object(ca.time,'monotonic',return_value=100.1):self.assertIsNone(c.prepare(sm,cp,False,12.,d))
    self.assertEqual(c.turn_plan.owner,'DRIVER')

class FreshCompletion(unittest.TestCase):
  def test_stale_frame_cannot_complete_an_exit_dwell(self):
    e=ManualTurnEntry();fixture=Lifecycle()
    for i in range(25):e.update(fixture.model(i),1,5.,True,100+i*.05,actual_curvature=-.06)
    m=fixture.model(25,True)
    self.assertTrue(e.update(m,1,5.,True,101.25,actual_curvature=0.))
    self.assertEqual(e.phase,'EXIT')
    self.assertFalse(e.update(m,1,5.,True,101.55,actual_curvature=0.))
    self.assertEqual(e.phase,'WAIT');self.assertFalse(e.blocked)
