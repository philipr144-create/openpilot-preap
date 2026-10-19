"""Pedal ownership, fresh re-entry and geometry-speed planning; no CAN I/O."""
import math
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
from cereal import log
from openpilot.selfdrive.controls.tests import test_city_turn_gates as gates
from openpilot.selfdrive.controls.tests import test_manual_turn_commit as commit
from openpilot.selfdrive.controls.lib.turn_speed_plan import TurnSpeedPlan, path_turn_target
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist

class PedalIntent(unittest.TestCase):
  setUp=gates.CityTurnGates.setUp
  prepare=gates.CityTurnGates.prepare
  tick=gates.CityTurnGates.tick
  def test_pedals_do_not_cancel_model_turn(self):
    for pedal in ['brakePressed','gasPressed']:
      with self.subTest(pedal=pedal):
        self.setUp();self.prepare();setattr(self.cs,pedal,True)
        for _ in range(10):self.tick()
        self.assertEqual(self.helper.desire,log.Desire.turnLeft)
        setattr(self.cs,pedal,False)
        self.assertEqual(self.tick(),log.Desire.turnLeft)
        self.assertFalse(self.helper.manual_entry.blocked)
  def test_bad_service_suspends_and_requires_new_dwell(self):
    self.prepare()
    for _ in range(10):self.tick()
    self.assertEqual(self.tick(valid=False),log.Desire.none)
    self.assertFalse(self.helper.manual_entry.blocked)
    for _ in range(4):self.assertEqual(self.tick(),log.Desire.none)
    for _ in range(4):self.tick()
    self.assertEqual(self.helper.desire,log.Desire.turnLeft)

class StoppedReentry(unittest.TestCase):
  setUp=PedalIntent.setUp
  prepare=PedalIntent.prepare
  tick=PedalIntent.tick
  def test_standstill_lateral_pause_does_not_latch_signal_out(self):
    self.prepare()
    for _ in range(10):self.tick()
    self.cs.vEgo=0.
    self.now+=.05;self.model.frameId+=1
    self.helper.update(self.cs,False,1.,manual_turn_direction=1,model=self.model,inputs_valid=True,now=self.now)
    self.assertEqual(self.helper.desire,log.Desire.none)
    self.assertFalse(self.helper.manual_entry.blocked)
    self.cs.vEgo=4.
    for _ in range(10):self.tick()
    self.assertEqual(self.helper.desire,log.Desire.turnLeft)


class SteeringPedals(unittest.TestCase):
  setUp=commit.ManualTests.setUp
  tick=commit.ManualTests.tick
  start=commit.ManualTests.start
  def test_brake_and_gas_keep_qualified_lateral_turn(self):
    self.start()
    for name in ['brakePressed','gasPressed']:
      setattr(self.cs,name,True)
      self.tick();self.assertTrue(self.helper.active);self.assertFalse(self.helper.blocked)
      setattr(self.cs,name,False)
    self.cs.steeringPressed=True
    self.tick();self.assertFalse(self.helper.active);self.assertTrue(self.helper.blocked)
  def test_stop_does_not_require_blinker_cycle_but_no_standstill_floor(self):
    self.start();self.cs.vEgo=0.
    for _ in range(10):self.assertEqual(self.tick(),self.curvature)
    self.assertFalse(self.helper.blocked)
    self.cs.vEgo=1.
    for _ in range(10):self.tick()
    self.assertTrue(self.helper.active)
  def test_stale_data_yields_then_requires_fresh_evidence(self):
    self.start();self.tick(dt=.3)
    self.assertFalse(self.helper.active);self.assertFalse(self.helper.blocked)
    for _ in range(3):self.tick();self.assertFalse(self.helper.active)
    for _ in range(10):self.tick()
    self.assertTrue(self.helper.active)

class Geometry(unittest.TestCase):
  def test_future_corner_gets_approach_distance_not_walking_speed_now(self):
    x=[0.,5.,10.,15.,20.,25.,30.,32.,32.];y=[0.,0.,0.,0.,0.,0.,1.,4.,8.]
    near=path_turn_target(x[5:],y[5:],1.2,.8,12.)
    far=path_turn_target(x,y,1.2,.8,12.)
    self.assertGreater(far,near)
    self.assertLess(far,12.)
  def test_straight_and_invalid_paths(self):
    self.assertEqual(path_turn_target([0.,5.,10.],[0.,0.,0.],1.2,.8,12.),12.)
    self.assertIsNone(path_turn_target([0.,5.,10.],[0.,float('nan'),0.],1.2,.8,12.))
  def test_driver_owns_speed_but_turn_context_retained(self):
    p=TurnSpeedPlan()
    self.assertEqual(p.update(5.,8.,.02,12.,1.,True,False,True),5.)
    self.assertIsNone(p.update(5.,7.,.02,12.,1.05,True,True,True))
    self.assertEqual(p.phase,'TURN');self.assertEqual(p.owner,'DRIVER')
    self.assertEqual(p.update(5.,5.,.02,12.,1.10,True,False,True),5.)
    self.assertAlmostEqual(p.update(12.,5.,0.,12.,1.15,True,False,True),5.03)
    self.assertIsNone(p.update(5.,5.,.02,12.,1.20,False,False,True))
    self.assertIsNone(p.target)
  def test_no_automatic_reengagement(self):
    p=TurnSpeedPlan()
    self.assertIsNone(p.update(5.,5.,.02,12.,1.,True,False,False))
    self.assertEqual(p.owner,'WAIT_LONG')

class PedalSpeedOwnership(unittest.TestCase):
  def test_corner_yields_while_pedals_touched(self):
    from openpilot.selfdrive.controls.lib import corner_assist as ca
    cs=N(vEgo=5.,leftBlinker=True,rightBlinker=False,steeringPressed=False,brakePressed=False,brake=0.,gasPressed=False,
         canValid=True,leftBlindspot=False,rightBlindspot=False,gearShifter='drive',steeringAngleDeg=-60.)
    model=N(frameId=10,position=N(x=[0.,4.,8.,10.],y=[0.,-.5,-2.,-4.]))
    cc=N(latActive=True,longActive=True)
    class SM(dict):pass
    sm=SM(carState=cs,carControl=cc,modelV2=model);sm.valid={k:True for k in sm};sm.alive=sm.valid.copy();sm.logMonoTime={k:int(100e9) for k in sm}
    d=dict(received_mono=100.,frame_id=10,model_evaluated=True,manual_turn_direction=1,city_turns_enabled=True,
           signal_owner='driver',tap_signal_active=False,synthetic_indicator_request='none',source='driver_desire_helper',final_desire='turnLeft')
    cp=N(steerRatio=15.,wheelbase=2.96)
    c=CornerAssist();c.config['corner_assist']=True
    with patch.object(ca.time,'monotonic',return_value=100.):
      self.assertIsNotNone(c.prepare(sm,cp,False,12.,d))
      for name in ['brakePressed','gasPressed']:
        setattr(cs,name,True)
        self.assertIsNone(c.prepare(sm,cp,False,12.,d))
        self.assertEqual(c.turn_plan.owner,'DRIVER')
        self.assertEqual(c.apply(.4,sm,cp,False,-.3,12.,d),.4)
        setattr(cs,name,False)
      cc.longActive=False
      self.assertIsNone(c.prepare(sm,cp,False,12.,d))
      cc.latActive=False
      self.assertIsNone(c.prepare(sm,cp,False,12.,d))
      self.assertEqual(c.turn_plan.phase,'OFF')

class PlannerIntegration(unittest.TestCase):
  from openpilot.selfdrive.controls.lib.tests import test_unified_long_v4 as v4
  setUp=v4.PlannerPath.setUp
  tick=v4.PlannerPath.tick
  def test_both_modes_receive_same_city_ceiling_before_mpc(self):
    from unittest.mock import patch
    from openpilot.selfdrive.controls.lib.tests import test_unified_long_v4 as v4
    v4.PlannerPath.prepare(self)
    self.sm['carState'].vCruise=50.
    for mode in [False,True]:
      self.sm['selfdriveState'].experimentalMode=mode
      with patch.object(self.p.corner_assist,'prepare',return_value=6.):self.tick()
      self.assertAlmostEqual(self.mpc.update.call_args.args[1],6.)
