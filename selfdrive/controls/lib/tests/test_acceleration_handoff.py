"""Source-switch sequences and braking priority through the real planner; no CAN."""
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
from opendbc.car.tesla.preap.accel_policy import ACCEL_RISE, REGEN_RELEASE_JERK
from openpilot.selfdrive.controls.lib.acceleration_handoff import handoff_acceleration, braking_priority, DECEL_JERK
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as base
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


def model(accel=0.,speeds=None):
  return N(action=N(desiredAcceleration=accel,shouldStop=False),velocity=N(t=[0.,1.,3.,6.,10.],x=speeds or [20.,20.,19.8,19.5,19.5]))


class Handoff(unittest.TestCase):
  def test_complete_source_sequences_have_no_history_reset(self):
    for personality in (0,1,2):
      for dt in (.01,.02,.05):
        previous=0.
        phases=[('cruise',.5),('curve',-.3),('coast',0.),('e2e',-.15),('lead_release',.1),('cruise',.5)]
        for name,request in phases:
          for _ in range(round(2./dt)):
            value=handoff_acceleration(request,previous,dt,personality,-2.5,.7)
            delta=value-previous
            self.assertGreaterEqual(delta,-DECEL_JERK[personality]*dt-1e-9,name)
            self.assertLessEqual(delta,max(REGEN_RELEASE_JERK[personality],ACCEL_RISE[personality])*dt+1e-9,name)
            self.assertLessEqual(value,.7)
            self.assertGreaterEqual(value,-2.5)
            previous=value

  def test_priority_braking_is_immediate_release_still_smooth(self):
    for reason in ('STOP','LEAD_BRAKING','STRONG_BRAKING','SPEED_CEILING','CORNER_BRAKING','FORCE_DECEL'):
      self.assertEqual(handoff_acceleration(-2.5,.4,.05,2,-2.5,.7,reason),-2.5)
      self.assertAlmostEqual(handoff_acceleration(.4,-2.5,.05,2,-2.5,.7,reason),-2.47)

  def test_profile_ceiling_and_zero_crossing_are_exact(self):
    self.assertEqual(handoff_acceleration(.6,.5,.05,2,-2.5,.2),.2)
    value=handoff_acceleration(.4,-.001,.02,2,-2.5,.7)
    self.assertAlmostEqual(value,.3*(.02-.001/.6))

  def test_priority_recognizes_stop_lead_curve_and_planned_model_braking(self):
    def priority(**kw):
      args=dict(model=model(-.3),speed=20.,request=-.3,mpc=.4,has_lead=False,stop=False,force_decel=False,effective_speed=25.,driver_speed=25.)
      args.update(kw);return braking_priority(**args)
    self.assertEqual(priority(),'ORDINARY')
    self.assertEqual(priority(stop=True),'STOP')
    self.assertEqual(priority(has_lead=True,mpc=-.2),'LEAD_BRAKING')
    self.assertEqual(priority(effective_speed=18.),'SPEED_CEILING')
    self.assertEqual(priority(model=model(-.3,[20.,19.,16.,8.,0.])),'PLANNED_MODEL_BRAKING')
    self.assertEqual(priority(valid=False),'INVALID_INPUT')
    self.assertEqual(priority(model=N(action=N(desiredAcceleration=-.3))),'UNPROVEN_MODEL_BRAKING')


class PlannerHandoff(unittest.TestCase):
  setUp=base.PlannerTransitions.setUp
  tick=base.PlannerTransitions.tick

  def prepare(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['carState'].vEgo=20.
    self.sm['modelV2'].velocity=model().velocity
    self.sm['modelV2'].action.desiredAcceleration=.4
    self.sm['radarState'].leadTwo=N(status=False)
    self.plan.return_value=(.5,False)
    for _ in range(50):self.tick()

  def test_cruise_model_coast_and_lead_release_use_same_final_history(self):
    self.prepare();previous=self.p.output_a_target
    self.sm['modelV2'].action.desiredAcceleration=-.3
    value=self.tick();self.assertAlmostEqual(value,previous-.02)
    for _ in range(50):value=self.tick()
    self.assertAlmostEqual(value,-.3)
    self.sm['modelV2'].action.desiredAcceleration=.4
    value=self.tick();self.assertAlmostEqual(value,-.27)
    self.sm['radarState'].leadOne.status=True
    self.plan.return_value=(-1.2,False)
    self.assertEqual(self.tick(),-1.2)
    self.plan.return_value=(.2,False)
    self.assertAlmostEqual(self.tick(),-1.17)
    self.sm['radarState'].leadOne.status=False
    for _ in range(100):value=self.tick()
    self.assertLessEqual(value,.2)
    self.assertGreater(value,0.)

  def test_stop_interrupts_and_driver_override_restarts_gently(self):
    self.prepare();self.sm['modelV2'].action.shouldStop=True
    self.plan.return_value=(-2.5,True)
    self.assertEqual(self.tick(),-2.5)
    self.assertTrue(self.p.output_should_stop)
    self.sm['modelV2'].action.shouldStop=False;self.plan.return_value=(.5,False)
    self.sm['carState'].gasPressed=True
    self.assertAlmostEqual(self.tick(),.015)
    self.assertEqual(self.p.handoff_reason,'OFF')
    self.sm['carState'].gasPressed=False
    self.assertAlmostEqual(self.tick(),.03)

  def test_binding_map_ceiling_has_immediate_braking_priority(self):
    self.prepare();self.p.nap_map_driving_assist=True
    self.p._params.get_bool.side_effect=lambda k:k in ('NAPAdaptiveAccel','NAPMapDrivingAssist')
    self.plan.return_value=(-.4,False)
    with patch.object(lp,'map_curve_speed_cap',return_value=18.):
      self.assertEqual(self.tick(),-.4)
      self.assertEqual(self.p.handoff_reason,'SPEED_CEILING')
