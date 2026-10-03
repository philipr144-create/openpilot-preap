"""Recorded mild model/cruise disagreements; no CAN output."""
import unittest
from types import SimpleNamespace as N
from openpilot.selfdrive.controls.lib.cruise_refinement import mild_transition_forecast, shape_mild_transition
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as base


def forecast(speed=22.,stop=False):
  return N(velocity=N(t=[0.,1.,3.,6.,10.],x=[speed-.8,speed-1.,speed-1.,speed-.9,speed-1.2]),action=N(shouldStop=stop))


class ForecastGates(unittest.TestCase):
  def test_recorded_offset_speed_holding_shape_qualifies(self):
    self.assertTrue(mild_transition_forecast(forecast(),22.))

  def test_stops_sustained_slowdown_missing_invalid_and_low_speed_fail(self):
    for model,speed in ((forecast(stop=True),22.),(N(),22.),(forecast(),5.)):
      self.assertFalse(mild_transition_forecast(model,speed))
    m=forecast();m.velocity.x=[21.2,20.,18.,12.,0.]
    self.assertFalse(mild_transition_forecast(m,22.))
    m=forecast();m.velocity.x[-1]=float('nan')
    self.assertFalse(mild_transition_forecast(m,22.))
    m=forecast();m.velocity.t[-1]=6.
    self.assertFalse(mild_transition_forecast(m,22.))

  def test_mild_regen_ramp_and_continuous_zero_crossing(self):
    previous=.4
    for i in range(20):
      value=shape_mild_transition(-.3,previous,.05,True)
      self.assertLessEqual(value,0.)
      self.assertGreaterEqual(value,min(previous,0.)-.02-1e-9)
      previous=value
    self.assertAlmostEqual(previous,-.3)
    for _ in range(20):
      value=shape_mild_transition(.4,previous,.05,True)
      if previous<0.:self.assertLessEqual(value-previous,.02+1e-9)
      previous=value
    self.assertEqual(shape_mild_transition(-2.5,.4,.05,False),-2.5)


class PlannerComfort(unittest.TestCase):
  setUp=base.PlannerTransitions.setUp
  tick=base.PlannerTransitions.tick

  def prepare(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda key:key=='NAPAdaptiveAccel'
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['carState'].vEgo=22.
    self.sm['modelV2'].velocity=forecast().velocity
    self.sm['radarState'].leadTwo=N(status=False)
    self.sm['modelV2'].action.desiredAcceleration=.4
    self.plan.return_value=(.5,False)
    for _ in range(50):self.tick()

  def test_recorded_brake_switch_ramps_and_recovery_crosses_zero(self):
    self.prepare()
    self.sm['modelV2'].action.desiredAcceleration=-.3
    self.assertAlmostEqual(self.tick(),-.02)
    self.assertEqual(self.p.exp_long_state,'MODEL_COMFORT')
    for _ in range(20):value=self.tick()
    self.assertAlmostEqual(value,-.3)
    self.sm['modelV2'].action.desiredAcceleration=.4
    self.assertAlmostEqual(self.tick(),-.28)
    for _ in range(30):value=self.tick()
    self.assertGreater(value,0.)
    self.assertLessEqual(value,.5)

  def test_stop_lead_mpc_strong_brake_and_binding_ceiling_bypass(self):
    for gate in ('stop','lead','mpc','strong','ceiling','forecast'):
      with self.subTest(gate=gate):
        self.prepare();self.p.output_a_target=.4
        self.sm['modelV2'].action.desiredAcceleration=-.3
        expected=-.3
        if gate=='stop':self.sm['modelV2'].action.shouldStop=True
        elif gate=='lead':self.sm['radarState'].leadOne.status=True
        elif gate=='mpc':self.plan.return_value=(-1.2,False);expected=-1.2
        elif gate=='strong':self.sm['modelV2'].action.desiredAcceleration=-1.5;expected=-1.5
        elif gate=='ceiling':self.sm['carState'].vCruise=22.*3.6
        else:self.sm['modelV2'].velocity.x=[21.,20.,18.,10.,0.]
        self.assertAlmostEqual(self.tick(),expected)
        self.sm['modelV2'].action.shouldStop=False
        self.sm['radarState'].leadOne.status=False
        self.sm['carState'].vCruise=90.
