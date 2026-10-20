import math
import unittest
from types import SimpleNamespace as N
from openpilot.selfdrive.controls.lib.predictive_grade_assist import (PredictiveGradeAssist, future_pitch_rise_deg,
                                     MAX_BOOST_MPS2, MAX_RISE_PER_SEC)


def make_model(delta=3.5):
  xs = [i * 2.0 for i in range(65)]
  pitch = [math.radians(delta * max(0., x-10.) / 30.) for x in xs]
  return N(position=N(x=xs), orientation=N(y=pitch))


def kw(model=None, **overrides):
  args = dict(model=model or make_model(), enabled=True, engaged=True,
              v_ego=17.0, a_ego=-.15, effective_speed=17.0,
              driver_speed=17.0, base_accel=.02, mpc_accel=.35,
              model_accel=.20, brake_reason='ORDINARY', model_stop=False,
              mpc_stop=False, corner_braking=False, force_decel=False,
              gas_pressed=False, brake_pressed=False, lead=None, inputs_valid=True,
              forecast_valid=True, model_braking_required=False, max_accel=.9)
  args.update(overrides)
  return args


class PredictiveAssistTests(unittest.TestCase):
  def test_future_pitch_relative_only(self):
    self.assertAlmostEqual(future_pitch_rise_deg(make_model()), 3.5)
    m=make_model();m.orientation.y=[.02+v for v in m.orientation.y]
    self.assertAlmostEqual(future_pitch_rise_deg(m),3.5)
    m.position.x=m.position.x[:-3]
    self.assertIsNone(future_pitch_rise_deg(m))

  def test_off_is_unchanged_and_no_veto_bypass(self):
    p=PredictiveGradeAssist()
    for kwargs in ({'enabled':False}, {'engaged':False}, {'model_stop':True},
                   {'mpc_stop':True}, {'force_decel':True},
                   {'brake_reason':'LEAD_BRAKING'}, {'brake_reason':'SPEED_CEILING'},
                   {'brake_reason':'CORNER_BRAKING'},
                   {'corner_braking':True}, {'gas_pressed':True},
                   {'brake_pressed':True}, {'base_accel':-.20},
                   {'model_accel':-.30}, {'mpc_accel':-.05},
                   {'effective_speed':16.0}, {'v_ego':37.0},
                   {'model': make_model(-3.5)}):
      p.reset()
      for _ in range(100):
        self.assertEqual(p.update(**kw(**kwargs)),0.0, kwargs)

  def test_boost_cap_delay_and_slew(self):
    p=PredictiveGradeAssist()
    prev=0.0
    for i in range(200):
      out=p.update(**kw())
      if i<4:self.assertEqual(out,0.0)
      self.assertGreaterEqual(out,0.)
      self.assertLessEqual(out,MAX_BOOST_MPS2+1e-10)
      self.assertLessEqual(out-prev,MAX_RISE_PER_SEC*p.dt+1e-8)
      prev=out
    self.assertAlmostEqual(out,MAX_BOOST_MPS2)
    self.assertEqual(p.update(**kw(model_stop=True)),0.)

  def test_behind_lead_if_mpc_permits(self):
    p=PredictiveGradeAssist()
    lead=N(status=True,dRel=30.,vRel=.0)
    for _ in range(100):out=p.update(**kw(lead=lead,mpc_accel=.09,base_accel=.04))
    self.assertAlmostEqual(out,.05)
    self.assertLessEqual(out+.04,.09)
    lead.vRel=-1.0
    self.assertEqual(p.update(**kw(lead=lead)),0.)
    lead.vRel=0.0;lead.dRel=15.0
    self.assertEqual(p.update(**kw(lead=lead)),0.)

  def test_support_through_80_and_rejects_above_limit(self):
    for mph in (49.,60.,65.,72.5,79.5,80.,80.5):
      p=PredictiveGradeAssist()
      speed=mph/2.2369362921
      for _ in range(200):
        out=p.update(**kw(v_ego=speed,effective_speed=speed,driver_speed=speed))
      if mph <=65.:self.assertAlmostEqual(out,MAX_BOOST_MPS2)
      elif mph <=80.:self.assertGreaterEqual(out,.20-1e-8);self.assertLess(out,MAX_BOOST_MPS2)
      else:self.assertEqual(out,0.)

  def test_flat_road_no_boost_and_blips_do_not_trigger(self):
    p=PredictiveGradeAssist()
    for _ in range(200):self.assertEqual(p.update(**kw(model=make_model(.2))),0.)
    p.reset()
    self.assertEqual(p.update(**kw()),0.)
    for _ in range(15):self.assertEqual(p.update(**kw(model=make_model(0.))),0.)


class FileToggleTests(unittest.TestCase):
  def test_default_off_and_explicit_true_only(self):
    import json, tempfile, pathlib
    from openpilot.selfdrive.controls.lib.predictive_grade_assist import FileGradeToggle
    with tempfile.TemporaryDirectory() as tmp:
      p=pathlib.Path(tmp)/'grade.json'
      t=FileGradeToggle(str(p),dt=.6)
      self.assertFalse(t.read())
      p.write_text(json.dumps({'enabled':'true'}))
      self.assertFalse(t.read())
      p.write_text(json.dumps({'enabled':True}))
      self.assertTrue(t.read())
      p.write_text('{broken')
      self.assertFalse(t.read())
      p.unlink()
      p.symlink_to('/etc/hosts')
      self.assertFalse(t.read())

class SustainedHillTests(unittest.TestCase):
  def test_neutral_cruise_and_weak_model_coast_do_not_block_preview(self):
    p=PredictiveGradeAssist()
    for _ in range(40):out=p.update(**kw(base_accel=0.,mpc_accel=0.,model_accel=-.05))
    self.assertGreater(out,.1)
    self.assertTrue(p.climb_active)

  def test_flat_relative_prediction_retains_confirmed_climb_load(self):
    p=PredictiveGradeAssist()
    for _ in range(40):p.update(**kw())
    for _ in range(300):out=p.update(**kw(model=make_model(0.)))
    self.assertGreater(out,.1)
    self.assertTrue(p.climb_active)
    self.assertGreater(p.load_estimate,0.)

  def test_stop_invalid_data_and_every_braking_owner_clear_carried_support(self):
    cases=[dict(model_stop=True),dict(mpc_stop=True),dict(model_braking_required=True),
           dict(forecast_valid=False),dict(inputs_valid=False),dict(gas_pressed=True),
           dict(brake_pressed=True),dict(corner_braking=True),dict(force_decel=True),
           dict(brake_reason='LEAD_BRAKING'),dict(brake_reason='SPEED_CEILING'),
           dict(brake_reason='CORNER_BRAKING'),dict(brake_reason='INVALID_INPUT'),
           dict(effective_speed=16.),dict(base_accel=-.02),dict(mpc_accel=-.02),
           dict(v_ego=16.0,effective_speed=15.5,driver_speed=15.5),
           dict(a_ego=float('nan')),dict(model=make_model(float('nan')))]
    for overrides in cases:
      with self.subTest(overrides=overrides):
        p=PredictiveGradeAssist()
        for _ in range(50):p.update(**kw())
        self.assertGreater(p.output,0.)
        self.assertEqual(p.update(**kw(**overrides)),0.)
        self.assertFalse(p.climb_active)

  def test_two_leads_and_existing_acceleration_preview_ceiling(self):
    p=PredictiveGradeAssist()
    far=N(status=True,dRel=80.,vRel=0.)
    close=N(status=True,dRel=10.,vRel=-1.)
    for _ in range(50):self.assertEqual(p.update(**kw(leads=(far,close))),0.)
    for _ in range(100):out=p.update(**kw(base_accel=.04,max_accel=.06))
    self.assertLessEqual(out+.04,.06+1e-10)
    self.assertEqual(p.update(**kw(base_accel=.04,max_accel=0.)),0.)

  def test_speed_aware_preview_requires_actual_model_extent(self):
    p=PredictiveGradeAssist()
    m=make_model();m.position.x=m.position.x[:31];m.orientation.y=m.orientation.y[:31]
    speed=80./2.2369362921
    self.assertIsNotNone(future_pitch_rise_deg(m))
    self.assertIsNone(future_pitch_rise_deg(m,speed))
    for _ in range(40):self.assertEqual(p.update(**kw(model=m,v_ego=speed,
                                              effective_speed=speed,driver_speed=speed)),0.)

  def test_crest_releases_without_changing_to_braking(self):
    p=PredictiveGradeAssist()
    for _ in range(100):p.update(**kw())
    previous=p.output
    for _ in range(120):
      out=p.update(**kw(model=make_model(-3.5),a_ego=.25))
      self.assertGreaterEqual(out,0.)
      self.assertLessEqual(previous-out,.20*p.dt+1e-9)
      previous=out
    self.assertEqual(out,0.)
    self.assertFalse(p.climb_active)

  def test_inactive_without_preview_does_not_add_speed_feedback(self):
    p=PredictiveGradeAssist()
    for _ in range(200):
      self.assertEqual(p.update(**kw(model=make_model(0.),v_ego=15.,a_ego=-.4)),0.)

  def test_synthetic_hill_tracking_and_false_prediction_overspeed(self):
    def simulate(assist=True, hill=True):
      p=PredictiveGradeAssist();speed=17.;accel=0.;motor=0.;minimum=17.;maximum=17.
      for tick in range(1000):
        t=tick*.05
        load=.25 if hill and 2.<=t<25. else 0.
        delta=3.0 if t<3. else -2.0 if 23.<=t<25. else 0.
        base=max(0.,min(.9,.25*(17.-speed)))
        boost=p.update(**kw(model=make_model(delta),enabled=assist,v_ego=speed,
                            a_ego=accel,base_accel=base,mpc_accel=base,model_accel=-.05))
        # A simple lagged actuator and external uphill load, not a vehicle model.
        motor += .05/(.4+.05)*(base+boost-motor)
        accel=motor-load;speed+=accel*.05
        minimum=min(minimum,speed);maximum=max(maximum,speed)
      return minimum,maximum,speed
    baseline=simulate(False)
    enabled=simulate()
    self.assertGreater(enabled[0],baseline[0]+.25)
    self.assertLess(enabled[1],17.35)
    flat=simulate(True,False)
    self.assertLess(flat[1],17.35)

class PlannerHillIntegration(unittest.TestCase):
  from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as _base
  setUp = _base.PlannerTransitions.setUp
  tick = _base.PlannerTransitions.tick

  def prepare(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda key:key=='NAPAdaptiveAccel'
    self.p.nap_v9e_toggle=N(read=lambda:True)
    self.sm['carControl'].longActive=True
    self.sm['carState'].brakePressed=False
    self.sm['carState'].vEgo=17.
    self.sm['carState'].vCruise=17.*3.6
    self.sm['radarState'].leadTwo=N(status=False)
    m=self.sm['modelV2'];curve=make_model()
    m.position=curve.position;m.position.y=[0.]*len(m.position.x)
    m.orientation=curve.orientation
    m.velocity=N(t=[0.,1.,2.,3.,6.,10.],x=[17.]*6)
    m.action.desiredAcceleration=0.
    self.plan.return_value=(0.,False)

  def test_unified_neutral_cruise_gets_bounded_support_and_stop_interrupts(self):
    self.prepare()
    for _ in range(40):value=self.tick()
    self.assertGreater(value,.05)
    self.assertGreater(self.p.nap_grade_applied_last,0.)
    self.assertTrue(self.p.propulsion_allowed)
    self.sm['modelV2'].action.shouldStop=True
    self.sm['modelV2'].action.desiredAcceleration=-1.
    self.plan.return_value=(-1.,True)
    self.assertLess(self.tick(),0.)
    self.assertEqual(self.p.nap_grade_applied_last,0.)

  def test_corner_positive_ceiling_cannot_be_bypassed(self):
    self.prepare()
    self.sm['modelV2'].action.desiredAcceleration=.4
    self.plan.return_value=(.4,False)
    self.p.corner_assist.apply.side_effect=lambda value,*args:min(value,.03)
    for _ in range(60):self.assertLessEqual(self.tick(),.03+1e-8)

  def test_forecast_braking_and_second_lead_deny_support(self):
    self.prepare()
    self.sm['modelV2'].velocity.x=[17.,16.,14.,12.,10.,8.]
    for _ in range(20):self.tick()
    self.assertEqual(self.p.nap_grade_applied_last,0.)
    self.sm['modelV2'].velocity.x=[17.]*6
    self.sm['radarState'].leadOne=N(status=True,dRel=100.,vRel=0.)
    self.sm['radarState'].leadTwo=N(status=True,dRel=10.,vRel=-1.)
    for _ in range(20):self.tick()
    self.assertEqual(self.p.nap_grade_applied_last,0.)

  def test_experimental_shares_the_same_preview_and_load_support(self):
    self.prepare()
    self.sm['selfdriveState'].experimentalMode=True
    for _ in range(40):value=self.tick()
    self.assertGreater(value,.05)
    self.assertGreater(self.p.nap_grade_applied_last,0.)
    self.sm['carState'].brakePressed=True
    self.tick()
    self.assertEqual(self.p.nap_grade_applied_last,0.)


if __name__=='__main__':
  unittest.main()
