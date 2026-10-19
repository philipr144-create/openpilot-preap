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
              gas_pressed=False, brake_pressed=False, lead=None)
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
                   {'model_accel':-.08}, {'mpc_accel':-.05},
                   {'effective_speed':16.0}, {'v_ego':26.0},
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

  def test_fades_to_zero_below_55(self):
    for mph in (49.,52.5,54.5,55.,65.):
      p=PredictiveGradeAssist()
      speed=mph/2.2369362921
      for _ in range(100):
        out=p.update(**kw(v_ego=speed,effective_speed=speed,driver_speed=speed))
      if mph < 50:self.assertAlmostEqual(out,.08)
      elif mph < 55:self.assertLess(out,.08)
      else:self.assertEqual(out,0.)

  def test_flat_road_no_boost_and_blips_do_not_trigger(self):
    p=PredictiveGradeAssist()
    for _ in range(200):self.assertEqual(p.update(**kw(model=make_model(.2))),0.)
    p.reset()
    self.assertEqual(p.update(**kw()),0.)
    for _ in range(15):self.assertEqual(p.update(**kw(model=make_model(0.))),0.)


if __name__=='__main__':
  unittest.main()

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
