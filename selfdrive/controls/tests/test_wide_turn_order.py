"""Exercise controlsd's actual curvature pipeline without starting its processes."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace as N
import unittest

from cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL


def pipeline():
  source = Path(os.environ.get('CONTROLSD_TEST_SOURCE', Path(__file__).resolve().parents[1] / 'controlsd.py'))
  tree = ast.parse(source.read_text())
  constants = [n for n in tree.body if isinstance(n, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id.startswith('LOW_SPEED_TURN_') for t in n.targets)]
  scale = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'low_speed_turn_scale')
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Controls')
  method = next(n for n in cls.body if isinstance(n, ast.FunctionDef)
                and any(isinstance(x, ast.Assign) and isinstance(x.value, ast.IfExp)
                        and ast.unparse(x.targets[0]) == 'new_desired_curvature' for x in n.body))
  start = next(i for i,n in enumerate(method.body) if isinstance(n, ast.Assign)
               and isinstance(n.value, ast.IfExp) and ast.unparse(n.targets[0]) == 'new_desired_curvature')
  end = next(i for i,n in enumerate(method.body) if isinstance(n, ast.AugAssign)
             and ast.unparse(n.target) == 'new_desired_curvature')
  fn = ast.parse('def run(self, CS, CC, model_v2):\n  pass').body[0]
  fn.body = method.body[start:end] + [ast.Return(value=ast.Name(id='new_desired_curvature',ctx=ast.Load()))]
  namespace = dict(CV=CV, DT_CTRL=DT_CTRL, log=log)
  module=ast.fix_missing_locations(ast.Module(body=constants+[scale,fn],type_ignores=[]))
  exec(compile(module,str(source),'exec'),namespace)
  return namespace['run']


class SM(dict):
  frame=1
  def all_checks(self, _): return True


class Commitment:
  def __init__(self,target): self.target=target; self.inputs=[]
  def update(self,curvature,*args,**kwargs):
    self.inputs.append(curvature)
    return self.target


class WideTurnOrder(unittest.TestCase):
  @classmethod
  def setUpClass(cls): cls.run_pipeline=staticmethod(pipeline())

  def setup_turn(self,sign=1.,speed=4.,enabled=True,blinker=True):
    self.commit=Commitment(sign*.055)
    self.control=N(navigation_turn_commit=self.commit,curvature=0.,
                   CP=N(carFingerprint='TESLA_MODEL_S_PREAP'),
                   sm=SM(liveCalibration=N(calStatus=log.LiveCalibrationData.Status.calibrated)),
                   wide_low_speed_turns_enabled=enabled, low_speed_turn_curvature=0.,
                   low_speed_turn_smoothing_active=False, low_speed_turn_entry_active=False,
                   low_speed_turn_ramp_time=0.)
    self.cs=N(vEgo=speed,leftBlinker=blinker and sign<0,rightBlinker=blinker and sign>0)
    self.cc=N(latActive=True)
    self.model=N(action=N(desiredCurvature=sign*.04))

  def tick(self): return self.run_pipeline(self.control,self.cs,self.cc,self.model)

  def test_city_turn_path_never_gets_extra_widening(self):
    for sign in (-1., 1.):
      for ownership in ('active', 'manual'):
        self.setup_turn(sign)
        if ownership == 'active': self.commit.active = True
        else:
          self.commit.decision = dict(city_turns_enabled=True, signal_owner='driver',
                                      manual_turn_direction=1 if sign < 0 else 2)
        self.assertEqual(self.tick(), sign*.055)
        self.assertFalse(self.control.low_speed_turn_smoothing_active)

  def test_commitment_cannot_bypass_wide_entry_rate_limit(self):
    for sign in (-1.,1.):
      self.setup_turn(sign)
      output=self.tick()
      self.assertGreater(sign*output,0.)
      self.assertLess(abs(output),.001)
      self.assertEqual(self.commit.inputs,[sign*.04])

  def test_commitment_from_straight_model_still_gets_entry_shaping(self):
    self.setup_turn();self.model.action.desiredCurvature=0.
    self.assertLess(self.tick(),.001)
    self.assertTrue(self.control.low_speed_turn_entry_active)

  def test_complete_turn_target_restored_after_entry(self):
    for sign in (-1.,1.):
      self.setup_turn(sign)
      outputs=[self.tick() for _ in range(200)]
      self.assertAlmostEqual(outputs[-1],sign*.055)
      self.assertFalse(self.control.low_speed_turn_entry_active)
      self.assertTrue(all(abs(b-a)<=.0009+1e-12 for a,b in zip(outputs,outputs[1:])))

  def test_toggle_off_high_speed_and_no_blinker_preserve_request(self):
    for kwargs in ({'enabled':False},{'speed':25*CV.MPH_TO_MS},{'blinker':False}):
      self.setup_turn(**kwargs)
      self.assertEqual(self.tick(),.055)
      self.assertFalse(self.control.low_speed_turn_smoothing_active)


if __name__=='__main__': unittest.main()
