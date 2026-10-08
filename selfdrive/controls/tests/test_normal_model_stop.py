from types import SimpleNamespace as N
from unittest.mock import patch
from openpilot.selfdrive.controls.lib.normal_model_stop import NormalModelStop
from openpilot.selfdrive.controls.lib.tests.test_preap_accel_transitions import PlannerTransitions
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


def model(accel=-1., stop=False, speeds=(10.,5.,0.)):
  return N(action=N(desiredAcceleration=accel, shouldStop=stop), velocity=N(t=[0.,5.,10.],x=list(speeds)))


def test_approach_and_clear_dwell():
  state=NormalModelStop(.05)
  assert state.update(model(),10.,enabled=True,valid=True,reset=False)
  for _ in range(5):
    assert state.update(model(.2,speeds=(10.,10.,10.)),10.,enabled=True,valid=True,reset=False)
  assert not state.update(model(.2,speeds=(10.,10.,10.)),10.,enabled=True,valid=True,reset=False)


def test_gates_and_invalid_forecast():
  for kwargs in [dict(enabled=False,valid=True,reset=False),dict(enabled=True,valid=False,reset=False),dict(enabled=True,valid=True,reset=True)]:
    s=NormalModelStop(.05);s.active=True
    assert not s.update(model(),10.,**kwargs)
  for m in [model(speeds=(10.,8.,5.)), model(accel=.2), model(speeds=(30.,5.,0.)),model(speeds=(10.,float('nan'),0.))]:
    assert not NormalModelStop(.05).update(m,10.,enabled=True,valid=True,reset=False)
  m=model();m.velocity.t=[0.,5.,4.]
  assert not NormalModelStop(.05).update(m,10.,enabled=True,valid=True,reset=False)


def test_imminent_flag_without_forecast():
  assert NormalModelStop(.05).update(N(action=N(desiredAcceleration=-1.,shouldStop=True)),0.,enabled=True,valid=True,reset=False)


class TestNormalPlannerStops(PlannerTransitions):
  def test_normal_stops_without_experimental_or_profile_changes(self):
    self.sm['modelV2'].action=model().action;self.sm['modelV2'].velocity=model().velocity
    self.plan.return_value=(.4,False)
    with patch.object(lp,'normal_model_stop_enabled',return_value=True):
      assert self.tick()==-1.
      assert not self.p.output_should_stop
      self.sm['modelV2'].action.shouldStop=True
      assert self.tick()==-1.
      assert self.p.output_should_stop
      self.sm['modelV2'].action=model(.2).action;self.sm['modelV2'].velocity=model(.2,speeds=(10.,10.,10.)).velocity
      for _ in range(5): assert self.tick()<=0.
      assert self.tick()>0.
      assert not self.p.output_should_stop
    assert not self.sm['selfdriveState'].experimentalMode

  def test_switch_off_and_gas_override(self):
    self.sm['modelV2'].action=model().action;self.sm['modelV2'].velocity=model().velocity;self.plan.return_value=(.4,False)
    with patch.object(lp,'normal_model_stop_enabled',return_value=False):
      assert self.tick()>0.
    self.sm['carState'].gasPressed=True
    with patch.object(lp,'normal_model_stop_enabled',return_value=True):
      self.tick();assert not self.p.normal_model_stop.active

  def test_mpc_lead_stop_still_wins(self):
    self.sm['modelV2'].action=model().action;self.sm['modelV2'].velocity=model().velocity;self.plan.return_value=(-2.,True)
    with patch.object(lp,'normal_model_stop_enabled',return_value=True):
      assert self.tick()<=-2.
      assert self.p.output_should_stop

  def test_exp_behavior_unaffected(self):
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['modelV2'].action=model(-.4).action;self.sm['modelV2'].velocity=model(-.4,speeds=(10.,9.,8.)).velocity
    self.plan.return_value=(.4,False)
    with patch.object(lp,'normal_model_stop_enabled',return_value=True):
      assert self.tick()==-.4
      assert not self.p.normal_model_stop.active
