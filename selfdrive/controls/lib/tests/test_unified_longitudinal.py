"""Net acceleration, grade, recovery and ownership integration; no CAN hardware."""
import math
from types import SimpleNamespace as N
from unittest.mock import MagicMock, patch
import numpy as np
import pytest
from cereal import log
from openpilot.selfdrive.controls.lib.longcontrol import LongControl, MAX_POSITIVE_TRACKING_TRIM
from openpilot.selfdrive.controls.lib.cruise_refinement import model_support_weight
from openpilot.selfdrive.controls.lib.model_forecast import assess_model_forecast
from openpilot.selfdrive.controls.lib.acceleration_handoff import braking_priority
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as base
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from opendbc.car.tesla.preap.interface import get_preap_accel_limits
from opendbc.car.tesla.preap.accel_policy import accel_ceiling
from opendbc.car.tesla.pedal.controller import PedalZeroTorque


def cp():
  return N(carFingerprint='TESLA_MODEL_S_PREAP', startingState=False,
           vEgoStarting=.2, stopAccel=-1.5, stoppingDecelRate=2.5,
           longitudinalTuning=N(kpBP=[0.],kpV=[0.],kiBP=[0.],kiV=[.1]))


def cs(accel=0.):
  return N(vEgo=11.176,aEgo=accel,brakePressed=False,cruiseState=N(standstill=False))


@pytest.mark.parametrize('target,measured,integral',[(.250,-.431,.221),(.301,-.480,.221),(.348,-.370,.221),(.363,-.210,.221)])
def test_recorded_relaxed_tracking_error_keeps_compensation(target,measured,integral):
  ctrl=LongControl(cp());ctrl.pid.i=integral
  result=ctrl.update(True,cs(measured),target,False,get_preap_accel_limits(11.176))
  assert result > target  # recorded correction is retained; the profile is now higher
  assert result == pytest.approx(target+integral+.001*(target-measured))
  assert result <= target+MAX_POSITIVE_TRACKING_TRIM


def test_compensation_bound_and_stop_override():
  ctrl=LongControl(cp())
  for _ in range(3000):result=ctrl.update(True,cs(-2.),.35,False,(-2.5,2.5))
  assert .9<result<=.95
  result=ctrl.update(True,cs(),-2.,True,(-2.5,2.5))
  assert result<=0. and ctrl.pid.i==0.
  assert ctrl.update(False,cs(),.35,False,(-2.5,2.5))==0.


@pytest.mark.parametrize('pitch',[-.06,0.,.06])
def test_outer_feedback_without_pitch_feedforward_tracks_gentle_net_acceleration(pitch):
  ctrl=LongControl(cp());state=cs();vdas=VirtualDAS(.02)
  grade=9.81*math.sin(pitch);actual=0.;di=3.5
  # An explicit affine motor/road-load plant isolates units and feedback.
  # This is not a vehicle model or a claim about measured road performance.
  with patch.object(vdas,'_feedforward',side_effect=lambda a,v,net_accel=None:20.*a+3.5):
    for tick in range(6000):
      state.aEgo=actual
      request=ctrl.update(True,state,.30,False,(-2.5,2.5))
      if tick%2==0:
        di=vdas.update(request,state.vEgo,vdas.prev_pedal_di,a_ego=actual,orientation_ned=[0.,pitch,0.],personality=2)
      actual += .01/(.25+.01)*((di-3.5)/20.-grade-actual)
  assert actual==pytest.approx(.30,abs=.06)
  assert ctrl.pid.i <= MAX_POSITIVE_TRACKING_TRIM


def test_grade_cannot_erase_strong_regen_and_motor_slew_is_bounded():
  v=VirtualDAS(.02);previous=0.
  for command in ([.3]*100+[-2.5]*150):
    v.update(command,10.,v.prev_pedal_di,orientation_ned=[0.,.15,0.],personality=2)
    assert abs(v.motor_accel-previous)<=.05+1e-9
    previous=v.motor_accel
  assert v.grade_load==0. and v.motor_accel==pytest.approx(-2.5)
  v.update(0.,10.,v.prev_pedal_di,orientation_ned=[0.,float('nan'),0.])
  assert math.isfinite(v.motor_accel)


def test_zero_torque_learning_requires_owned_settled_pedal():
  z=PedalZeroTorque()
  for _ in range(100):z.update(-1.,8.,15.,valid=False)
  assert z.value==0.
  for i in range(100):z.update(-1.,float(i),15.,valid=True)
  assert z.value==0.
  for _ in range(30):z.update(-1.,8.,15.,valid=True)
  assert z.value==8.


class TestPlannerOwnership:
  @pytest.fixture(autouse=True)
  def setup(self):
    # Existing unittest fixture patches the real planner's external dependencies.
    self.case=base.PlannerTransitions();self.case.setUp()
    yield
    self.case.doCleanups()

  def test_far_stop_cannot_receive_cruise_recovery(self):
    c=self.case;c.p.nap_adaptive_accel=True
    c.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    c.sm['selfdriveState'].experimentalMode=True
    model=c.sm['modelV2'];model.action.desiredAcceleration=-.1
    model.velocity=N(t=[0.,1.,2.,3.,5.,7.,10.],x=[10.,10.,10.,10.,6.,0.,0.])
    c.plan.return_value=(.95,False)
    forecast=assess_model_forecast(model,10.)
    assert forecast.planned_stop and model_support_weight(model,10.,-.1,forecast)==0.
    assert c.tick()==pytest.approx(-.1)
    assert c.p.cruise_refinement.support==0.
    assert c.p.handoff_reason=='PLANNED_MODEL_BRAKING'

  def test_map_stop_setting_reaches_lifecycle(self):
    c=self.case;c.p.nap_map_driving_assist=True
    c.p.map_stop.update=MagicMock(return_value=(None,False))
    with patch.object(lp,'mapped_stop_setting_enabled',return_value=False):c.tick()
    assert c.p.map_stop.update.call_args.kwargs['enabled'] is False
    with patch.object(lp,'mapped_stop_setting_enabled',return_value=True):c.tick()
    assert c.p.map_stop.update.call_args.kwargs['enabled'] is True

  def test_secondary_lead_and_delay_publication(self):
    c=self.case;c.sm['radarState'].leadTwo=N(status=True)
    c.tick();c.sm.logMonoTime={'modelV2':1_900_000_000}
    msg=N(logMonoTime=2_000_000_000,longitudinalPlan=N())
    with patch.object(lp.messaging,'new_message',return_value=msg):c.p.publish(c.sm,MagicMock())
    assert msg.longitudinalPlan.hasLead is True
    assert msg.longitudinalPlan.processingDelay==pytest.approx(.1)


@pytest.mark.parametrize('pitch',[.15,-.15,float('nan')])
def test_route363_requests_do_not_receive_pitch_dependent_torque(pitch):
  # Recorded 1 Hz actuator requests from the Experimental hill burst.
  # Repeat each request at 50 Hz; this is an actuator regression, not
  # a closed-loop vehicle replay or proof of on-road smoothness.
  flat=VirtualDAS(.02);tilted=VirtualDAS(.02)
  for request in [-.228,-.191,.009,-.459,-.319,-.123]:
    for _ in range(50):
      a=flat.update(request,24.,flat.prev_pedal_di,orientation_ned=[0.,0.,0.])
      b=tilted.update(request,24.,tilted.prev_pedal_di,orientation_ned=[0.,pitch,0.])
      assert b==pytest.approx(a)
      assert tilted.grade_load==0.



def test_feedback_headroom_remains_independent_of_new_relaxed_profile():
  ctrl=LongControl(cp());ctrl.pid.i=.4
  result=ctrl.update(True,cs(-.3),.35,False,get_preap_accel_limits(11.176))
  assert result > accel_ceiling(11.176,2)
  assert result == pytest.approx(.35+.4+.001*(.35+.3))
