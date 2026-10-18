"""Regressions from routes 377/379 and combined transitions; never transmits CAN."""
import unittest
from types import SimpleNamespace as N
from unittest.mock import patch
from openpilot.selfdrive.controls.lib.longcontrol import StopRelease, update_preap_feedback, LongCtrlState
from openpilot.selfdrive.controls.lib.acceleration_handoff import UnifiedCruiseSmoother
from openpilot.selfdrive.controls.tests.test_preap_longcontrol import setup_controller
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as base
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp

class FeedbackUnits(unittest.TestCase):
  def test_recorded_negative_target_cannot_rewrite_integrator(self):
    ctl,cs=setup_controller();ctl.pid.i=.4;cs.vEgo=15.4;cs.aEgo=.08
    out=ctl.update(True,cs,-2.2,False,(-2.5,2.5))
    self.assertGreater(ctl.pid.i,.35)
    self.assertGreater(out,-1.9)
    self.assertEqual(ctl.pid.pos_limit,2.5)
  def test_exact_pid_limit_reproduction(self):
    ctl,_=setup_controller();pid=ctl.pid;pid.i=.4;pid.set_limits(2.5,-2.5)
    out=update_preap_feedback(pid,-2.28,15.4,-2.2)
    self.assertAlmostEqual(out,-1.8026495173,places=6)
    self.assertGreater(pid.i,.39)
  def test_positive_trim_bound_and_unwind(self):
    ctl,cs=setup_controller();cs.vEgo=.5;cs.aEgo=-.3
    for _ in range(6000):out=ctl.update(True,cs,.14,False,(-2.5,2.5))
    self.assertLessEqual(out,.74+1e-9)
    self.assertGreater(out,.73)
    before=ctl.pid.i;cs.aEgo=.8
    for _ in range(50):ctl.update(True,cs,.14,False,(-2.5,2.5))
    self.assertLess(ctl.pid.i,before)
  def test_full_braking_remains_available(self):
    ctl,cs=setup_controller();cs.aEgo=0.
    self.assertEqual(ctl.update(True,cs,-2.5,False,(-2.5,2.5)),-2.5)
  def test_saturation_does_not_create_hidden_correction(self):
    ctl,_=setup_controller();p=ctl.pid;p.i=.2;p.set_limits(.1,-.5)
    for _ in range(100):update_preap_feedback(p,1.,10.,1.)
    self.assertEqual(p.i,.2)
    for _ in range(100):update_preap_feedback(p,-1.,10.,-1.)
    self.assertEqual(p.i,.2)

class StopTransitions(unittest.TestCase):
  def test_flickering_stop_does_not_restart_ramp(self):
    ctl,cs=setup_controller();cs.vEgo=.05;cs.aEgo=0.
    values=[]
    for i in range(200):
      values.append(ctl.update(True,cs,.02,i%20==0,(-2.5,2.5)))
      self.assertEqual(ctl.long_control_state,LongCtrlState.stopping)
    self.assertEqual(values[-1],-2.5)
    self.assertTrue(all(b<=a for a,b in zip(values,values[1:])))
  def test_sustained_launch_releases_smoothly_without_negative_trim(self):
    ctl,cs=setup_controller();cs.vEgo=0.;cs.aEgo=0.
    for _ in range(110):ctl.update(True,cs,0.,True,(-2.5,2.5))
    values=[]
    for i in range(160):
      values.append(ctl.update(True,cs,.14,False,(-2.5,2.5)))
      if i<29:self.assertEqual(ctl.long_control_state,LongCtrlState.stopping)
      self.assertGreaterEqual(ctl.pid.i,0.)
    self.assertGreater(values[-1],.1)
    self.assertTrue(all(0.<=b-a<=.02500001 for a,b in zip(values,values[1:])))
  def test_small_relaxed_launch_request_can_release_hold(self):
    ctl,cs=setup_controller();cs.vEgo=0.;cs.aEgo=0.
    for _ in range(110):ctl.update(True,cs,0.,True,(-2.5,2.5))
    for _ in range(160):value=ctl.update(True,cs,.027,False,(-2.5,2.5))
    self.assertEqual(ctl.long_control_state,LongCtrlState.pid)
    self.assertGreater(value,0.)

  def test_reappearing_stop_brakes_without_waiting_for_release(self):
    ctl,cs=setup_controller();cs.vEgo=.05
    for _ in range(100):ctl.update(True,cs,0.,True,(-2.5,2.5))
    for _ in range(45):ctl.update(True,cs,.14,False,(-2.5,2.5))
    before=ctl.last_output_accel
    after=ctl.update(True,cs,-1.,True,(-2.5,2.5))
    self.assertEqual(ctl.long_control_state,LongCtrlState.stopping)
    self.assertLess(after,before)
  def test_disengagement_clears_hold(self):
    ctl,cs=setup_controller();cs.vEgo=0.
    ctl.update(True,cs,0.,True,(-2.5,2.5))
    self.assertEqual(ctl.update(False,cs,0.,False,(-2.5,2.5)),0.)
    self.assertFalse(ctl.stop_release.holding)
    self.assertGreater(ctl.update(True,cs,.14,False,(-2.5,2.5)),0.)
  def test_moving_stop_can_clear_without_near_stop_latch(self):
    s=StopRelease();s.update(True,10.,True,-1.,False,False,.01)
    self.assertFalse(s.update(True,10.,False,-.1,False,False,.01))
  def test_brake_and_stock_standstill_prevent_release(self):
    for brake,standstill in ((True,False),(False,True)):
      s=StopRelease();s.update(True,0.,True,0.,False,False,.01)
      for _ in range(100):self.assertTrue(s.update(True,0.,False,.2,brake,standstill,.01))

class CaptureContinuity(unittest.TestCase):
  def test_recorded_capture_is_inside_comfort_limit(self):
    s=UnifiedCruiseSmoother();previous=.52447474
    # Route 379, 6:07:47.961 to 6:07:48.162: installed output fell to zero.
    for speed,accel in [(25.598,1.618),(25.755,1.94),(25.814,1.947),(25.941,2.165),(26.036,2.068)]:
      value=s.update(.524,previous,.05,2,speed,26.8224,accel,-2.5,.54)
      self.assertGreaterEqual(value,previous-.02000001)
      previous=value
    self.assertGreater(previous,.4)
  def test_forecast_braking_and_overspeed_are_explicit_exceptions(self):
    s=UnifiedCruiseSmoother()
    self.assertEqual(s.update(-1.,.5,.05,2,20.,25.,0.,-2.5,.8,'PLANNED_MODEL_BRAKING'),-1.)
    self.assertEqual(s.update(.5,.5,.05,2,26.,25.,0.,-2.5,.8),0.)
  def test_personality_change_retains_history_when_limits_allow(self):
    s=UnifiedCruiseSmoother();previous=.2
    for p in [1,2]*20:
      value=s.update(.4,previous,.05,p,10.,20.,0.,-2.5,.5)
      self.assertLessEqual(abs(value-previous),.02000001)
      previous=value

class PlannerPath(unittest.TestCase):
  setUp=base.PlannerTransitions.setUp
  tick=base.PlannerTransitions.tick
  def prepare(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    self.sm['selfdriveState'].personality=2
    self.sm['carState'].vEgo=4.;self.sm['carState'].aEgo=0.
    self.sm['radarState'].leadTwo=N(status=False)
    self.sm['modelV2'].velocity=N(t=[0.,1.,3.,6.,10.],x=[4.]*5)
    self.sm['modelV2'].action.desiredAcceleration=.2
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs=[1.,1.]
    self.p.corner_assist.target_speed=None
    self.mpc.source=lp.LongitudinalPlanSource.cruise
    self.plan.return_value=(.2,False)
  def test_low_speed_modes_use_same_handoff(self):
    self.prepare();owner=self.p.cruise_smoother
    for mode in [False,True]*50:
      self.sm['selfdriveState'].experimentalMode=mode
      previous=self.p.output_a_target;value=self.tick()
      self.assertIs(owner,self.p.cruise_smoother)
      self.assertEqual(self.p.handoff_reason,'ORDINARY')
      self.assertLessEqual(abs(value-previous),.02000001)
  def test_common_planner_feedback_and_actuator_transition_path(self):
    from opendbc.car.tesla.preap.virtual_das import VirtualDAS
    import math
    self.prepare();self.sm['carState'].vEgo=10.
    self.sm['modelV2'].velocity.x=[10.]*5
    ctl,cs=setup_controller();cs.vEgo=10.;cs.aEgo=0.
    with patch('opendbc.car.tesla.preap.virtual_das.get_zero_torque') as zero:
      zero.return_value.get.return_value=0.
      actuator=VirtualDAS(dt=.02);previous_actuator=0.
      for i in range(400):
        self.sm['selfdriveState'].experimentalMode=bool((i//40)%2)
        personality=1 if (i//80)%2 else 2
        self.sm['selfdriveState'].personality=personality
        target=.2 if (i//100)%2==0 else 0.
        self.sm['modelV2'].action.desiredAcceleration=target
        self.plan.return_value=(target,False)
        previous=self.p.output_a_target;planned=self.tick()
        self.assertLessEqual(abs(planned-previous),.02500001)
        for _ in range(5):
          command=ctl.update(True,cs,planned,False,(-2.5,2.5))
        actuator.update(command,cs.vEgo,actuator.prev_pedal_di,personality=personality)
        actual_command=actuator.jerk_limiter.a_limited
        self.assertTrue(math.isfinite(actuator.prev_pedal_di))
        self.assertLessEqual(abs(actual_command-previous_actuator),.05000001)
        previous_actuator=actual_command

  def test_coast_preference_is_not_negative_physical_cap(self):
    self.prepare();self.sm['carState'].vEgo=10.;self.sm['modelV2'].velocity.x=[10.]*5
    self.sm['carControl'].orientationNED=[0.,0.,0.]
    self.p.output_a_target=.2
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs=[0.,0.]
    value=self.tick()
    self.assertGreaterEqual(self.mpc.update.call_args.kwargs['max_accel'],0.)
    self.assertAlmostEqual(value,.18)
  def test_personality_change_blends_comfort_ceiling(self):
    self.prepare();self.sm['carState'].vEgo=10.;self.sm['selfdriveState'].personality=1
    self.sm['modelV2'].velocity.x=[10.]*5
    self.sm['modelV2'].action.desiredAcceleration=1.
    self.plan.return_value=(1.,False)
    for _ in range(100):self.tick()
    previous=self.p.output_a_target
    self.assertGreater(previous,.8)
    self.sm['selfdriveState'].personality=2
    for _ in range(100):
      value=self.tick()
      self.assertGreaterEqual(value,previous-.02000001)
      previous=value
    self.assertLessEqual(value,.54)
  def test_low_speed_stop_keeps_priority(self):
    self.prepare();self.sm['modelV2'].action.shouldStop=True
    self.plan.return_value=(-1.,True);self.p.output_a_target=.2
    self.assertEqual(self.tick(),-1.)
    self.assertTrue(self.p.output_should_stop)
