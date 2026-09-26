"""Offline planner/actuator transition regressions; no CAN output."""
import unittest
from contextlib import ExitStack
from types import SimpleNamespace as N
from unittest.mock import patch, MagicMock
import numpy as np
from cereal import log
import cereal.messaging as messaging
from opendbc.car.tesla.preap.accel_policy import accel_ceiling, limit_positive_accel
from opendbc.car.tesla.preap import interface
from opendbc.car.tesla.preap.virtual_das import VirtualDAS
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc

class Policy(unittest.TestCase):
  def test_relaxed_lowest(self):
    for v in np.linspace(0,40,161):
      self.assertLess(accel_ceiling(v,2),accel_ceiling(v,1))
      self.assertLessEqual(accel_ceiling(v,2),.58)

  def test_interface_matches(self):
    with patch.object(interface,'_params') as params:
      for p in range(3):
        params.get.return_value=str(p)
        for v in (0.,1.3,7.5,15.,30.):
          self.assertEqual(interface.get_preap_accel_limits(v),(-2.5,accel_ceiling(v,p)))
    self.assertEqual(accel_ceiling(5,log.LongitudinalPersonality.relaxed),accel_ceiling(5,2))

  def test_reduced_demand_immediate(self):
    for request in (-3.,-1.,0.,.1):
      self.assertEqual(limit_positive_accel(request,.4,10.,2,.05),request)

  def test_actuator_rise_and_braking(self):
    with patch('opendbc.car.tesla.preap.virtual_das.get_zero_torque') as zero:
      zero.return_value.get.return_value=0.
      for p,rise in ((2,.2),(1,.4),(0,.6)):
        vdas=VirtualDAS(dt=.02)
        for i in range(20):
          vdas.update(2.,10.,vdas.prev_pedal_di,personality=p)
          self.assertAlmostEqual(vdas.jerk_limiter.a_limited,(i+1)*rise*.02)
        previous=vdas.jerk_limiter.a_limited
        vdas.update(-2.,10.,vdas.prev_pedal_di,personality=p)
        self.assertAlmostEqual(vdas.jerk_limiter.a_limited,previous-2.5*.02)

class PlannerTransitions(unittest.TestCase):
  def setUp(self):
    stack=ExitStack();self.addCleanup(stack.close)
    def mock(name,**kwargs):return stack.enter_context(patch.object(lp,name,**kwargs))
    params=mock('Params');params.return_value.get_bool.return_value=False;params.return_value.get.return_value=5
    self.mpc=mock('LongitudinalMpc').return_value
    self.mpc.v_solution=np.full(len(lp.T_IDXS_MPC),10.)
    self.mpc.a_solution=np.zeros(len(lp.T_IDXS_MPC));self.mpc.j_solution=np.zeros(len(lp.T_IDXS_MPC)-1)
    self.mpc.crash_cnt=0
    mock('CornerAssist').return_value.apply.side_effect=lambda a,*args:a
    mock('read_snapshot',return_value=None)
    self.plan=mock('get_accel_from_plan',return_value=(1.5,False))
    cp=N(brand='tesla',carFingerprint='TESLA_MODEL_S_PREAP',openpilotLongitudinalControl=True,
         pcmCruise=False,steerRatio=15.,wheelbase=2.96,longitudinalActuatorDelay=.4,vEgoStopping=.1)
    class SM(dict):
      def all_checks(self,service_list=None):return True
    self.sm=SM(carState=N(vEgo=10.,vCruise=90.,standstill=False,aEgo=0.,steeringAngleDeg=0.,gasPressed=False),
      carControl=N(orientationNED=[]),controlsState=N(longControlState=lp.LongCtrlState.pid,forceDecel=False),
      liveParameters=N(angleOffsetDeg=0.),radarState=N(leadOne=N(status=False)),
      selfdriveState=N(enabled=True,personality=2,experimentalMode=False),
      modelV2=N(position=N(x=[]),meta=N(disengagePredictions=N(gasPressProbs=[])),
                action=N(desiredAcceleration=1.,shouldStop=False)))
    self.p=lp.LongitudinalPlanner(cp)
    self.p.map_stop=N(update=lambda *a,**kw:(None,False))

  def tick(self):
    self.p.update(self.sm)
    return self.p.output_a_target

  def test_lead_cruise_model_switches_keep_ramp(self):
    previous=0.
    for i in range(100):
      self.sm['radarState'].leadOne.status=bool(i%3==0)
      self.sm['selfdriveState'].experimentalMode=bool(i%3==2)
      current=self.tick()
      self.assertGreaterEqual(current,previous)
      self.assertLessEqual(current-previous,.01000001)
      self.assertLessEqual(current,accel_ceiling(10.,2))
      self.assertEqual(self.mpc.update.call_args.kwargs['max_accel'],accel_ceiling(10.,2))
      self.assertTrue(self.mpc.update.call_args.kwargs['preap_accel_profile'])
      previous=current
    self.assertAlmostEqual(previous,accel_ceiling(10.,2))

  def test_model_lower_demand_and_cruise_recovery(self):
    for _ in range(60):self.tick()
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['modelV2'].action.desiredAcceleration=-.2
    self.assertAlmostEqual(self.tick(),-.2)
    self.sm['selfdriveState'].experimentalMode=False
    self.assertAlmostEqual(self.tick(),.01)

  def test_lead_braking_bypasses_phantom_buffer(self):
    for _ in range(60):self.tick()
    self.plan.return_value=(-2.,True)
    self.assertEqual(self.tick(),-2.)
    self.assertTrue(self.p.output_should_stop)

  def test_model_braking_and_stop(self):
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['modelV2'].action.desiredAcceleration=-2.
    self.sm['modelV2'].action.shouldStop=True
    self.assertEqual(self.tick(),-2.)
    self.assertTrue(self.p.output_should_stop)

  def test_override_restarts_ramp(self):
    for _ in range(60):self.tick()
    self.sm['carState'].gasPressed=True
    self.assertAlmostEqual(self.tick(),.01)
    self.sm['carState'].gasPressed=False
    self.assertAlmostEqual(self.tick(),.02)

  def test_lower_personality_cap_immediate(self):
    self.sm['selfdriveState'].personality=1
    for _ in range(100):self.tick()
    self.assertGreater(self.p.output_a_target,accel_ceiling(10.,2))
    self.sm['selfdriveState'].personality=2
    self.assertLessEqual(self.tick(),accel_ceiling(10.,2))

  def test_standstill_launch(self):
    self.sm['carState'].vEgo=0.;self.sm['carState'].standstill=True
    for i in range(40):
      self.assertLessEqual(self.tick(),min(.18,(i+1)*.01)+1e-9)
    self.assertAlmostEqual(self.p.output_a_target,.18)

class SolverIntegration(unittest.TestCase):
  def test_real_mpc_lead_and_cruise_cap(self):
    radar=messaging.new_message('radarState').radarState
    for lead in (False,True):
      radar.leadOne.status=lead;radar.leadOne.dRel=70.;radar.leadOne.vLead=20.
      radar.leadOne.aLeadTau=1.5;radar.leadOne.modelProb=1.
      mpc=LongitudinalMpc()
      for _ in range(8):
        mpc.set_weights(True,personality=2);mpc.set_cur_state(10.,0.)
        mpc.update(radar,25.,personality=2,nap_follow_dist=5,max_accel=.45,preap_accel_profile=True)
      self.assertEqual(mpc.solution_status,0)
      self.assertTrue(np.all(mpc.params[:,1]==.45))
      # MPC bounds are soft costs; the planner and actuator enforce hard caps.
      self.assertTrue(np.all(np.isfinite(mpc.a_solution)))
      self.assertGreater(float(np.max(mpc.a_solution)),.1)



class ControllerIntegration(unittest.TestCase):
  def test_gas_override_and_recovery_keep_driver_authority(self):
    from opendbc.car.tesla.preap import carcontroller as cc
    with patch.object(cc,'nap_conf') as conf, patch.object(cc,'get_zero_torque') as zero, \
         patch.object(cc,'get_preap_personality',return_value=2), \
         patch.object(cc,'get_preap_accel_limits',return_value=(-2.5,.45)):
      conf.use_pedal=True;conf.pedal_factor=1.;conf.di_to_pedal.side_effect=lambda v:v
      zero.return_value.get.return_value=0.
      cs=N(cruiseEnabled=True,enableLongControl=True,pedal_interceptor_value=0.,
           out=N(vEgo=10.,aEgo=0.,gasPressed=False),cruise_buttons=0,prev_cruise_buttons=0,
           pedal_timeout=False,pedal=N(torque_level=0.))
      control=N(longActive=True,actuators=N(accel=1.),orientationNED=[])
      can=MagicMock();controller=cc.PreAPLongController()
      controller.update(control,cs,0,can,0)
      for frame in range(2,100,2):controller.update(control,cs,frame,can,0)
      self.assertLessEqual(controller.vdas.jerk_limiter.a_limited,.2)
      cs.out.gasPressed=True
      controller.update(control,cs,100,can,0)
      self.assertEqual(can.create_pedal_command.call_args.kwargs['enable'],0)
      self.assertEqual(controller.vdas.jerk_limiter.a_limited,0.)
      cs.out.gasPressed=False
      controller.update(control,cs,102,can,0)
      self.assertEqual(can.create_pedal_command.call_args.kwargs['enable'],1)
      self.assertAlmostEqual(controller.vdas.jerk_limiter.a_limited,.004)
      cs.enableLongControl=False
      controller.update(control,cs,104,can,0)
      self.assertEqual(can.create_pedal_command.call_args.kwargs['enable'],0)


if __name__=='__main__':
  unittest.main()
