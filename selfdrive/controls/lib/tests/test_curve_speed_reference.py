"""Offline repeated-curve regression tests. No vehicle or CAN output."""
import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

import numpy as np
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist, LAT_ACCEL_TARGET
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp

MPH = .44704


class CornerReference(unittest.TestCase):
  def setUp(self):
    self.corner = CornerAssist()
    self.corner.config['corner_assist'] = True
    self.now = 100.

  def tick(self, speed, curvature=.02, base=0., cruise=55*MPH, active=True, coast=-.3):
    self.now += .05
    return self.corner.update(base, speed, curvature, coast, active, self.now, cruise)

  def test_no_coasting_at_or_below_target(self):
    target = math.sqrt(LAT_ACCEL_TARGET/.02)
    for coast in (-.4, -.3, 0., .3):
      for speed in (target, target-.1, 6.):
        for _ in range(100):
          self.assertEqual(self.tick(speed, coast=coast), 0.)

  def test_repeated_equal_bends_do_not_ratchet_target_or_speed(self):
    target = math.sqrt(LAT_ACCEL_TARGET/.02)
    speed = 10.
    for bend in range(4):
      for _ in range(600):
        accel = self.tick(speed)
        speed += accel*.05
      self.assertAlmostEqual(self.corner.target_speed, target)
      self.assertGreaterEqual(speed, target-.03)
      self.assertLess(speed, target+.03)
      for _ in range(10): self.tick(speed, curvature=0.)

  def test_old_negative_cap_cannot_brake_below_new_target(self):
    for _ in range(60): self.tick(10.)
    self.assertLess(self.corner.cap, 0.)
    self.assertEqual(self.tick(6.), 0.)
    self.assertGreater(self.tick(6., base=.6), 0.)

  def test_lower_cruise_wins_and_higher_cruise_never_forces_curve_speed(self):
    self.tick(8., cruise=5.)
    self.assertEqual(self.corner.target_speed, 5.)
    self.tick(8., cruise=30.)
    self.assertAlmostEqual(self.corner.target_speed, math.sqrt(1.2/.02))

  def test_tighter_bend_has_lower_absolute_target(self):
    for _ in range(60): self.tick(8., curvature=.01)
    first = self.corner.target_speed
    for _ in range(60): result = self.tick(8., curvature=.03)
    self.assertLess(self.corner.target_speed, first)
    self.assertLess(result, 0.)

  def test_lead_braking_not_stored_and_limits_preserved(self):
    for _ in range(40): self.assertEqual(self.tick(6., base=-2.), -2.)
    self.assertEqual(self.tick(6., base=0.), 0.)
    for _ in range(100): self.assertGreaterEqual(self.tick(15., curvature=.1), -.8)
    self.assertEqual(self.tick(8., active=False, base=.5), .5)
    self.assertIsNone(self.corner.cap)

  def test_invalid_inputs_release_only_corner_authority(self):
    for kwargs in ({'cruise':float('nan')}, {'curvature':float('nan')}, {'speed':float('nan')}):
      for _ in range(40): self.tick(10.)
      args=dict(speed=8.,base=-1.); args.update(kwargs)
      self.assertEqual(self.tick(**args), -1.)
      self.assertIsNone(self.corner.cap)


class PlannerReference(unittest.TestCase):
  def test_multiple_map_curves_use_driver_cruise_not_previous_slowed_speed(self):
    cp=N(brand='tesla',carFingerprint='TESLA_MODEL_S_PREAP',openpilotLongitudinalControl=True,
         pcmCruise=False,steerRatio=15.,wheelbase=2.96,longitudinalActuatorDelay=.4,vEgoStopping=.1)
    class SM(dict):
      def all_checks(self, service_list=None): return True
    sm=SM(carState=N(vEgo=55*MPH,vCruise=55*MPH*3.6,standstill=False,aEgo=0.,steeringAngleDeg=0.,gasPressed=False),
          carControl=N(orientationNED=[]),controlsState=N(longControlState=lp.LongCtrlState.pid,forceDecel=False),
          liveParameters=N(angleOffsetDeg=0.),radarState=N(leadOne=N(status=False)),
          selfdriveState=N(enabled=True,personality=2,experimentalMode=False),
          modelV2=N(position=N(x=[]),meta=N(disengagePredictions=N(gasPressProbs=[])),
                    action=N(desiredAcceleration=0.,shouldStop=False)))
    with tempfile.TemporaryDirectory() as tmp, patch.object(lp,'Params') as params, \
         patch.object(lp,'LongitudinalMpc') as factory, patch.object(lp,'CornerAssist') as corner, \
         patch.object(lp,'read_snapshot',return_value=None), \
         patch.object(lp,'get_accel_from_plan',return_value=(0.,False)), \
         patch.object(lp.time,'monotonic',return_value=10.), \
         patch.object(lp,'mapped_stop_setting_enabled',return_value=False):
      params.return_value.get_bool.side_effect=lambda key:key=='NAPMapDrivingAssist'
      params.return_value.get.return_value=None
      mpc=factory.return_value
      mpc.v_solution=np.full(len(lp.T_IDXS_MPC),20.)
      mpc.a_solution=np.zeros(len(lp.T_IDXS_MPC))
      mpc.j_solution=np.zeros(len(lp.T_IDXS_MPC)-1);mpc.crash_cnt=0
      corner.return_value.apply.side_effect=lambda a,*args:a
      planner=lp.LongitudinalPlanner(cp)
      planner.map_stop=N(update=lambda *args,**kwargs:(None,False))
      state=dict(version=3,enabled=True,valid=True,control_valid=True,status='ok',mono_time=10.,expires_mono=12.,
                 match=dict(distance_m=0.,heading_error_deg=0.,confidence=1.),lookahead_m=450.)
      path=Path(tmp)/'curves.json'
      real_cap=lp.map_curve_speed_cap
      def run(speed,targets,cruise=55):
        sm['carState'].vEgo=speed*MPH;sm['carState'].vCruise=cruise*MPH*3.6
        state['control_curves']=[dict(start_distance_m=12.+i*40,end_distance_m=42.+i*40,length_m=30.,
                                     target_speed_mps=t*MPH,curvature=1.8/(t*MPH)**2) for i,t in enumerate(targets)]
        path.write_text(json.dumps(state))
        with patch.object(lp,'map_curve_speed_cap',side_effect=lambda **kw:real_cap(path=path,now=10.,**kw)):
          planner.update(sm)
        # Curve assistant always receives the original driver setting.
        self.assertAlmostEqual(corner.return_value.apply.call_args.args[-1], cruise*MPH)
        return mpc.update.call_args.args[1]/MPH
      # With the shorter map-entry buffer, the vehicle is allowed to
      # remain slightly above the final mapped curve target while still
      # approaching the curve. The important invariant is that repeated
      # curves do not ratchet the target lower from the previously slowed
      # speed.
      for speed in (55,45,40,35,45):
        result = run(speed,[45,45])
        self.assertGreaterEqual(result, 45.)
        self.assertLess(result, 47.)

      # Exact on-road complaint: with a 45 mph MPP target, two consecutive
      # 40 mph bends retain the same absolute 40 mph corridor target. They do
      # not subtract again from the speed reached in the first bend.
      for speed in (45, 40, 39, 40):
        result = run(speed, [40, 40], cruise=45)
        self.assertGreaterEqual(result, 40.)
        self.assertLess(result, 42.)
      recovered = run(40, [], cruise=45)
      self.assertGreater(recovered, result)
      self.assertLessEqual((recovered-result)*MPH, .6*.05+1e-9)
      for _ in range(100): recovered = run(40, [], cruise=45)
      self.assertAlmostEqual(recovered,45.)

      # A genuinely tighter next bend must still command a lower ceiling.
      tight = run(45,[30])
      self.assertGreaterEqual(tight, 30.)
      self.assertLess(tight, 32.)

      relaxed = run(35,[45],cruise=40)
      self.assertGreater(relaxed,tight)
      self.assertLess(relaxed,40.)
      for _ in range(200): relaxed = run(35,[45],cruise=40)
      self.assertAlmostEqual(relaxed,40.)
      for _ in range(250): recovered = run(35,[])
      self.assertAlmostEqual(recovered,55.)  # Recovery retains the original driver target.
      sm['controlsState'].forceDecel=True
      self.assertEqual(run(45,[45]),0.)


if __name__ == '__main__':
  unittest.main()
