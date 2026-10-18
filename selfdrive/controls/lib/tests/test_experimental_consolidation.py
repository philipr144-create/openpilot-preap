"""Experimental recovery/capture regressions. Offline only; no CAN calls."""
import importlib.util
import os
from pathlib import Path
from types import SimpleNamespace as N
import unittest
from unittest.mock import patch
import numpy as np
from openpilot.selfdrive.controls.lib.acceleration_handoff import ExperimentalCruiseSmoother, handoff_acceleration
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as base
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


class Capture(unittest.TestCase):
  def test_positive_drive_eases_before_overshoot(self):
    s=ExperimentalCruiseSmoother()
    self.assertEqual(s.update(.4,.4,.05,1,19.9,20.,.5,-2.5,.8),0.)
    self.assertEqual(s.update(.08,.4,.05,1,18.,20.,0.,-2.5,.8),.08)

  def test_braking_priority_and_release_preserved(self):
    for priority in ('STOP','LEAD_BRAKING','STRONG_BRAKING','PLANNED_MODEL_BRAKING',
                     'SPEED_CEILING','CORNER_BRAKING','UNPROVEN_MODEL_BRAKING','INVALID_INPUT','FORCE_DECEL'):
      s=ExperimentalCruiseSmoother()
      self.assertEqual(s.update(-2.,.4,.05,2,19.9,20.,.5,-2.5,.8,priority),-2.)
      self.assertEqual(s.update(.2,-2.,.05,2,19.9,20.,.5,-2.5,.8,priority),
                       handoff_acceleration(.2,-2.,.05,2,-2.5,.8,priority))

  def test_disabled_capture_is_exact_existing_handoff(self):
    s=ExperimentalCruiseSmoother()
    for request in (-1.,-.1,0.,.1,.5):
      for previous in (-1.,-.1,0.,.1,.5):
        self.assertEqual(s.update(request,previous,.05,1,19.9,20.,.5,-2.5,.8,capture=False),
                         handoff_acceleration(request,previous,.05,1,-2.5,.8))


class Planner(unittest.TestCase):
  setUp=base.PlannerTransitions.setUp
  tick=base.PlannerTransitions.tick

  def prepare(self):
    self.p.nap_adaptive_accel=True
    self.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
    self.sm['selfdriveState'].experimentalMode=True
    self.sm['carState'].vEgo=20.
    self.sm['carState'].aEgo=-.8
    self.sm['radarState'].leadTwo=N(status=False)
    self.sm['modelV2'].velocity=N(t=[0.,1.,3.,6.,10.],x=[20.]*5)
    self.sm['modelV2'].action.desiredAcceleration=0.
    self.sm['modelV2'].meta.disengagePredictions.gasPressProbs=[1.,1.]
    self.plan.return_value=(.6,False)

  def test_one_recovery_never_modifies_raw_model(self):
    self.prepare()
    for _ in range(100):value=self.tick()
    self.assertEqual(self.p.cruise_refinement.support,0.)
    self.assertGreater(value,0.)
    self.assertLessEqual(value,.25)
    self.sm['modelV2'].action.desiredAcceleration=-.3
    previous=value
    self.assertLess(self.tick(),previous)
    for _ in range(100):value=self.tick()
    self.assertAlmostEqual(value,-.3)  # existing ordinary braking handoff

  def test_capture_and_relaxed_acceleration(self):
    self.prepare()
    self.sm['selfdriveState'].personality=2
    for _ in range(100):value=self.tick()
    self.assertGreater(value,.1)
    self.sm['carState'].vEgo=24.9
    self.sm['carState'].aEgo=.8
    # Set model positive, so capture (rather than model braking) removes drive.
    self.sm['modelV2'].action.desiredAcceleration=.4
    self.sm['modelV2'].velocity.x=[24.9]*5
    for _ in range(20):value=self.tick()
    self.assertEqual(value,0.)

  def test_leads_stops_and_model_braking_are_preserved(self):
    self.prepare()
    for case in ('lead1','lead2','modelstop','mpcstop','modelbrake','force'):
      with self.subTest(case=case):
        self.sm['radarState'].leadOne.status=case=='lead1'
        self.sm['radarState'].leadTwo.status=case=='lead2'
        self.sm['modelV2'].action.shouldStop=case=='modelstop'
        self.sm['controlsState'].forceDecel=case=='force'
        self.sm['modelV2'].action.desiredAcceleration=-1.2
        self.plan.return_value=(-1.3,case=='mpcstop')
        self.p.output_a_target=.4
        self.assertLessEqual(self.tick(),-1.2)


class ChillEquivalence(unittest.TestCase):
  def test_identical_normal_planner_outputs_against_installed_baseline(self):
    path=os.environ.get('EXP_BASELINE_PLANNER')
    if not path:self.skipTest('baseline supplied by validation runner')
    spec=importlib.util.spec_from_file_location('baseline_planner',path)
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    def trace(module):
      rng=np.random.default_rng(364)
      original=base.lp;base.lp=module
      fixture=base.PlannerTransitions()
      try:
        fixture.setUp()
        fixture.p.nap_adaptive_accel=True
        fixture.p._params.get_bool.side_effect=lambda k:k=='NAPAdaptiveAccel'
        fixture.sm['radarState'].leadTwo=N(status=False)
        fixture.mpc.source=module.LongitudinalPlanSource.cruise
        out=[]
        for i in range(1200):
          fixture.sm['selfdriveState'].personality=(i//400)%3
          fixture.sm['carState'].vEgo=float(rng.uniform(5.,26.))
          fixture.sm['carState'].aEgo=float(rng.uniform(-1.5,1.5))
          fixture.sm['modelV2'].action.desiredAcceleration=float(rng.uniform(-.7,.8))
          fixture.sm['modelV2'].velocity=N(t=[0.,1.,3.,6.,10.],x=[fixture.sm['carState'].vEgo]*5)
          fixture.sm['radarState'].leadOne.status=i%13==0
          fixture.sm['radarState'].leadTwo.status=i%17==0
          fixture.plan.return_value=(float(rng.uniform(-1.2,.8)),i%53==0)
          module.LongitudinalPlanner.update(fixture.p,fixture.sm)
          out.append((fixture.p.output_a_target,fixture.p.output_should_stop,fixture.p.handoff_reason))
        return out
      finally:
        fixture.doCleanups();base.lp=original
    self.assertEqual(trace(old),trace(lp))
