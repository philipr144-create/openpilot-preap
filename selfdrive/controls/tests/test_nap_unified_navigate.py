"""Comprehensive offline tests for NAP unified turning, exit assist, and grade compensation."""
import json
import math
import os
import sys
import tempfile
import time
import unittest
from types import SimpleNamespace as N
from unittest.mock import MagicMock

# Mock compiled/external dependencies for pure offline testing
sys.modules.setdefault('msgq', MagicMock())
sys.modules.setdefault('openpilot.common.params_pyx', MagicMock())
sys.modules.setdefault('openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code', MagicMock())
sys.modules.setdefault('openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx', MagicMock())
sys.modules.setdefault('casadi', MagicMock())

from openpilot.selfdrive.controls.lib.longcontrol import LongControl, LongCtrlState
from openpilot.selfdrive.controls.lib.predictive_grade_assist import PredictiveGradeAssist
from openpilot.selfdrive.controls.lib.turn_context import classify_route_maneuver, route_reference_speed
from openpilot.selfdrive.controls.lib.turn_propulsion import TurnPropulsionPlan
from openpilot.selfdrive.controls.lib.longitudinal_planner import navigation_speed_cap, map_curve_speed_cap
from openpilot.selfdrive.nap.nap_mapd import find_upcoming_speed_limit


def setup_controller(fingerprint="TESLA_MODEL_S_PREAP"):
  cp = N(carFingerprint=fingerprint, startingState=False, vEgoStarting=.1,
         stopAccel=-1.5, stoppingDecelRate=1.0, startAccel=.5,
         longitudinalTuning=N(kpBP=[0., 3., 6., 35.], kpV=[0., 0., 0., 0.],
                              kiBP=[0., 3., 6., 35.], kiV=[.04, .06, .10, .15]))
  cs = N(vEgo=0., aEgo=0., brakePressed=False, cruiseState=N(standstill=False))
  return LongControl(cp), cs


def make_model_with_pitch(pitch_rise_deg=1.5):
  """Create a synthetic modelV2 object with an uphill pitch profile."""
  xs = list(range(0, 100, 5))  # 0 to 95 meters
  # Linearly rise from 0 to pitch_rise_deg over the distance
  pitch_rad = math.radians(pitch_rise_deg)
  ys = [(x / 95.0) * pitch_rad for x in xs]
  return N(
    position=N(x=xs),
    orientation=N(y=ys),
    action=N(shouldStop=False)
  )


class TestNAPUnifiedNavigate(unittest.TestCase):

  # ─── 1. Stop-to-Turn Launch Assist ───

  def test_stop_to_turn_launch_floor_active(self):
    tp = TurnPropulsionPlan()
    now = 100.0
    # Car stopped at standstill (speed=0.0 m/s), wheel turned 35 deg, base accel is sluggish (0.0 m/s^2)
    req = tp.update(
      0.0, speed=0.0, target=7.0, actual_curvature=0.01,
      mpc_accel=0.5, model_accel=0.1, upper_accel=1.5, now=now,
      ready=True, blocked=False, profile_owned=False,
      turn_assist_enabled=True, steer_angle_deg=35.0
    )
    # Floor should provide positive launch propulsion between 0.18 and 0.35 m/s^2, capped at 0.40
    self.assertGreaterEqual(req, 0.18)
    self.assertLessEqual(req, 0.40)
    self.assertEqual(tp.diagnostic['reason'], 'LAUNCH_TURN_ASSIST')
    self.assertTrue(tp.diagnostic['supporting'])

  def test_stop_to_turn_toggle_off_fallback(self):
    tp = TurnPropulsionPlan()
    now = 100.0
    # Same condition, but turn_assist_enabled=False (fallback)
    req = tp.update(
      0.0, speed=0.5, target=7.0, actual_curvature=0.01,
      mpc_accel=0.5, model_accel=0.1, upper_accel=1.5, now=now,
      ready=True, blocked=False, profile_owned=False,
      turn_assist_enabled=False, steer_angle_deg=35.0
    )
    # When toggle is off, base_accel=0.0 < WEAK_COAST_MIN or normal PI applies without launch floor
    self.assertNotEqual(tp.diagnostic.get('reason'), 'LAUNCH_TURN_ASSIST')

  # ─── 2. Moving Turn Speed Maintenance & Friction Circle ───

  def test_moving_turn_speed_maintenance_floor(self):
    tp = TurnPropulsionPlan()
    now = 100.0
    # Car moving at 6.0 m/s (~13.5 mph), target 7.0 m/s, profile_owned=True, curvature 0.02 (R=50m)
    # Base acceleration sags to -0.1 m/s^2 from model hesitation
    req = tp.update(
      -0.1, speed=6.0, target=7.0, actual_curvature=0.02,
      mpc_accel=0.3, model_accel=0.0, upper_accel=1.5, now=now,
      ready=True, blocked=False, profile_owned=True,
      turn_assist_enabled=True, steer_angle_deg=20.0
    )
    self.assertGreater(req, 0.0)
    self.assertEqual(tp.diagnostic['reason'], 'TURN_SPEED_MAINTENANCE')
    self.assertTrue(tp.diagnostic['supporting'])

    # Verify ISO friction circle comfort limit remaining
    lat_accel = 6.0**2 * 0.02  # 0.72 m/s^2
    max_long = math.sqrt(max(0.0, 1.8**2 - lat_accel**2))
    self.assertLessEqual(req, max_long)

  def test_moving_turn_blocked_when_lead_or_stop_present(self):
    tp = TurnPropulsionPlan()
    now = 100.0
    # When blocked (e.g. lead vehicle stopping), turn propulsion must immediately bail to base
    req = tp.update(
      -1.0, speed=6.0, target=7.0, actual_curvature=0.02,
      mpc_accel=-1.0, model_accel=-1.0, upper_accel=1.5, now=now,
      ready=True, blocked=True, profile_owned=True,
      turn_assist_enabled=True, steer_angle_deg=20.0
    )
    self.assertEqual(req, -1.0)
    self.assertEqual(tp.diagnostic['reason'], 'BLOCKED_OR_UNAVAILABLE')

  # ─── 3. Highway Exit Deceleration & Turn Context ───

  def test_route_reference_speed_highway_exit(self):
    off_ramp = {'type': 'off ramp', 'modifier': 'right'}
    fork_ramp = {'type': 'off ramp', 'modifier': 'slight right'}
    other = {'type': 'turn', 'modifier': 'right'}

    # Highway exit assist enabled
    self.assertAlmostEqual(route_reference_speed(off_ramp, highway_exit_enabled=True), 15.65)  # 35 mph
    self.assertAlmostEqual(route_reference_speed(fork_ramp, highway_exit_enabled=True), 20.1)  # 45 mph
    self.assertEqual(route_reference_speed(other, highway_exit_enabled=True), 7.0)

    # Highway exit assist disabled (fallback)
    self.assertIsNone(route_reference_speed(off_ramp, highway_exit_enabled=False))
    self.assertIsNone(route_reference_speed(fork_ramp, highway_exit_enabled=False))

  def test_navigation_speed_cap_off_ramp(self):
    now = time.monotonic()
    # Create temporary nav snapshot JSON for off ramp at 200m
    snap = {
      'enabled': True,
      'route_state': 'active',
      'received_mono': now,
      'expires_mono': now + 1.0,
      'version': 4,
      'position_quality': {
        'projection_valid': True,
        'path_ambiguous': False,
        'heading_error_deg': 2.0,
        'match_error_m': 3.0,
        'gps_accuracy_m': 2.0,
        'gps_age_s': 0.1
      },
      'maneuver': {
        'type': 'off ramp',
        'modifier': 'right',
        'distance_m': 200.0
      }
    }
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
      f.write(json.dumps(snap))
      temp_path = f.name

    try:
      # Enabled: should produce deceleration speed cap targeting 15.65 m/s
      cap_enabled = navigation_speed_cap(
        path=temp_path, now=now, v_ego=31.0, highway_exit_enabled=True
      )
      self.assertIsNotNone(cap_enabled)
      # At 200m distance with 30m buffer: sqrt(15.65^2 + 2 * 1.0 * 170) ~ 24.2 m/s (< 31 m/s)
      self.assertLess(cap_enabled, 31.0)
      self.assertGreater(cap_enabled, 15.65)

      # Disabled: returns None (100% fallback to cruise)
      cap_disabled = navigation_speed_cap(
        path=temp_path, now=now, v_ego=31.0, highway_exit_enabled=False
      )
      self.assertIsNone(cap_disabled)
    finally:
      if os.path.exists(temp_path):
        os.remove(temp_path)

  # ─── 4. Anticipatory Speed Limit Transitions (Feedforward) ───

  def test_find_upcoming_speed_limit(self):
    current_way = N(id=101, tags={'maxspeed': '55 mph'})
    next_way = N(id=102, tags={'maxspeed': '35 mph'})
    ways = {101: current_way, 102: next_way}
    geom = [
      {'distance': 0.0, 'way_id': 101},
      {'distance': 50.0, 'way_id': 101},
      {'distance': 150.0, 'way_id': 102},
      {'distance': 200.0, 'way_id': 102},
    ]
    res = find_upcoming_speed_limit(geom, ways, current_way)
    self.assertIsNotNone(res)
    self.assertAlmostEqual(res['target_mps'], 35.0 / 2.236936, places=2)
    self.assertEqual(res['target_mph'], 35.0)
    self.assertEqual(res['distance_m'], 150.0)

  def test_map_curve_speed_cap_anticipatory_speed_drop(self):
    now = time.monotonic()
    # Create temporary map context snapshot with upcoming speed limit drop from 55 mph to 35 mph at 200m
    snap = {
      'enabled': True,
      'valid': True,
      'control_valid': True,
      'status': 'ok',
      'version': 3,
      'mono_time': now,
      'expires_mono': now + 1.0,
      'lookahead_m': 300.0,
      'match': {
        'distance_m': 2.0,
        'heading_error_deg': 1.0,
        'confidence': 0.95
      },
      'upcoming_speed_limit': {
        'target_mps': 15.65,  # 35 mph
        'target_mph': 35.0,
        'distance_m': 200.0,
        'current_mps': 24.58  # 55 mph
      }
    }
    with tempfile.NamedTemporaryFile('w', suffix='.json', delete=False) as f:
      f.write(json.dumps(snap))
      temp_path = f.name

    try:
      # Enabled: should calculate smooth approach cap toward 15.65 m/s
      # At 200m with 15m buffer: sqrt(15.65^2 + 2 * 0.65 * 185) ~ 22.0 m/s (~49 mph)
      cap_enabled = map_curve_speed_cap(
        path=temp_path, now=now, v_ego=24.58, include_curves=False,
        anticipatory_speed_limit_enabled=True
      )
      self.assertIsNotNone(cap_enabled)
      self.assertLess(cap_enabled, 24.58)
      self.assertGreater(cap_enabled, 15.65)

      # Disabled (UI toggle OFF): returns None (100% fallback)
      cap_disabled = map_curve_speed_cap(
        path=temp_path, now=now, v_ego=24.58, include_curves=False,
        anticipatory_speed_limit_enabled=False
      )
      self.assertIsNone(cap_disabled)
    finally:
      if os.path.exists(temp_path):
        os.remove(temp_path)

  # ─── 5. Predictive Road Grade / Model Pitch Compensation ───

  def test_predictive_grade_city_speed_and_curve(self):
    pga = PredictiveGradeAssist(dt=0.05)
    model = make_model_with_pitch(pitch_rise_deg=1.8)

    # City speed 6.0 m/s (~13.5 mph) climbing an uphill road with a curve cap: target=6.5, driver_target=15.0
    boost = 0.0
    for tick in range(25):  # 1.25 seconds to establish climb tracking
      boost = pga.update(
        model=model, enabled=True, engaged=True,
        v_ego=6.0, a_ego=0.0, effective_speed=6.5, driver_speed=15.0,
        base_accel=0.05, mpc_accel=0.2, model_accel=0.05,
        brake_reason='ORDINARY', model_stop=False, mpc_stop=False,
        corner_braking=False, force_decel=False,
        gas_pressed=False, brake_pressed=False,
        inputs_valid=True, forecast_valid=True,
        model_braking_required=False, max_accel=1.5
      )

    self.assertGreater(boost, 0.0)
    self.assertLessEqual(boost, 0.35)
    self.assertIn(pga.reason, ('APPLYING', 'CONFIRMING_UPHILL', 'TRACKING_CLIMB'))

  def test_predictive_grade_toggle_off_fallback(self):
    pga = PredictiveGradeAssist(dt=0.05)
    model = make_model_with_pitch(pitch_rise_deg=1.8)
    boost = pga.update(
      model=model, enabled=False, engaged=True,
      v_ego=6.0, a_ego=0.0, effective_speed=6.5, driver_speed=15.0,
      base_accel=0.05, mpc_accel=0.2, model_accel=0.05,
      brake_reason='ORDINARY', model_stop=False, mpc_stop=False,
      corner_braking=False, force_decel=False,
      gas_pressed=False, brake_pressed=False,
      inputs_valid=True, forecast_valid=True,
      model_braking_required=False, max_accel=1.5
    )
    self.assertEqual(boost, 0.0)
    self.assertEqual(pga.reason, 'TOGGLE_OFF')

  # ─── 5. Responsive Stop Release on Positive Command ───

  def test_longcontrol_responsive_launch_from_stopping(self):
    controller, cs = setup_controller()
    # Step into stopping state
    for _ in range(10):
      controller.update(True, cs, -1.5, True, (-1.5, 0.8))
    self.assertEqual(controller.long_control_state, LongCtrlState.stopping)

    # Now should_stop is False, a_target is positive (0.35 m/s^2 from launch assist)
    # Step through 0.30s stop release debounce (30 ticks at DT_CTRL=0.01s)
    for _ in range(30):
      controller.update(True, cs, 0.35, False, (-1.5, 0.8))

    # 31st tick: stop_release clears, transitioning state to PID with releasing_stop=True
    out = controller.update(True, cs, 0.35, False, (-1.5, 0.8))
    self.assertEqual(controller.long_control_state, LongCtrlState.pid)
    # Responsive release rate (>= 2.5 m/s^2/s) should ramp output up faster than stock stoppingDecelRate
    self.assertGreater(out, -1.5)
    # After another 10 ticks (0.1s), output should have risen significantly (~0.25 m/s^2)
    for _ in range(10):
      out = controller.update(True, cs, 0.35, False, (-1.5, 0.8))
    self.assertGreater(out, -1.5 + 0.20)

  # ─── 6. Simultaneous Multi-Feature Integration & Constraint Authority Tests ───

  def test_simultaneous_lead_braking_with_turn_and_grade(self):
    """Verify that a braking lead vehicle completely suppresses turn floors and grade assist."""
    tp = TurnPropulsionPlan()
    pga = PredictiveGradeAssist(dt=0.05)
    model = make_model_with_pitch(pitch_rise_deg=2.0)
    lead = N(status=True, dRel=14.0, vRel=-3.5)

    lead_mpc_accel = -2.2  # Lead MPC is demanding hard braking
    v_ego = 12.0           # 27 mph
    now = 200.0

    # 1. Turn propulsion with active turn, BUT lead is present & braking -> blocked=True
    tp_out = tp.update(
      lead_mpc_accel, speed=v_ego, target=10.0, actual_curvature=0.008,
      mpc_accel=lead_mpc_accel, model_accel=-1.8, upper_accel=1.5, now=now,
      ready=True,
      blocked=True,  # planning_has_lead or lead_braking
      profile_owned=True, turn_assist_enabled=True, steer_angle_deg=35.0
    )
    # Turn propulsion must NOT apply any floor when blocked by a lead
    self.assertEqual(tp_out, lead_mpc_accel)
    # When blocked, upper limit is either unconstrained or <= lead_mpc_accel
    self.assertTrue(tp.upper is None or tp.upper <= lead_mpc_accel)

    # 2. Predictive grade assist under lead braking
    grade_boost = pga.update(
      model=model, enabled=True, engaged=True,
      v_ego=v_ego, a_ego=-1.0, effective_speed=15.0, driver_speed=15.0,
      base_accel=tp_out, mpc_accel=lead_mpc_accel, model_accel=-1.8,
      brake_reason='LEAD_BRAKING', model_stop=False, mpc_stop=False,
      corner_braking=False, force_decel=False,
      gas_pressed=False, brake_pressed=False,
      lead=lead, leads=(lead,),
      inputs_valid=True, forecast_valid=True,
      model_braking_required=False, max_accel=lead_mpc_accel
    )
    # Grade boost must be strictly 0.0 under lead braking
    self.assertEqual(grade_boost, 0.0)
    self.assertIn('LEAD_NEAR_OR_CLOSING', pga.block_reasons)

    # 3. Combined output remains the lead MPC braking request
    combined_accel = tp_out + grade_boost
    self.assertEqual(combined_accel, -2.2)

  def test_simultaneous_navigation_exit_with_turn_and_grade(self):
    """Verify that highway off-ramp deceleration overrules turn propulsion and grade assist."""
    tp = TurnPropulsionPlan()
    pga = PredictiveGradeAssist(dt=0.05)
    model = make_model_with_pitch(pitch_rise_deg=1.8)

    # Approaching off-ramp: reference speed is 35 mph (15.65 m/s)
    off_ramp = {'type': 'off ramp', 'modifier': 'right', 'instruction': 'Take exit 42'}
    v_cruise_cap = route_reference_speed(off_ramp, highway_exit_enabled=True)
    self.assertAlmostEqual(v_cruise_cap, 15.65)  # 35 mph

    driver_target = 31.3  # 70 mph
    decel_base_accel = -1.2
    now = 300.0

    # 1. Turn propulsion must be blocked by speed ceiling decel / corner braking
    tp_out = tp.update(
      decel_base_accel, speed=29.0, target=v_cruise_cap, actual_curvature=0.005,
      mpc_accel=-1.2, model_accel=-1.0, upper_accel=0.8, now=now,
      ready=True,
      blocked=True,  # blocked due to speed ceiling reduction / corner braking
      profile_owned=True, turn_assist_enabled=True, steer_angle_deg=28.0
    )
    self.assertEqual(tp_out, decel_base_accel)

    # 2. Grade assist cannot add throttle when target < driver_target - 0.25 (SPEED_RESTRICTION)
    grade_boost = pga.update(
      model=model, enabled=True, engaged=True,
      v_ego=29.0, a_ego=-0.5, effective_speed=v_cruise_cap, driver_speed=driver_target,
      base_accel=tp_out, mpc_accel=-1.2, model_accel=-1.0,
      brake_reason='SPEED_CEILING', model_stop=False, mpc_stop=False,
      corner_braking=True, force_decel=False,
      gas_pressed=False, brake_pressed=False,
      inputs_valid=True, forecast_valid=True,
      model_braking_required=False, max_accel=decel_base_accel
    )
    self.assertEqual(grade_boost, 0.0)
    self.assertIn('SPEED_RESTRICTION', pga.block_reasons)

    # Combined output continues smooth deceleration toward the off-ramp
    combined_accel = tp_out + grade_boost
    self.assertLess(combined_accel, 0.0)

  def test_simultaneous_stop_constraint_suppresses_launch_and_grade(self):
    """Verify that red light / stop sign / shouldStop suppresses launch floor and grade assist."""
    tp = TurnPropulsionPlan()
    pga = PredictiveGradeAssist(dt=0.05)
    model = make_model_with_pitch(pitch_rise_deg=1.5)
    controller, cs = setup_controller()

    # Step into stopping state at an intersection (160 ticks = 1.6s to ramp full stop brake -1.5 m/s^2)
    for _ in range(160):
      controller.update(True, cs, -1.5, True, (-1.5, 0.8))
    self.assertEqual(controller.long_control_state, LongCtrlState.stopping)

    # Stopped at 0 mph with wheel turned 40 degrees facing an uphill slope
    stop_accel = -1.5
    now = 400.0

    # 1. Turn propulsion receives blocked=True because output_should_stop=True
    tp_out = tp.update(
      stop_accel, speed=0.0, target=6.0, actual_curvature=0.015,
      mpc_accel=-1.5, model_accel=-1.5, upper_accel=1.5, now=now,
      ready=True,
      blocked=True,  # STOP active
      profile_owned=False, turn_assist_enabled=True, steer_angle_deg=40.0
    )
    # Launch floor (0.18 - 0.35 m/s^2) MUST NOT be applied while stopped at red light
    self.assertEqual(tp_out, stop_accel)

    # 2. Grade assist receives model_stop=True / mpc_stop=True
    grade_boost = pga.update(
      model=model, enabled=True, engaged=True,
      v_ego=0.0, a_ego=0.0, effective_speed=6.0, driver_speed=15.0,
      base_accel=tp_out, mpc_accel=-1.5, model_accel=-1.5,
      brake_reason='STOP', model_stop=True, mpc_stop=True,
      corner_braking=False, force_decel=False,
      gas_pressed=False, brake_pressed=False,
      inputs_valid=True, forecast_valid=True,
      model_braking_required=True, max_accel=stop_accel
    )
    self.assertEqual(grade_boost, 0.0)
    self.assertIn('MODEL_STOP', pga.block_reasons)

    # 3. LongControl controller remains in stopping state with full holding brake applied
    out = controller.update(True, cs, tp_out + grade_boost, True, (-1.5, 0.8))
    self.assertEqual(controller.long_control_state, LongCtrlState.stopping)
    self.assertEqual(out, -1.5)

  def test_simultaneous_unobstructed_turn_and_grade_coordination(self):
    """Verify that in free driving (no lead, no stop, no speed drop), both features coordinate within friction circle."""
    tp = TurnPropulsionPlan()
    pga = PredictiveGradeAssist(dt=0.05)
    model = make_model_with_pitch(pitch_rise_deg=1.6)

    # Clean uphill curve: v_ego = 8.0 m/s (18 mph), steer = 25 deg, curvature = 0.008 (R ~ 125m)
    # Lateral accel = v^2 * kappa = 64 * 0.008 = 0.512 m/s^2
    v_ego = 8.0
    kappa = 0.008
    a_lat = v_ego**2 * kappa
    now = 500.0

    # Base model output is weak coasting (0.02 m/s^2)
    base_accel = 0.02
    tp_out = tp.update(
      base_accel, speed=v_ego, target=10.0, actual_curvature=kappa,
      mpc_accel=0.3, model_accel=0.02, upper_accel=1.5, now=now,
      ready=True, blocked=False, profile_owned=True,
      turn_assist_enabled=True, steer_angle_deg=25.0
    )
    # Turn propulsion provides moving turn speed maintenance floor
    self.assertGreater(tp_out, base_accel)
    self.assertGreaterEqual(tp_out, 0.08)

    # Grade assist establishes climb tracking
    grade_boost = 0.0
    for _ in range(25):
      grade_boost = pga.update(
        model=model, enabled=True, engaged=True,
        v_ego=v_ego, a_ego=0.0, effective_speed=10.0, driver_speed=15.0,
        base_accel=tp_out, mpc_accel=0.3, model_accel=tp_out,
        brake_reason='ORDINARY', model_stop=False, mpc_stop=False,
        corner_braking=False, force_decel=False,
        gas_pressed=False, brake_pressed=False,
        inputs_valid=True, forecast_valid=True,
        model_braking_required=False, max_accel=1.2
      )

    combined_accel = tp_out + grade_boost
    self.assertGreater(combined_accel, tp_out)
    # Friction circle check: total vector acceleration must be within comfort limit (<= 1.8 m/s^2)
    a_total = math.hypot(combined_accel, a_lat)
    self.assertLessEqual(a_total, 1.8)

  # ─── 4. Drive 44a Specific Regression Tests ───

  def test_stop_sign_and_traffic_signal_map_cap(self):
    """Verify that both stop_sign and traffic_signal produce approach speed caps."""
    now = 100.0
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json') as tf:
      # Map context snapshot with a traffic signal at 40m
      data = {
        'version': 3, 'enabled': True, 'valid': True, 'control_valid': True, 'status': 'ok',
        'mono_time': now, 'expires_mono': now + 1.0,
        'match': {'distance_m': 1.0, 'heading_error_deg': 2.0, 'confidence': 0.95},
        'lookahead_m': 200.0,
        'events': [
          {'type': 'traffic_signal', 'node_id': 101, 'distance_m': 40.0},
          {'type': 'stop_sign', 'node_id': 102, 'distance_m': 80.0},
        ]
      }
      tf.write(json.dumps(data))
      tf.flush()

      # map_curve_speed_cap should cap speed down toward MAPPED_STOP_APPROACH_SPEED
      cap = map_curve_speed_cap(path=tf.name, now=now, v_ego=10.0, include_curves=False)
      self.assertIsNotNone(cap)
      # 40m traffic signal cap: sqrt(2.23^2 + 2*0.8*(40-5)) ~ sqrt(5 + 56) ~ 7.8 m/s (~17 mph)
      self.assertLess(cap, 10.0)
      self.assertGreater(cap, 2.0)

  def test_speed_limit_transition_holds_current_road_limit(self):
    """Verify that current road speed limit prevents cruise ceiling from rebounding when upcoming drop expires."""
    now = 200.0
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json') as tf:
      # Vehicle crossed into the 45 mph zone (20.1 m/s); upcoming_speed_limit is None
      data = {
        'version': 3, 'enabled': True, 'valid': True, 'control_valid': True, 'status': 'ok',
        'mono_time': now, 'expires_mono': now + 1.0,
        'match': {'distance_m': 1.0, 'heading_error_deg': 2.0, 'confidence': 0.95},
        'lookahead_m': 200.0,
        'road': {'way_id': 999, 'speed_limit_mps': 20.1168, 'speed_limit_mph': 45.0},
        'upcoming_speed_limit': None,
      }
      tf.write(json.dumps(data))
      tf.flush()

      cap = map_curve_speed_cap(path=tf.name, now=now, v_ego=24.0, include_curves=False, anticipatory_speed_limit_enabled=True)
      self.assertIsNotNone(cap)
      # Cap must remain at 20.1168 m/s (45 mph), not None
      self.assertAlmostEqual(cap, 20.1168, places=3)

  def test_turn_propulsion_continuous_through_2_5_mps_transition(self):
    """Verify that taking off into a sharp turn maintains torque without stumbling when crossing 2.5 m/s."""
    tp = TurnPropulsionPlan()
    now = 300.0
    steer_angle = 50.0  # Sharp turn
    curvature = 0.015

    # 1. Standstill launch (speed = 1.0 m/s < 2.5 m/s)
    req_low = tp.update(
      0.05, speed=1.0, target=6.7, actual_curvature=curvature,
      mpc_accel=0.3, model_accel=0.05, upper_accel=1.0, now=now,
      ready=True, blocked=False, profile_owned=False,
      turn_assist_enabled=True, steer_angle_deg=steer_angle
    )
    self.assertGreater(req_low, 0.12)

    # 2. Crossing 2.5 m/s threshold (speed = 2.8 m/s, at 20 Hz dt = 0.05s)
    now += 0.05
    req_mid = tp.update(
      0.05, speed=2.8, target=6.7, actual_curvature=curvature,
      mpc_accel=0.3, model_accel=0.05, upper_accel=1.0, now=now,
      ready=True, blocked=False, profile_owned=False,
      turn_assist_enabled=True, steer_angle_deg=steer_angle
    )
    # Must maintain maintenance torque, NOT reset or drop to base 0.05
    self.assertGreater(req_mid, 0.12)
    self.assertEqual(tp.diagnostic['reason'], 'TURN_SPEED_MAINTENANCE')


if __name__ == '__main__':
  unittest.main()
