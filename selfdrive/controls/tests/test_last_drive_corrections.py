import ast, inspect, math, json
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch, MagicMock
from cereal import car, log
from openpilot.selfdrive.controls.lib.turn_steering_gate import TurnSteeringGate, matching_signal
from opendbc.car.tesla.preap.accel_policy import accel_ceiling, limit_positive_accel


def cs(signal=0):
 return N(vEgo=5.,leftBlinker=signal==1,rightBlinker=signal==2,steeringPressed=False)


def model(curvature=0.,left=0.,right=0.):
 return N(action=N(desiredCurvature=curvature),meta=N(desireState=[1-left-right,left,right,0,0,0,0,0]))


def test_tight_turn_and_model_memory_require_matching_indicator():
 g=TurnSteeringGate();c=cs()
 assert g.allowed(c,model(.005))
 assert not g.allowed(c,model(-.075,left=1))
 assert not g.allowed(c,model(0,left=1))
 c.leftBlinker=True;assert g.allowed(c,model(-.075,left=1))
 assert not g.allowed(c,model(.075,right=1))
 c.rightBlinker=True;assert not g.allowed(c,model(-.075,left=1))
 c.vEgo=30.;assert g.allowed(c,model(-.075,left=1))


def test_override_cancels_held_turn_but_ordinary_tracking_returns():
 g=TurnSteeringGate();c=cs(1);m=model(-.05,left=1)
 assert g.allowed(c,m)
 c.steeringPressed=True;assert not g.allowed(c,m)
 c.steeringPressed=False
 for _ in range(100):assert not g.allowed(c,m)
 assert g.allowed(c,model(.001))
 assert not g.allowed(c,model(.001),-.05)
 c.leftBlinker=False;assert g.allowed(c,model())
 c.leftBlinker=True;assert g.allowed(c,m)


def test_recorded_unsignaled_left_turn_regression():
 # Last-drive t=296.14: model retains turnLeft after the lamp clears.
 c=cs();c.vEgo=6.137
 assert not TurnSteeringGate().allowed(c,model(-.075,left=1.))


def test_relaxed_lower_launch_and_city_rise_other_profiles_unchanged():
 for speed in (0.,1.3,5.,7.5,8.9408,11.176):
  assert accel_ceiling(speed,2)<=.35
  assert limit_positive_accel(1.,0.,speed,2,.05)<=.009000001
 assert accel_ceiling(22.352,2)==.56
 assert limit_positive_accel(1.,0.,25.,2,.05)==.015
 assert accel_ceiling(7.5,0)==1.1
 assert accel_ceiling(7.5,1)==1.
 assert limit_positive_accel(1.,0.,5.,0,.05)==.03
 assert math.isclose(limit_positive_accel(1.,0.,5.,1,.05),.02)
 assert limit_positive_accel(-2.,.3,5.,2,.05)==-2.


def test_actual_hso_yields_to_level_one_with_torque_and_returns_from_measured_angle():
 from opendbc.car.tesla import carcontroller as module
 c=module.CarController.__new__(module.CarController)
 c.frame=2;c.CP=N(openpilotLongitudinalControl=False);c.apply_angle_last=90.;c.VM=None
 c.params=N(get_bool=lambda _:True);c.low_speed_steering_rate_enabled=True;c.tap_params=None
 c.tesla_can=MagicMock();c.stock_cc=N(update=lambda *a:[],pcc_event=None)
 c.parked_signal_test=N(update=lambda *a:[]);c.tap_lane_change=N(update=lambda *a,**k:[]);c.navigation_signal=N(update=lambda *a,**k:[])
 state=N(out=N(vEgo=2.,vEgoRaw=2.,leftBlinker=True,rightBlinker=False,steeringPressed=False,steeringAngleDeg=90.,steeringTorque=2.6,canValid=True),hands_on_level=1,tap_stalk=N())
 builder=car.CarControl.new_message();builder.latActive=True;builder.actuators.steeringAngleDeg=120.;cc=builder.as_reader()
 with patch.object(module,'apply_steer_angle_limits_vm',side_effect=lambda target,*args:target) as limit:
  state.out.steeringTorque=1.2
  for _ in range(4):c._update_preap(cc,state)
  assert c.tesla_can.create_steering_control.call_args.args[-1] is True
  state.out.steeringTorque=2.6
  for _ in range(60):c._update_preap(cc,state)
  assert c.tesla_can.create_steering_control.call_args.args[-1] is False
  assert c.low_speed_turn_takeover_latched
  state.hands_on_level=0
  for _ in range(24):c._update_preap(cc,state)
  assert c.tesla_can.create_steering_control.call_args.args[-1] is True
  assert not c.low_speed_turn_takeover_latched
  # Indicator clear removes the boosted exit-rate allowance immediately.
  state.out.leftBlinker=False;c._update_preap(cc,state);c._update_preap(cc,state)
  assert limit.call_args.args[5] is module.CarControllerParams


def test_raw_hso_entry_is_two_frames_not_half_second():
 from opendbc.car.tesla.preap import carstate as module
 from opendbc.car.interfaces import CarStateBase
 tree=ast.parse(inspect.getsource(module.update_preap))
 block=next(n for n in tree.body[0].body if isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='ENABLE_HSO')
 code=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'hso-entry','exec')
 class State:
  update_steering_pressed=CarStateBase.update_steering_pressed
 c=State();c.steering_pressed_cnt=0;c.hands_on_level=1
 ret=N(vEgo=2.,steeringTorque=2.6,steeringPressed=False)
 from opendbc.car.tesla.preap.steering_override import hands_on_override
 env=dict(cs=c,ret=ret,ENABLE_HSO=True,STEER_THRESHOLD=1.,hands_on_override=hands_on_override)
 ret.steeringTorque=1.2
 for _ in range(3):exec(code,env)
 assert not ret.steeringPressed
 ret.steeringTorque=2.6
 for _ in range(3):exec(code,env)
 assert ret.steeringPressed
 c.hands_on_level=0;ret.steeringTorque=.5;exec(code,env)
 assert not ret.steeringPressed


def test_navigation_announces_then_waits_for_actual_matching_lamp():
 import tempfile
 from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire
 state=dict(enabled=True,route_active=True,received_mono=100.,expires_mono=103.,route_state='active',route_id='test',maneuver_id='turn1',position_quality=dict(match_error_m=2.,gps_accuracy_m=3.,gps_age_s=.1),maneuver=dict(type='turn',modifier='left',distance_m=10.))
 with tempfile.TemporaryDirectory() as td:
  n=NavigationDesire(str(Path(td)/'nav'),str(Path(td)/'diag'),str(Path(td)/'signal'));n.params=N(get_bool=lambda _:True)
  c=cs();c.steeringDisengage=False;c.gearShifter='drive';c.brakePressed=False
  def tick():
   n.prepared_navigation=(state,'');return n.update(c,N(latActive=True),True,now=101.,physical_direction=0)
  assert tick()=='none'
  assert n.indicator_request=='left'
  c.leftBlinker=True;assert tick()=='turnLeft'
  c.steeringPressed=True;assert tick()=='none';assert n.blocked
  c.steeringPressed=False;assert tick()=='none'
