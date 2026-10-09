from types import SimpleNamespace as N
from unittest.mock import patch
from openpilot.selfdrive.controls.lib.normal_propulsion import NormalPropulsion
from openpilot.selfdrive.controls.lib.tests import test_preap_accel_transitions as fixtures
from openpilot.selfdrive.controls.lib import longitudinal_planner as lp


def forecast(accel=.6, speeds=None):
 return N(action=N(desiredAcceleration=accel,shouldStop=False),velocity=N(t=[0.,2.,5.,8.,10.],x=speeds or [10.,11.,12.,13.,14.]))


def test_normal_permission_requires_sustained_complete_nonstopping_forecast():
 s=NormalPropulsion(.05)
 for _ in range(5):assert not s.update(forecast(),10.,15.,enabled=True,valid=True,blocked=False)
 assert s.update(forecast(),10.,15.,enabled=True,valid=True,blocked=False)
 assert not s.update(forecast(),10.,15.,enabled=True,valid=True,blocked=True)
 for m in [forecast(-.1),forecast(speeds=[10.,9.,8.,5.,0.]),forecast(speeds=[30.,31.,32.,33.,34.])]:
  s=NormalPropulsion(.05)
  for _ in range(20):assert not s.update(m,10.,15.,enabled=True,valid=True,blocked=False)
 m=forecast();m.action.shouldStop=True
 assert not NormalPropulsion(.05).update(m,10.,15.,enabled=True,valid=True,blocked=False)


def test_real_normal_planner_recovers_permission_without_raising_accel_profile():
 t=fixtures.PlannerTransitions();t.setUp();t.sm['modelV2'].action=forecast().action;t.sm['modelV2'].velocity=forecast().velocity
 t.sm['carState'].vCruise=54.
 t.sm['selfdriveState'].experimentalMode=False;t.p.nap_adaptive_accel=True
 with patch.object(lp,'model_support_weight',return_value=1.),patch.object(lp,'normal_model_stop_enabled',return_value=True):
  # Keep raw model throttle denied, but prove validated permission can recover.
  t.p.parse_model=lambda _: ([0.]*33,[10.]*33,[0.]*33,[0.]*33,.3)
  for _ in range(6):t.tick()
  assert not t.p.allow_throttle
  assert t.p.propulsion_allowed and t.p.normal_propulsion_recovery
  t.sm['modelV2'].action.shouldStop=True;t.tick()
  assert not t.p.normal_propulsion_recovery


def test_city_controller_yields_and_blends_from_measured_angle_at_18_mph():
 from unittest.mock import MagicMock
 from cereal import car
 from opendbc.car.tesla import carcontroller as module
 c=module.CarController.__new__(module.CarController)
 c.frame=2;c.CP=N(openpilotLongitudinalControl=False);c.apply_angle_last=90.;c.VM=None
 c.params=N(get_bool=lambda _:True);c.low_speed_steering_rate_enabled=True;c.tap_params=None
 c.tesla_can=MagicMock();c.stock_cc=N(update=lambda *a:[],pcc_event=None)
 c.parked_signal_test=N(update=lambda *a:[]);c.tap_lane_change=N(update=lambda *a,**k:[]);c.navigation_signal=N(update=lambda *a,**k:[])
 state=N(out=N(vEgo=18*.44704,vEgoRaw=18*.44704,leftBlinker=True,rightBlinker=False,steeringPressed=True,steeringAngleDeg=90.,steeringTorque=3.5,canValid=True),hands_on_level=1,tap_stalk=N())
 builder=car.CarControl.new_message();builder.latActive=True;builder.actuators.steeringAngleDeg=120.;cc=builder.as_reader()
 with patch.object(module,'apply_steer_angle_limits_vm',side_effect=lambda target,*args:target):
  for _ in range(4):c._update_preap(cc,state)
  assert c.low_speed_turn_takeover_latched
  assert c.tesla_can.create_steering_control.call_args.args[-1] is False
  state.out.steeringPressed=False;state.out.steeringTorque=.1;state.hands_on_level=0
  for _ in range(18):c._update_preap(cc,state)
  assert c.low_speed_turn_takeover_latched
  for _ in range(3):c._update_preap(cc,state)
  assert not c.low_speed_turn_takeover_latched
  assert c.tesla_can.create_steering_control.call_args.args[-1] is True
  assert c.tesla_can.create_steering_control.call_args.args[1]==90.
  c._update_preap(cc,state);c._update_preap(cc,state)
  assert 90.<c.tesla_can.create_steering_control.call_args.args[1]<120.
  state.out.steeringTorque=3.5;state.hands_on_level=2
  for _ in range(2):c._update_preap(cc,state)
  assert c.tesla_can.create_steering_control.call_args.args[-1] is False


def test_normal_recovery_preserves_restrictive_lead_mpc_and_physical_brake():
 t=fixtures.PlannerTransitions();t.setUp();m=forecast()
 t.sm['modelV2'].action=m.action;t.sm['modelV2'].velocity=m.velocity
 t.sm['carState'].vCruise=54.;t.p.nap_adaptive_accel=True
 t.sm['radarState'].leadOne.status=True;t.plan.return_value=(-2.,True)
 t.p.parse_model=lambda _: ([0.]*33,[10.]*33,[0.]*33,[0.]*33,.3)
 for _ in range(6):output=t.tick()
 assert t.p.normal_propulsion_recovery
 assert output<=-2. and t.p.output_should_stop
 t.sm['carState'].brake=1.;t.tick();assert not t.p.normal_propulsion_recovery
