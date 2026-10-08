"""Cross-module engagement/turn regression coverage; no vehicle I/O."""
import json, math, tempfile
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import MagicMock, patch
import pytest
from cereal import car
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap.carcontroller import PreAPLongController
from openpilot.selfdrive.controls.lib.map_stop import MapStop
from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnEntry, ManualTurnCommit, NavigationTurnCommit
from openpilot.selfdrive.controls.lib.turn_context import driver_braking, manual_turn_intent
from openpilot.selfdrive.controls.lib.longitudinal_planner import map_curve_speed_cap, navigation_speed_cap


def nav(now,d=7):
 return dict(version=4,enabled=True,route_active=True,route_state='active',route_id='r',received_mono=now,expires_mono=now+2,position_quality=dict(projection_valid=True,path_ambiguous=False,heading_error_deg=2,match_error_m=1,gps_accuracy_m=3,gps_age_s=.1),mapped_stop_sign=dict(id='123',distance_m=d),maneuver=dict(type='turn',modifier='left',distance_m=100))
def status(now,seq=0,paused=False,session='s'):
 return dict(session=session,sequence=seq,cruise=True,paused=paused,brake=False,healthy=True,enabled=True,received_mono=now,expires_mono=now+.5)
def stop_tick(f,now,d=7,speed=2,seq=0,paused=False,gas=False,session='s'):
 return f.update(nav(now,d),status(now,seq,paused,session),enabled=True,active=not paused,inputs_valid=True,v_ego=speed,gas=gas,now=now)
def paused():
 e=PreAPEngagement(False,400);e.cruiseEnabled=True;e.brake_paused_long=True;e.resume_target_kph=50;return e

@pytest.mark.parametrize('speed',[0.,2.,10/2.236936])
def test_resume_on_press_once_and_no_release_ack(speed):
 e=paused();assert e.process_brake_resume(1000,speed,False,True,True,True,True,True)
 assert e.enableLongControl and e.pedal_speed_kph==50 and e.resume_sequence==1
 assert not e.process_brake_resume(1050,speed,False,True,True,True,True,True)
 assert not e.process_brake_resume(1100,speed,False,False,True,True,True,True)
 assert e.resume_sequence==1

@pytest.mark.parametrize('field', ['brake','unhealthy','fast','disabled','not_cruise','bad_transform'])
def test_resume_interlocks_still_require_fresh_press(field):
 e=paused();e.cruiseEnabled=field!='not_cruise'
 assert not e.process_brake_resume(1000,5 if field=='fast' else 0,field=='brake',True,field!='unhealthy',field!='bad_transform',True,field!='disabled')
 assert not e.enableLongControl
 assert not e.process_brake_resume(1100,0,False,True,True,True,True,True)

@pytest.mark.parametrize('speed',[0.,2.])
def test_map_stop_consumes_press_ack_while_driver_gas_is_active(speed):
 f=MapStop();stop_tick(f,10,30,5);stop_tick(f,10.1,speed=speed,paused=True)
 assert stop_tick(f,10.2,speed=speed,seq=1,gas=True)==(None,False)
 assert '123' in f.consumed
 assert stop_tick(f,10.3,speed=speed,seq=1)==(None,False)

def test_resume_does_not_consume_unpaused_stop_or_restart_token():
 f=MapStop();stop_tick(f,10,30,5);stop_tick(f,10.1,seq=1)
 assert not f.consumed
 f=MapStop();stop_tick(f,10,30,5);stop_tick(f,10.1,speed=0,paused=True)
 stop_tick(f,10.2,speed=0,seq=1,session='new');assert not f.consumed

def test_consumed_id_releases_both_caps_but_not_next_stop_or_curve():
 with tempfile.TemporaryDirectory() as td,patch('openpilot.selfdrive.controls.lib.longitudinal_planner.mapped_stop_setting_enabled',return_value=True):
  p=Path(td)/'map';obj=dict(version=3,enabled=True,valid=True,control_valid=True,status='ok',mono_time=10,expires_mono=12,match=dict(distance_m=1,heading_error_deg=2,confidence=.9),lookahead_m=100,control_curves=[],events=[dict(type='stop_sign',node_id=123,distance_m=7)])
  p.write_text(json.dumps(obj));assert map_curve_speed_cap(str(p),10,2) is not None
  assert map_curve_speed_cap(str(p),10,2,consumed_stops={'123'}) is None
  assert map_curve_speed_cap(str(p),10,2,include_stops=False) is None
  obj['events'].append(dict(type='stop_sign',node_id=456,distance_m=20));p.write_text(json.dumps(obj))
  assert map_curve_speed_cap(str(p),10,2,consumed_stops={'123'}) is not None

def cs():
 return N(vEgo=5,leftBlinker=True,rightBlinker=False,steeringPressed=False,steeringDisengage=False,brakePressed=False,brake=0,canValid=True,gearShifter='drive',leftBlindspot=False,rightBlindspot=False)
def decision(now=100,frame=10):
 return dict(received_mono=now,frame_id=frame,model_evaluated=True,manual_turn_direction=1,city_turns_enabled=True,signal_owner='driver',tap_signal_active=False,synthetic_indicator_request='none',manual_turn_blocked=False,source='none',final_desire='none')
def model(frame=10,heading=15,offset=1.5):
 return N(frameId=frame,position=N(t=[0,1,2,3],x=[0,4,8,12],y=[0,0,-offset/2,-offset]),orientation=N(z=[0,0,-math.radians(heading)/2,-math.radians(heading)]),action=N(desiredCurvature=0))

def test_earlier_conditioning_keeps_strict_steering_entry_gate():
 m=model();assert ManualTurnEntry.obvious_future_corner(m,-1)
 assert not ManualTurnCommit.directional_entry_evidence(m,-1,5)
 assert not ManualTurnEntry.obvious_future_corner(model(heading=2,offset=2.5),-1)

def test_future_turn_requires_speed_intent_dwell_and_fresh_geometry():
 m=model();e=ManualTurnEntry()
 for i in range(7):m.frameId+=1;value=e.update(m,1,5,True,100+i*.05)
 assert value
 assert not e.update(m,1,12,True,100.4)
 e=ManualTurnEntry()
 for i in range(7):m.frameId+=1;assert not e.update(m,1,5,False,100+i*.05)

@pytest.mark.parametrize('change',[dict(brake=1),dict(rightBlinker=True),dict(steeringPressed=True),dict(leftBlindspot=True)])
def test_shared_intent_gates(change):
 c=cs();assert manual_turn_intent(c,decision(),model(),100)
 for k,v in change.items():setattr(c,k,v)
 assert not manual_turn_intent(c,decision(),model(),100)

def test_unpaired_decision_cannot_veto_model():
 with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
  n=NavigationTurnCommit()
 n._refresh=lambda _:None;n.enabled=False;n.decision=decision(frame=1)
 assert n.update(.01,cs(),True,100,model=model(frame=100),healthy=True,actual_curvature=0)==.01

def test_driver_help_cancels_held_turn_until_intent_reset():
 m=model(heading=25,offset=2.5);e=ManualTurnEntry()
 for i in range(7):m.frameId+=1;e.update(m,1,5,True,100+i*.05)
 assert e.started is not None
 assert not e.update(m,1,5,False,100.4,driver_assisting=True)
 assert e.blocked and e.started is None
 for i in range(7):
  m.frameId+=1
  assert not e.update(m,1,5,True,100.5+i*.05)
 e.reset()  # Physical neutral reset supplied by DesireHelper.
 for i in range(7):m.frameId+=1;value=e.update(m,1,5,True,101+i*.05)
 assert value
 # Completion/invalid cancellation still latches until physical intent is cleared.
 e.cancel();assert e.blocked
 assert not e.update(m,1,5,False,101,driver_assisting=True)
 assert e.blocked

def pedal_cs(gas=False):
 return N(cruiseEnabled=True,enableLongControl=True,out=N(vEgo=5,aEgo=.4,gasPressed=gas),pedal_interceptor_value=6.,pedal_timeout=False,pedal=N(torque_level=0),cruise_buttons=0,prev_cruise_buttons=0,pccEvent=None)

def test_pedal_press_passthrough_then_release_picks_up_current_state():
 from opendbc.car.tesla.preap import carcontroller as ccmod
 cc=PreAPLongController();cc.vdas=MagicMock();cc.vdas.update.return_value=6.
 can=MagicMock();can.create_pedal_command.side_effect=lambda cmd,enable:dict(cmd=cmd,enable=enable)
 conf=N(use_pedal=True,pedal_factor=1,di_to_pedal=lambda d:d)
 cs_=pedal_cs(True);control=N(longActive=False,actuators=N(accel=.4),orientationNED=[])
 with patch.object(ccmod,'nap_conf',conf),patch.object(ccmod,'get_zero_torque') as z:
  z.return_value.get.return_value=4
  sends=cc.update(control,cs_,0,can,0)
  assert sends[-1]['enable']==0
  assert not cc.vdas.update.called
  cs_.out.gasPressed=False;cs_.pedal_interceptor_value=5.;control.longActive=True
  sends=cc.update(control,cs_,2,can,0)
  assert sends[-1]['enable']==1
  cc.vdas.reset.assert_called_with(a_init=.4,pedal_di_init=5.)
  assert cc.vdas.update.call_args.args[0]==.4
  assert cc.vdas.update.call_args.kwargs['freeze_integrator'] is False

def test_brake_pause_never_enables_pedal_output():
 from opendbc.car.tesla.preap import carcontroller as ccmod
 c=PreAPLongController();c.vdas=MagicMock();can=MagicMock();can.create_pedal_command.side_effect=lambda cmd,enable:dict(cmd=cmd,enable=enable)
 cs_=pedal_cs();cs_.enableLongControl=False
 with patch.object(ccmod,'nap_conf',N(use_pedal=True,pedal_factor=1,di_to_pedal=lambda d:d)),patch.object(ccmod,'get_zero_torque'):
  sends=c.update(N(longActive=False,actuators=N(accel=1),orientationNED=[]),cs_,0,can,0)
  assert sends[-1]['enable']==0 and not c.vdas.update.called

def test_steering_handback_does_not_wait_for_wheel_to_straighten():
 from opendbc.car.tesla import carcontroller as ccmod
 c=ccmod.CarController.__new__(ccmod.CarController)
 c.CP=N(openpilotLongitudinalControl=False);c.frame=0;c.apply_angle_last=25.;c.VM=MagicMock()
 c.tesla_can=MagicMock();c.stock_cc=N(update=lambda *a:[],pcc_event=None)
 c.parked_signal_test=N(update=lambda *a:[]);c.tap_lane_change=N(update=lambda *a,**kw:[])
 c.navigation_signal=N(update=lambda *a,**kw:[]);c.tap_params=None
 c.params=N(get_bool=lambda _:True);c.low_speed_steering_rate_enabled=True
 out=N(vEgo=4.,vEgoRaw=4.,leftBlinker=True,rightBlinker=False,steeringPressed=True,steeringAngleDeg=25.,canValid=True)
 cs_=N(out=out,hands_on_level=2,tap_stalk=N())
 actuator=MagicMock();actuator.steeringAngleDeg=30.
 control=N(actuators=actuator,latActive=True)
 with patch.object(ccmod,'apply_steer_angle_limits_vm',side_effect=lambda target,previous,*a:target) as apply:
  c._update_preap(control,cs_)
  assert not apply.call_args.args[4]
  out.steeringPressed=False;cs_.hands_on_level=0
  for _ in range(22):c._update_preap(control,cs_)
  assert apply.call_args.args[4]
  assert out.steeringAngleDeg==25 and actuator.steeringAngleDeg==30
  assert any(call.args[4] and call.args[1]==25 for call in apply.call_args_list)  # First active handback starts from measured angle.
  out.steeringPressed=True;cs_.hands_on_level=2
  c._update_preap(control,cs_);c._update_preap(control,cs_)
  assert not apply.call_args.args[4]

def test_odd_frame_release_still_rebases_before_first_applied_command():
 from opendbc.car.tesla.preap import carcontroller as ccmod
 c=PreAPLongController();c.vdas=MagicMock();c.vdas.update.return_value=6.
 can=MagicMock();cs_=pedal_cs(True);control=N(longActive=False,actuators=N(accel=.5),orientationNED=[])
 with patch.object(ccmod,'nap_conf',N(use_pedal=True,pedal_factor=1,di_to_pedal=lambda d:d)),patch.object(ccmod,'get_zero_torque') as z:
  z.return_value.get.return_value=4.
  c.update(control,cs_,0,can,0)
  cs_.out.gasPressed=False;cs_.out.aEgo=.5;cs_.pedal_interceptor_value=5.;control.longActive=True
  c.update(control,cs_,1,can,0)
  c.update(control,cs_,2,can,0)
  c.vdas.reset.assert_called_with(a_init=.5,pedal_di_init=5.)


def test_actual_controls_gate_pauses_outer_feedback_and_preserves_other_cars():
 import ast
 from openpilot.selfdrive.controls.tests.test_preap_longcontrol import setup_controller
 source=Path(__file__).parents[1]/'controlsd.py';tree=ast.parse(source.read_text())
 method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and any(isinstance(x,ast.Assign) and ast.unparse(x.targets[0])=='CC.longActive' for x in n.body))
 start=next(i for i,n in enumerate(method.body) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='CC.longActive')
 end=next(i for i,n in enumerate(method.body[start:],start) if isinstance(n,ast.Assign) and ast.unparse(n.targets[0])=='actuators')
 code=compile(ast.fix_missing_locations(ast.Module(body=[n for n in method.body[start:end] if not (isinstance(n,ast.If) and "turn_steering_gate" in ast.unparse(n))],type_ignores=[])),str(source),'exec')
 def run(fingerprint,paused):
  cp=N(carFingerprint=fingerprint,openpilotLongitudinalControl=True,pcmCruise=False)
  cc=N(enabled=True);cs_=N(pedalLongActive=not paused)
  env=dict(self=N(CP=cp,sm={'onroadEvents':[]}),CC=cc,CS=cs_)
  exec(code,env);return cc.longActive
 assert not run('TESLA_MODEL_S_PREAP',True)
 assert run('TESLA_MODEL_S_PREAP',False)
 assert run('OTHER_CAR',True)
 controller,cs_=setup_controller();controller.pid.i=.4
 assert controller.update(run('TESLA_MODEL_S_PREAP',True),cs_,.3,False,(-2.5,.8))==0.
 assert controller.pid.i==0.


def test_resume_press_transition_emits_existing_engagement_chime():
 import ast
 from openpilot.selfdrive.selfdrived.events import EVENTS,EventName,ET,AudibleAlert,Events
 tree=ast.parse((Path(__file__).parents[2]/'selfdrived/selfdrived.py').read_text())
 block=next(n for n in ast.walk(tree) if isinstance(n,ast.If) and 'pedal_long_active =' in ast.unparse(n) and 'self.prev_pedal_long_active' in ast.unparse(n) and 'self.CP.brand' in ast.unparse(n.test))
 code=compile(ast.fix_missing_locations(ast.Module(body=[block],type_ignores=[])),'engagement-events','exec')
 s=N(CP=N(brand='tesla',carFingerprint='TESLA_MODEL_S_PREAP',openpilotLongitudinalControl=True,pcmCruise=False),prev_pedal_long_active=False,events=Events())
 cs_=N(cruiseState=N(enabled=True),pedalLongActive=False,pedalMaxRegen=False)
 e=paused();assert e.process_brake_resume(1000,0,False,True,True,True,True,True)
 cs_.pedalLongActive=e.enableLongControl
 exec(code,dict(self=s,CS=cs_,EventName=EventName))
 assert EventName.pedalCruiseEnabled in s.events.names
 assert EVENTS[EventName.pedalCruiseEnabled][ET.WARNING].audible_alert==AudibleAlert.engage
 s.events=Events();exec(code,dict(self=s,CS=cs_,EventName=EventName));assert EventName.pedalCruiseEnabled not in s.events.names


def test_settings_api_reports_and_writes_effective_param_authority():
 from phone_navigation import turn_settings as settings
 from openpilot.common.params import Params
 with tempfile.TemporaryDirectory() as td:
  p=Params(str(Path(td)/'params'));p.put_bool('NAPCornerAssist',True)
  f=Path(td)/'settings';f.write_text('{"corner_assist":false,"manual_blinker_turns":false,"blinker_braking":false}')
  with patch.object(settings,'PATH',f),patch.object(settings,'require_park'),patch('openpilot.common.params.Params',return_value=p):
   assert settings.read()['corner_assist'] is True
   assert settings.write(dict(name='corner_assist',enabled=False))['corner_assist'] is False
   assert p.get_bool('NAPCornerAssist') is False
   assert settings.write(dict(name='manual_blinker_turns',enabled=True))['manual_blinker_turns'] is True
   assert p.get_bool('NAPCityTurns') is True


def test_corner_assistance_honors_classified_intent_brake_and_takeover():
 from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist
 c=cs();c.vEgo=8;c.steeringAngleDeg=20;c.gasPressed=False
 m=model();m.position=N(x=[0,10,20],y=[0,0,-10])
 class SM(dict):pass
 sm=SM(carState=c,carControl=N(latActive=True,longActive=True),modelV2=m)
 sm.valid=sm.alive={k:True for k in sm};sm.logMonoTime={k:int(100e9) for k in sm}
 a=CornerAssist();a.config['corner_assist']=True
 with patch('openpilot.selfdrive.controls.lib.corner_assist.time.monotonic',return_value=100):
  assert a.apply(.2,sm,N(steerRatio=15,wheelbase=2.96),False,-.3,10,decision())<.2
  c.brake=1;assert a.apply(.2,sm,N(steerRatio=15,wheelbase=2.96),False,-.3,10,decision())==.2
  c.brake=0;c.steeringPressed=True;assert a.apply(.2,sm,N(steerRatio=15,wheelbase=2.96),False,-.3,10,decision())==.2
  c.steeringPressed=False;d=decision();d['source']='tap_lane_change'
  assert a.apply(.2,sm,N(steerRatio=15,wheelbase=2.96),False,-.3,10,d)==.2


def test_carried_direction_veto_is_bounded_and_cannot_create_a_turn():
 with patch('openpilot.selfdrive.controls.lib.navigation_turn_commit.Params'):
  n=NavigationTurnCommit()
 n._refresh=lambda _:None;n.enabled=False;n.decision=decision(frame=10)
 assert n.update(.01,cs(),True,100,model=model(frame=10),healthy=True,actual_curvature=0)==0
 n.decision=decision(now=100,frame=10)
 assert n.update(.01,cs(),True,100.1,model=model(frame=20),healthy=True,actual_curvature=0)==0
 assert n.update(.01,cs(),True,100.16,model=model(frame=20),healthy=True,actual_curvature=0)==.01
 assert not n.manual.active
