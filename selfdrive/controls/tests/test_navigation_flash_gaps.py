import copy,hashlib,math,unittest,json,tempfile
from pathlib import Path
from types import SimpleNamespace as N
from openpilot.selfdrive.modeld.navigation_city_turn import NavigationCityTurn
from openpilot.selfdrive.controls.lib.turn_context import navigation_signal_matches,navigation_turn_intent
from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnCommit
from openpilot.selfdrive.controls.lib.city_turn_plan import CityTurnPlan
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist
class SM(dict):pass
class Sim:
 def __init__(self,direction=1):
  self.nav=NavigationCityTurn();self.direction=direction;self.sign=-1 if direction==1 else 1;self.i=-1
  self.cs=N(vEgo=2.,canValid=True,gearShifter='drive',steeringPressed=False,steeringDisengage=False,leftBlindspot=False,rightBlindspot=False,leftBlinker=direction==1,rightBlinker=direction==2,gasPressed=False,brakePressed=False,brake=0.,steeringAngleDeg=0.)
  self.cc=N(latActive=True,longActive=True,enabled=True)
  self.model=N(frameId=0,position=N(t=[0.,1.,2.,3.],x=[0.,4.,8.,10.],y=[0.,self.sign*.5,self.sign*2.,self.sign*4.]),orientation=N(z=[0.,self.sign*.1,self.sign*.3,self.sign*.6]),action=N(desiredCurvature=self.sign*.03))
  self.plan=CityTurnPlan();self.executor=ManualTurnCommit();self.corner=CornerAssist();self.corner.config['corner_assist']=True
 def tick(self,lit=True,ack=True,ack_mutate=None,state_mutate=None,**kw):
  self.i+=1;self.now=100.+self.i*.05;self.model.frameId+=1
  self.cs.leftBlinker=bool(lit and self.direction==1);self.cs.rightBlinker=bool(lit and self.direction==2)
  for k,v in kw.items():setattr(self.cs,k,v)
  state=dict(version=4,enabled=True,route_active=True,route_state='active',route_id='route',maneuver_id='1',received_mono=self.now,expires_mono=self.now+2.,maneuver=dict(type='turn',modifier='left' if self.direction==1 else 'right',distance_m=10.),position_quality=dict(projection_valid=True,path_ambiguous=False,heading_error_deg=2.,match_error_m=2.,gps_accuracy_m=10.,gps_age_s=.2))
  if state_mutate:state_mutate(state)
  token=hashlib.sha256('|'.join((self.nav.controller,'route','1')).encode()).hexdigest()[:32]
  status=dict(time=self.now,request_key=token,route_id='route',maneuver_id='1',controller=self.nav.controller,direction=self.direction,active=True,sent_recent=True,physical_override=False)
  if ack_mutate:ack_mutate(status)
  actual=self.sign*.06 if self.i>=20 else 0.
  self.out=self.nav.step(state,10.,self.cs,self.cc,now=self.now,enabled=True,model=self.model,model_valid=True,physical_direction=0,signal_status=status if ack else None,actual_curvature=actual)
  r=self.out.get('request')
  self.decision=dict(received_mono=self.now,frame_id=self.model.frameId,model_evaluated=True,signal_owner='navigation',source='navigation_city_turn',city_turns_enabled=True,route_id='route',maneuver_id='1',tap_signal_active=False,final_desire=self.out['desire'],navigation_turn=self.out,manual_turn_direction=0)
  self.shared=self.plan.update(intent=self.nav.intent,model=self.model,cs=self.cs,now=self.now,enabled=True,inputs_valid=True,controller=self.nav.controller,direction=self.direction,owner='navigation',request=r)
  self.decision['city_turn_plan']=self.shared
  self.output=self.executor.update(0.,self.cs,True,self.decision,self.model,True,actual,self.now)
  sm=SM(carState=self.cs,carControl=self.cc,modelV2=self.model);sm.valid=sm.alive={k:True for k in sm};sm.logMonoTime={k:int(self.now*1e9) for k in sm}
  self.target=self.corner.prepare_shared(sm,N(steerRatio=15.,wheelbase=2.9),False,10.,self.decision,self.now)
  return self.out
 def arm(self):
  for _ in range(14):self.tick()
  assert self.out['desire'] in ('turnLeft','turnRight'),self.out
class FlashContract(unittest.TestCase):
 def test_four_flash_cycles_match_continuous_lamp_all_consumers(self):
  for direction in (1,2):
   a,b=Sim(direction),Sim(direction);ever_active=False;ever_plan=False
   for i in range(96):
    lit=i%24<14
    a.tick(lit=lit);b.tick(lit=True)
    self.assertEqual(a.out['desire'],b.out['desire'],(direction,i,a.out))
    self.assertEqual(a.out['indicator'],direction)
    self.assertFalse(a.out['lifecycle']['blocked'])
    self.assertEqual(a.shared['id'],b.shared['id']);self.assertEqual(a.shared['phase'],b.shared['phase'])
    self.assertAlmostEqual(a.output,b.output,places=10)
    self.assertEqual(a.executor.active,b.executor.active)
    self.assertEqual(a.target,b.target)
    ever_active|=a.executor.active;ever_plan|=a.target is not None
   self.assertTrue(ever_active,'Steering fixture never acquired');self.assertTrue(ever_plan,'Speed-plan fixture never acquired')
 def test_recorded_flash_cadence_preserves_all_consumers(self):
  # Raw GTW transitions on the final approach, rebased to its first lit sample.
  edges=[(0.,True),(.800,False),(1.310,True),(2.009,False),(2.500,True),(3.199,False),(3.700,True)]
  a,b=Sim(),Sim()
  for i in range(100):
   lit=next(v for t,v in reversed(edges) if t<=i*.05)
   a.tick(lit=lit);b.tick()
   self.assertEqual(a.out['desire'],b.out['desire'])
   self.assertEqual(a.shared['phase'],b.shared['phase'])
   self.assertAlmostEqual(a.output,b.output,places=10)
   self.assertEqual(a.target,b.target)
 def test_flashing_signal_cannot_invent_steering_geometry(self):
  s=Sim();s.model.position.y=[0.]*4;s.model.orientation.z=[0.]*4;s.model.action.desiredCurvature=0.
  for i in range(60):
   s.tick(lit=i%24<14)
   self.assertAlmostEqual(s.output,0.)
   self.assertFalse(s.executor.active)
 def test_real_navigation_signal_feedback_loop_survives_flashes(self):
  from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire
  from opendbc.car.tesla.preap.tap_lane_change import NavigationSignalController,PhysicalStalk,crc8
  with tempfile.TemporaryDirectory() as tmp:
   root=Path(tmp);n=NavigationDesire(str(root/'nav'),str(root/'diag'),str(root/'request'))
   n.signal_status_path=str(root/'status');n.params=N(get_bool=lambda k:True,get=lambda k:True)
   producer=NavigationSignalController(root/'request',root/'status');stalk=PhysicalStalk();s=Sim()
   for i in range(96):
    now=100.+i*.05;s.model.frameId+=1;s.cs.leftBlinker=i%24<14
    state=dict(version=4,enabled=True,route_active=True,route_state='active',route_id='route',maneuver_id='1',received_mono=now,expires_mono=now+2.,maneuver=dict(type='turn',modifier='left',distance_m=10.),position_quality=dict(projection_valid=True,path_ambiguous=False,heading_error_deg=2.,match_error_m=2.,gps_accuracy_m=10.,gps_age_s=.2))
    (root/'nav').write_text(json.dumps(state))
    desire=n.update(s.cs,s.cc,True,now=now,city_enabled=True,model=s.model,model_inputs_valid=True,physical_direction=0,actual_curvature=-.06 if i>=20 else 0.)
    raw=bytearray(8);raw[7]=crc8(raw[:7]);stalk.raw=bytes(raw);stalk.direction=0;stalk.time=now
    frames=producer.update(stalk,s.cs,lateral_active=True,overriding=False,now=now)
    self.assertEqual(n.indicator_request,'left',(i,n.decision))
    self.assertEqual([f[1][2]&3 for f in frames],[1],(i,n.decision))
    if i>=14:self.assertEqual(desire,'turnLeft',(i,n.decision))
 def test_ack_without_initial_lamp_never_authorizes(self):
  s=Sim()
  for _ in range(30):s.tick(lit=False)
  self.assertEqual(s.out['desire'],'none');self.assertFalse(s.out['request']['signal_verified'])
 def test_long_dark_revokes_and_cannot_rearm_same_maneuver(self):
  s=Sim();s.arm()
  for _ in range(18):s.tick(lit=False)
  self.assertEqual(s.out['indicator'],0)
  for _ in range(15):s.tick()
  self.assertEqual(s.out['desire'],'none');self.assertEqual(s.out['indicator'],0)
 def test_stale_missing_future_wrong_ack_revokes(self):
  for change in (None,lambda a:a.update(time=0.),lambda a:a.update(time=1000.),lambda a:a.update(controller='other'),lambda a:a.update(request_key='wrong'),lambda a:a.update(active=False),lambda a:a.update(sent_recent=False),lambda a:a.update(physical_override=True)):
   s=Sim();s.arm();s.tick(lit=False,ack=change is not None,ack_mutate=change)
   self.assertEqual(s.out['indicator'],0,change)
 def test_driver_hazards_opposite_and_vehicle_gates_revoke(self):
  for fields in (dict(steeringPressed=True),dict(steeringDisengage=True),dict(leftBlindspot=True),dict(canValid=False),dict(gearShifter='park'),dict(leftBlinker=True,rightBlinker=True),dict(rightBlinker=True)):
   s=Sim();s.arm();s.tick(lit=False,**fields)
   self.assertEqual(s.out['indicator'],0,fields)
 def test_route_gps_expiry_revoke(self):
  for change in (lambda x:x.update(enabled=False),lambda x:x.update(expires_mono=0.),lambda x:x['position_quality'].update(gps_age_s=2.),lambda x:x['position_quality'].update(path_ambiguous=True),lambda x:x.update(route_id='other')):
   s=Sim();s.arm();s.tick(lit=False,state_mutate=change);self.assertEqual(s.out['indicator'],0)
 def test_consumers_reject_stale_or_forged_dark_proof(self):
  s=Sim();s.arm();s.tick(lit=False)
  self.assertTrue(navigation_turn_intent(s.cs,s.decision,s.model,s.now))
  for field,value in (('signal_last_lamp_mono',0.),('signal_last_lamp_mono',float('nan')),('signal_last_lamp_mono',1000.),('signal_ack_mono',0.),('signal_ack_mono',1000.),('signal_source','physical_confirmation'),('signal_verified',False)):
   request=copy.deepcopy(s.out['request']);request[field]=value
   self.assertFalse(navigation_signal_matches(s.cs,request,s.now),(field,value))
 def test_restart_new_identity_cannot_inherit_dark_confirmation(self):
  s=Sim();s.arm();s.nav=NavigationCityTurn();s.tick(lit=False)
  self.assertEqual(s.out['desire'],'none');self.assertFalse(s.out['request']['signal_verified'])
 def test_conflicting_lamp_before_acquisition_never_announces(self):
  for fields in (dict(leftBlinker=True,rightBlinker=True),dict(rightBlinker=True)):
   s=Sim();s.tick(lit=False,**fields)
   self.assertEqual(s.out['indicator'],0)
 def test_lateral_disengagement_revokes_during_dark(self):
  s=Sim();s.arm();s.cc.latActive=False;s.tick(lit=False)
  self.assertEqual(s.out['indicator'],0)
 def test_manual_signal_has_no_new_dark_grace(self):
  s=Sim();s.arm();request=copy.deepcopy(s.out['request']);request['signal_source']='physical_confirmation'
  s.cs.leftBlinker=False;self.assertFalse(navigation_signal_matches(s.cs,request,s.now))
if __name__=='__main__':unittest.main(verbosity=2)
