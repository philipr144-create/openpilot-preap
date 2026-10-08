import math
from types import SimpleNamespace as N
from unittest.mock import patch
from pathlib import Path
import tempfile
from cereal import log
from openpilot.selfdrive.controls.lib.navigation_turn_commit import ManualTurnEntry
from openpilot.selfdrive.controls.lib.turn_context import manual_turn_intent
from openpilot.selfdrive.controls.tests import test_manual_turn_commit as manual_cases
from openpilot.selfdrive.controls.tests import test_city_turn_gates as city_cases
from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire
from opendbc.car.tesla.preap.steering_override import hands_on_override


def model(frame=0):
 return N(frameId=frame,position=N(t=[0.,1.,2.,3.],x=[0.,4.,8.,10.],y=[0.,-.5,-2.,-4.]),orientation=N(z=[0.,-.1,-.3,-.6]),action=N(desiredCurvature=-.03))


def test_entry_starts_below_25_and_continues_across_speed_boundary():
 e=ManualTurnEntry();m=model()
 for i in range(8):m.frameId+=1;e.update(m,1,24*.44704,True,100+i*.05)
 assert e.started is not None
 started=e.started
 for i,speed in enumerate([24.9,25.1,26.,30.,34.9]):
  m.frameId+=1
  assert e.update(m,1,speed*.44704,True,100.4+i*.05)
  assert e.started==started and not e.blocked
 m.frameId+=1
 assert not e.update(m,1,36*.44704,True,100.7)
 assert e.blocked
 assert not e.update(m,1,24*.44704,True,100.75)
 fresh=ManualTurnEntry()
 assert not fresh.update(model(1),1,26*.44704,True,100.)
 assert fresh.started is None


def test_manual_actuator_commit_does_not_cancel_at_26():
 t=manual_cases.ManualTests();t.setUp();t.start();started=t.helper.started
 t.cs.vEgo=26*.44704
 t.tick()
 assert t.helper.active and not t.helper.blocked
 assert t.helper.started==started
 t.cs.steeringPressed=True;t.tick()
 assert not t.helper.active and t.helper.blocked


def test_desire_helper_preserves_conditioning_at_26_then_exits_normally():
 t=city_cases.CityTurnGates();t.setUp();t.prepare();t.cs.vEgo=24*.44704
 for _ in range(8):t.tick()
 assert t.helper.desire==log.Desire.turnLeft
 started=t.helper.manual_entry.started
 for speed in [24.9,25.1,26.,24.8,26.1]:
  t.cs.vEgo=speed*.44704
  assert t.tick()==log.Desire.turnLeft
  assert t.helper.manual_entry.started==started
 t.cs.steeringPressed=True;assert t.tick()==log.Desire.none
 t.cs.steeringPressed=False;t.cs.vEgo=24*.44704
 for _ in range(8):assert t.tick()==log.Desire.none
 t.cs.leftBlinker=False;t.tick(direction=0)
 t.cs.leftBlinker=True
 for _ in range(8):t.tick()
 assert t.helper.desire==log.Desire.turnLeft


def test_shared_intent_requires_verified_continuation_above_25():
 c=N(vEgo=26*.44704,leftBlinker=True,rightBlinker=False,steeringPressed=False,steeringDisengage=False,brakePressed=False,canValid=True,leftBlindspot=False,rightBlindspot=False,gearShifter='drive')
 d=dict(received_mono=100.,frame_id=1,model_evaluated=True,manual_turn_direction=1,city_turns_enabled=True,signal_owner='driver',tap_signal_active=False,synthetic_indicator_request='none',source='driver_desire_helper',final_desire='turnLeft',manual_turn_blocked=False)
 assert not manual_turn_intent(c,d,model(1),100.)
 d['manual_turn_continuing']=True;assert manual_turn_intent(c,d,model(1),100.)
 c.leftBlinker=False;assert not manual_turn_intent(c,d,model(1),100.)
 c.leftBlinker=True;c.steeringPressed=True;assert not manual_turn_intent(c,d,model(1),100.)
 c.steeringPressed=False;assert not manual_turn_intent(c,d,model(1),100.5)


def test_navigation_continuation_is_bound_to_actual_started_maneuver():
 state=dict(enabled=True,route_active=True,received_mono=100.,expires_mono=103.,route_state='active',route_id='test',maneuver_id='turn1',position_quality=dict(match_error_m=2.,gps_accuracy_m=3.,gps_age_s=.1),maneuver=dict(type='turn',modifier='left',distance_m=10.))
 with tempfile.TemporaryDirectory() as td:
  n=NavigationDesire(str(Path(td)/'nav'),str(Path(td)/'diag'),str(Path(td)/'signal'));n.params=N(get_bool=lambda _:True)
  c=N(vEgo=26*.44704,leftBlinker=True,rightBlinker=False,steeringPressed=False,steeringDisengage=False,gearShifter='drive',brakePressed=False)
  def tick():n.prepared_navigation=(state,'');return n.update(c,N(latActive=True),True,now=101.,physical_direction=1)
  assert tick()=='none' # No new turn above the original entry limit.
  c.vEgo=24*.44704;assert tick()=='turnLeft'
  c.vEgo=26*.44704;assert tick()=='turnLeft'
  state['maneuver_id']='turn2';assert tick()=='none' # New route step cannot inherit permission.
  c.vEgo=24*.44704;assert tick()=='turnLeft'
  c.vEgo=26*.44704;c.steeringPressed=True;assert tick()=='none'
  c.steeringPressed=False;assert tick()=='none'


def test_low_epas_contact_is_not_override_strong_effort_still_wins():
 for torque in [.0,.5,.78,-.97]:assert not hands_on_override(1,torque)
 assert hands_on_override(1,1.1)
 assert hands_on_override(1,-1.1)
 assert hands_on_override(2,.0)
 assert hands_on_override(3,.0)
 assert hands_on_override(0,float('nan'))
