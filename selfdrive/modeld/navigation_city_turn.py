"""Navigation adapter to the same city-turn intent engine. No CAN or I/O."""
import hashlib
import math
import uuid
from openpilot.selfdrive.controls.lib.navigation_turn_commit import CityTurnIntent
from openpilot.selfdrive.controls.lib.turn_context import (
  navigation_source_valid, NAV_SIGNAL_DARK_MAX_S, NAV_SIGNAL_ACK_MAX_S,
)


class NavigationCityTurn:
  def __init__(self):
    self.controller=uuid.uuid4().hex
    self.intent=CityTurnIntent()
    self.key=None
    self.direction=0
    self.kind=None
    self.completed=set()
    self.request=None
    self.last_result=None
    self.handoff_started=None
    self.signal_lamp_mono=None

  def handoff_active(self, now):
    return (self.intent.phase=='COMPLETE' and self.request is not None
            and (self.handoff_started is None or 0<=now-self.handoff_started<=3.))

  def accepted(self):
    return (self.intent.started is not None or self.intent.stop_armed
            or self.intent.phase=='ARMED' and self.intent.queue_waiting
            and isinstance(self.request,dict) and self.request.get('signal_verified') is True)

  def withdraw(self, reason='NAVIGATION_WITHDRAWN'):
    if self.key and (self.accepted() or self.request is not None):self.completed.add(self.key)
    self.intent.cancel(reason)
    self.request=None
    self.last_result=None
    self.handoff_started=None
    self.signal_lamp_mono=None

  def step(self, state, distance, cs, cc, *, now, enabled, model, model_valid,
           physical_direction=0, signal_status=None, actual_curvature=None):
    if not navigation_source_valid(state,now) or type(distance) not in (int,float) or not math.isfinite(distance):
      self.withdraw('NAVIGATION_SOURCE_INVALID')
      return self._result('none',0,'NAVIGATION_SOURCE_INVALID')
    if type(physical_direction) is not int or physical_direction not in (0,1,2):
      self.withdraw('INVALID_PHYSICAL_SIGNAL')
      return self._result('none',0,'INVALID_PHYSICAL_SIGNAL')
    if self.intent.phase=='COMPLETE':
      return self._release(state,cs,cc,now,enabled,model_valid,physical_direction)
    incoming=(state['route_id'],state['maneuver_id'])
    requested={'left':1,'slight left':1,'right':2,'slight right':2}.get(state['maneuver']['modifier'],0)
    keep=False
    if self.key and incoming!=self.key and self.accepted():
      # A normal next-step display update can precede physical completion.
      # Only a sequential step on the SAME route, after measured entry and
      # acquisition, can renew the accepted turn's lease. Reroutes revoke it.
      keep=(incoming[0]==self.key[0] and self.key[1].isdigit() and incoming[1].isdigit()
            and int(incoming[1])==int(self.key[1])+1
            and self.intent.intent_path_acquired and self.intent.physical_turn_started)
      if not keep:
        self.withdraw('ROUTE_OR_MANEUVER_CHANGED')
        return self._result('none',0,'ROUTE_OR_MANEUVER_CHANGED')
    key=self.key if keep else incoming
    direction=self.direction if keep else requested
    if not keep and state['maneuver']['type'] not in ('turn','end of road'):
      self.withdraw('NOT_A_CITY_TURN')
      return self._result('none',0,'NOT_A_CITY_TURN')
    if key!=self.key:
      self.intent.reset();self.key=key;self.direction=direction;self.kind=state['maneuver']['type'];self.handoff_started=None
      self.signal_lamp_mono=None
    if len(self.completed)>128:self.completed={self.key} if self.key in self.completed else set()
    conflicting_lamp=(bool(cs.leftBlinker) and bool(cs.rightBlinker)
                      or direction==1 and bool(cs.rightBlinker)
                      or direction==2 and bool(cs.leftBlinker))
    if (physical_direction not in (0,direction) or cs.steeringPressed
        or getattr(cs,'steeringDisengage',False) or conflicting_lamp):
      self.completed.add(key)
    if (key in self.completed or not enabled or direction not in (1,2)
        or physical_direction not in (0,direction)
        or not cs.canValid or not math.isfinite(cs.vEgo)
        or str(cs.gearShifter) not in ('drive','low','sport','eco')
        or cs.steeringPressed or getattr(cs,'steeringDisengage',False)
        or (cs.leftBlindspot if direction==1 else cs.rightBlindspot)):
      self.withdraw('NAVIGATION_OVERRIDE_OR_DISABLED')
      return self._result('none',0,'NAVIGATION_OVERRIDE_OR_DISABLED')
    accepted=self.accepted()
    if not keep and not (0<distance<=80. or accepted and distance<=80.):
      self.withdraw('OUTSIDE_APPROACH_OR_UNACCEPTED_ZERO')
      return self._result('none',0,'OUTSIDE_APPROACH_OR_UNACCEPTED_ZERO')
    token=hashlib.sha256(('|'.join((self.controller,*key))).encode()).hexdigest()[:32]
    lamp=bool(cs.leftBlinker)!=bool(cs.rightBlinker) and (direction==1)==bool(cs.leftBlinker)
    status=signal_status if isinstance(signal_status,dict) else {}
    stamp=status.get('time')
    ack=(type(stamp) in (int,float) and math.isfinite(stamp) and 0<=now-stamp<=NAV_SIGNAL_ACK_MAX_S
         and status.get('request_key')==token and status.get('route_id')==key[0]
         and status.get('maneuver_id')==key[1] and status.get('controller')==self.controller
         and type(status.get('direction')) is int and status.get('direction')==direction and status.get('active') is True
         and status.get('sent_recent') is True and status.get('physical_override') is False)
    physical=(type(physical_direction) is int and physical_direction==direction)
    # A factory flasher's dark interval is not cancellation. Require an
    # actual matching lamp first, then keep only this identity's confirmation
    # across a bounded gap while matching transmission feedback stays fresh.
    synthetic = bool(ack and not physical and physical_direction == 0)
    if not synthetic or conflicting_lamp:
      self.signal_lamp_mono=None
    elif lamp:
      self.signal_lamp_mono=now
    recent_lamp=(self.signal_lamp_mono is not None
                 and 0<=now-self.signal_lamp_mono<=NAV_SIGNAL_DARK_MAX_S)
    verified=bool(lamp and physical or synthetic and recent_lamp and not conflicting_lamp)
    context=state.get('turn_context')
    angle=context.get('expected_angle_deg') if isinstance(context,dict) else None
    angle=angle if type(angle) in (int,float) and math.isfinite(angle) and -180<=angle<=180 else None
    self.request=dict(version=1,owner='navigation',controller=self.controller,route_id=key[0],
      maneuver_id=key[1],key=token,direction=direction,kind=self.kind,lease_mono=now,
      source_mono=state['received_mono'],expires_mono=min(state['expires_mono'],state['received_mono']+1.5-state['position_quality']['gps_age_s']),
      city_enabled=bool(enabled),route_authorized=True,signal_verified=verified,
      stationary_wait_allowed=bool(0<=cs.vEgo<=.3 and getattr(cc,'enabled',False)),
      expected_angle_deg=angle,
      signal_source='physical_confirmation' if physical else 'synthetic_feedback',
      signal_last_lamp_mono=self.signal_lamp_mono, signal_ack_mono=stamp if synthetic else None,
      queued_maneuver_id=incoming[1] if keep else None)
    stationary=0<=cs.vEgo<=.3 and not self.intent.physical_turn_started
    authorized=bool(verified and model_valid and cc.latActive)
    waiting=bool(verified and model_valid and stationary and not cc.latActive)
    guidance_window=min(40.,max(12.,max(0.,cs.vEgo)*3.))
    # Announcement can lead acquisition; geometry gathering above 25 mph is
    # allowed, but new conditioning still requires the existing guidance window.
    enter_window=accepted or keep or distance<=guidance_window
    on=False
    if enter_window:
      on=self.intent.update(model,direction,max(0.,cs.vEgo),authorized,now,
        transient_unavailable=not model_valid,actual_curvature=actual_curvature,stationary_wait=waiting)
    else:
      self.intent.reset()
    if self.intent.phase=='COMPLETE':
      return self._release(state,cs,cc,now,enabled,model_valid,physical_direction)
    if self.intent.blocked:
      self.completed.add(key);self.request=None
      return self._result('none',0,self.intent.reason)
    indicator=0 if physical else direction
    desire=('turnLeft' if direction==1 else 'turnRight') if on and cc.latActive else 'none'
    return self._result(desire,indicator,self.intent.reason if enter_window else 'ANNOUNCE_BEFORE_GUIDANCE')

  def _release(self,state,cs,cc,now,enabled,model_valid,physical_direction):
    """Fresh, bounded speed-ceiling release; never signal or steering authority."""
    if self.handoff_started is None:
      self.handoff_started=now
    self.completed.add(self.key)
    incoming=(state['route_id'],state['maneuver_id'])
    same_step=incoming==self.key
    next_step=(incoming[0]==self.key[0] and self.key[1].isdigit() and incoming[1].isdigit()
               and int(incoming[1])==int(self.key[1])+1)
    if (not self.handoff_active(now) or not (same_step or next_step)
        or not enabled or not model_valid or not cc.latActive or physical_direction!=0
        or not self.request.get('signal_verified') or not cs.canValid
        or cs.steeringPressed or getattr(cs,'steeringDisengage',False)
        or cs.gasPressed or cs.brakePressed or getattr(cs,'brake',0.)>0.
        or not math.isfinite(cs.vEgo) or not .3<=cs.vEgo<=40*.44704
        or str(cs.gearShifter) not in ('drive','low','sport','eco')
        or (cs.leftBlindspot if self.direction==1 else cs.rightBlindspot)):
      self.withdraw('HANDOFF_FINISHED_OR_REVOKED')
      return self._result('none',0,'HANDOFF_FINISHED_OR_REVOKED')
    self.request=dict(self.request,lease_mono=now,source_mono=state['received_mono'],
      expires_mono=min(state['expires_mono'],state['received_mono']+1.5-state['position_quality']['gps_age_s']),
      release_only=True,release_started_mono=self.handoff_started,release_until_mono=self.handoff_started+3.,
      stationary_wait_allowed=False,queued_maneuver_id=incoming[1] if next_step else None)
    return self._result('none',0,'SHARED_LONGITUDINAL_HANDOFF')

  def _result(self,desire,indicator,reason):
    e=self.intent
    self.last_result=dict(request=self.request,lifecycle=dict(phase=e.phase,reason=e.reason,
      blocked=e.blocked,continuing=e.started is not None,direction=self.direction,
      acquired=e.intent_path_acquired,stage='TURNING' if e.turn_started is not None else e.phase),
      desire=desire,indicator=indicator,reason=reason)
    return self.last_result
