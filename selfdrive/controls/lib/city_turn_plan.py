"""One published turn contract, projected from the existing lateral lifecycle.

This module never authorizes steering, resumes from a stop, ignores a stop,
or writes actuators. Longitudinal control consumes the acquired path's geometry
and the SAME lifecycle rather than guessing another turn from curvature.
"""
import math
from openpilot.selfdrive.controls.lib.turn_context import navigation_request_valid, navigation_signal_matches


def finite(value):
  return type(value) in (int, float) and math.isfinite(value)


def path_constraints(xs, ys):
  """Bounded spatial curvature profile. No assumed angle, radius or turn speed.

  Select supports at least .75 m apart, so tiny early model-point spacing
  cannot amplify millimetre noise into a tight corner. Compress conservatively:
  each group retains its largest curvature at its earliest distance.
  """
  if (len(xs) != len(ys) or not 3 <= len(xs) <= 64
      or not all(finite(x) for x in (*xs, *ys))):
    return None
  support=[(0., float(xs[0]), float(ys[0]))]
  distance=0.
  for i in range(1, len(xs)):
    step=math.hypot(xs[i]-xs[i-1], ys[i]-ys[i-1])
    if step > 40.:
      return None
    distance += step
    if distance > 100.:
      break
    if distance-support[-1][0] >= .75:
      support.append((distance, float(xs[i]), float(ys[i])))
  if len(support) < 3 or distance < 8.:
    return None
  points=[]
  for a,b,c in zip(support,support[1:],support[2:]):
    ab=math.hypot(b[1]-a[1],b[2]-a[2]);bc=math.hypot(c[1]-b[1],c[2]-b[2]);ac=math.hypot(c[1]-a[1],c[2]-a[2])
    if min(ab,bc,ac) < .5:
      return None
    k=abs(2*((b[1]-a[1])*(c[2]-a[2])-(b[2]-a[2])*(c[1]-a[1])))/(ab*bc*ac)
    if k > .25:
      return None
    if k >= .003:
      points.append((max(0.,a[0]-2.), k))
  size=max(1,math.ceil(len(points)/12))
  return [[round(group[0][0],3), round(max(p[1] for p in group),6)]
          for i in range(0,len(points),size) if (group:=points[i:i+size])]


def profile_target(constraints, lateral_accel, decel, cruise):
  if (not isinstance(constraints,list) or len(constraints)>12
      or not all(finite(v) and v>0 for v in (lateral_accel,decel,cruise))):
    return None
  target=cruise;previous=-1.
  for point in constraints:
    if (not isinstance(point,(list,tuple)) or len(point)!=2
        or not all(finite(v) for v in point)):
      return None
    distance,k=point
    if not previous <= distance <= 100 or not .003 <= k <= .25:
      return None
    previous=distance
    target=min(target, math.sqrt(lateral_accel/k+2*decel*distance))
  return target


class CityTurnPlan:
  """Stable identity and phases; lateral intent remains the authority."""
  HANDOFF_SECONDS=3.

  def __init__(self):
    self.sequence=0
    self.clear()

  def clear(self):
    self.direction=0
    self.controller=None
    self.identity=None
    self.complete_since=None
    self.last_time=None
    self.last_geometry_time=None
    self.last_profile=None
    self.last_phase=None

  def update(self, *, intent, model, cs, now, enabled, inputs_valid, controller, direction, owner, request=None):
    release=(owner=='navigation' and navigation_request_valid(request,now,allow_release=True)
             and request.get('release_only') is True and intent.phase=='COMPLETE' and self.identity is not None)
    valid=(enabled is True and inputs_valid is True and finite(now)
           and (owner=='driver' or owner=='navigation' and navigation_request_valid(request,now,allow_release=True)
                and request['direction']==direction) and type(direction) is int and direction in (1,2)
           and isinstance(controller,str) and bool(controller)
           and (release or owner=='navigation' and navigation_signal_matches(cs,request,now)
                or owner=='driver' and bool(cs.leftBlinker)!=bool(cs.rightBlinker)
                and (direction==1)==bool(cs.leftBlinker))
           and cs.canValid and not cs.steeringPressed and not getattr(cs,'steeringDisengage',False)
           and not (cs.leftBlindspot if direction==1 else cs.rightBlindspot)
           and str(cs.gearShifter) in ('drive','low','sport','eco')
           and finite(cs.vEgo) and -.1<=cs.vEgo<=40*.44704
           and type(getattr(model,'frameId',None)) is int)
    stamp=now if finite(now) else 0.
    empty=dict(version=1,id=None,phase='OFF',direction=0,frame_id=getattr(model,'frameId',None),
               producer_mono=stamp,profile=None,reason='NO_AUTHORITY',lateral_phase=intent.phase)
    if not valid or (self.last_time is not None and not 0<now-self.last_time<=.15):
      self.clear()
      return empty
    if self.direction not in (0,direction) or self.controller not in (None,controller):
      self.clear()
      return dict(empty,reason='OWNER_OR_DIRECTION_CHANGED')
    self.last_time=now
    if intent.blocked and intent.phase!='COMPLETE':
      self.clear()
      return dict(empty,phase='ABORT',reason=intent.reason)
    if self.identity is None:
      # Do not invent an acquired turn merely because a lamp is active.
      if intent.phase in ('IDLE','WAIT','COMPLETE','CANCELLED'):
        return empty
      self.sequence+=1;self.identity=self.sequence;self.direction=direction;self.controller=controller
    profile=path_constraints(model.position.x,model.position.y)
    credible=(intent.last_gates.get('path_credible') is True
              and (intent.intent_path_acquired or intent.turn_locked))
    geometry_status='CURRENT'
    if credible and profile is not None:
      self.last_geometry_time=now
      self.last_profile=profile
    held=(not credible and intent.intent_path_acquired and intent.started is not None
          and intent.phase in ('TURN','EXIT') and self.last_profile is not None
          and self.last_geometry_time is not None and 0<=now-self.last_geometry_time<=.15
          and self.last_phase in ('APPROACH','TURN','EXIT'))
    if held:
      # Retain only a bounded speed ceiling across a lateral-authorized gap.
      # Never retain steering, launch permission, or a positive speed request.
      travel=max(0.,cs.vEgo)*(now-self.last_geometry_time)
      profile=[[round(max(0.,s-travel),3),k] for s,k in self.last_profile]
      geometry_status='HELD_CEILING'
    if intent.phase=='COMPLETE':
      if self.complete_since is None:
        self.complete_since=now
      phase='HANDOFF' if now-self.complete_since<=self.HANDOFF_SECONDS else 'COMPLETE'
    elif intent.queue_waiting or intent.stop_armed and cs.vEgo<=.3:
      phase='WAITING'
    elif intent.phase=='EXIT':
      phase='EXIT'
    elif credible and intent.physical_turn_started:
      phase='TURN'
    elif credible:
      phase='APPROACH'
    elif held:
      phase=self.last_phase
    else:
      phase='ACQUIRE'
    # Profile loss means no invented corner speed. Model/road/stop constraints
    # remain in the ordinary planner; there is no guessed zero-speed fallback.
    usable=(phase in ('APPROACH','TURN','EXIT','HANDOFF') or phase=='WAITING' and credible) and profile is not None
    self.last_phase=phase
    return dict(empty,id=self.identity,phase=phase,direction=direction,owner=owner,
                owner_key=request['key'] if owner=='navigation' else None,
                profile=profile if usable else None,reason=intent.reason,
                acquired=credible,conditioning=intent.started is not None or intent.stop_armed,
                geometry_status=geometry_status,
                geometry_age_s=(round(now-self.last_geometry_time,6) if held else 0. if usable else None),
                measured_heading_deg=round(math.degrees(intent.heading),3))


def paired_turn_plan(decision, model, now):
  """Strict consumer boundary; the model decision's own pairing is also needed."""
  p=decision.get('city_turn_plan') if isinstance(decision,dict) else None
  if (not isinstance(p,dict) or p.get('version')!=1 or type(p.get('id')) is not int
      or type(p.get('direction')) is not int or p['direction'] not in (1,2)
      or p.get('phase') not in ('ACQUIRE','WAITING','APPROACH','TURN','EXIT','HANDOFF','COMPLETE')
      or not finite(p.get('producer_mono')) or not finite(now)
      or not 0<=now-p['producer_mono']<=.15
      or type(p.get('frame_id')) is not int or type(getattr(model,'frameId',None)) is not int
      or abs(p['frame_id']-model.frameId)>2):
    return None
  return p
