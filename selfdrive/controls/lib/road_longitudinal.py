"""Spatial curve planning and short-horizon vision-lead continuity.

Pure helpers: no configuration, message clients, timers, or vehicle I/O.
Existing Map Driving Assist and Adaptive Accel switches remain the owners.
"""
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math
import numpy as np

from openpilot.selfdrive.controls.lib.curve_comfort import road_curve_speed


def _number(value):
  try:
    value = float(value)
    return value if math.isfinite(value) else None
  except (TypeError, ValueError, OverflowError):
    return None


def _interp(x, xs, ys):
  # Geometry is monotonic. Binary search preserves the exact old brackets
  # and arithmetic without repeatedly scanning up to 64 Python objects.
  i = bisect_left(xs,x,lo=1)
  if i < len(xs):
    fraction = (x-xs[i-1])/(xs[i]-xs[i-1])
    return ys[i-1] + fraction*(ys[i]-ys[i-1])
  return ys[-1]


def _path(position):
  xs, ys = tuple(position.x), tuple(position.y)
  if (not 12 <= len(xs) <= 64 or len(xs) != len(ys)
      or not all(math.isfinite(v) for v in xs+ys) or abs(xs[0]) > 2.
      or max(abs(v) for v in ys) > 250.):
    raise ValueError('Invalid path')
  stop = next((i+1 for i, (a,b) in enumerate(zip(xs,xs[1:])) if b <= a), len(xs))
  xs, ys = xs[:stop], ys[:stop]
  if len(xs) < 12 or xs[-1] < 60.:
    raise ValueError('Insufficient forward geometry')
  distances = [0.]
  for i in range(1,len(xs)):
    distances.append(distances[-1]+math.hypot(xs[i]-xs[i-1],ys[i]-ys[i-1]))
  if any(b <= a for a,b in zip(distances,distances[1:])):
    raise ValueError('Repeated path point')
  return distances, xs, ys


def _curvature(position, centers):
  distances, xs, ys = _path(position)
  n = next((i for i,c in enumerate(centers) if c+10.>distances[-1]),len(centers))
  if not n:return distances,[]
  # The 20 Hz planner already depends on NumPy. Fit all local triangles in
  # one bounded array operation, preserving interpolation brackets/order.
  ds=np.asarray(distances);queries=np.asarray(centers[:n])[:,None]+np.array([-10.,0.,10.])
  idx=np.clip(np.searchsorted(ds,queries,side='left'),1,len(ds)-1)
  fraction=(queries-ds[idx-1])/(ds[idx]-ds[idx-1])
  xx,yy=np.asarray(xs),np.asarray(ys)
  xx=xx[idx-1]+fraction*(xx[idx]-xx[idx-1])
  yy=yy[idx-1]+fraction*(yy[idx]-yy[idx-1])
  ax,bx,cx=xx.T;ay,by,cy=yy.T
  ab=np.hypot(bx-ax,by-ay);bc=np.hypot(cx-bx,cy-by);ac=np.hypot(cx-ax,cy-ay)
  denominator=ab*bc*ac
  if not np.all(np.isfinite(denominator)) or np.any(denominator<=0.):
    raise ValueError('Degenerate curvature support')
  values=2.*((bx-ax)*(cy-ay)-(by-ay)*(cx-ax))/denominator
  return distances,values.tolist()


@dataclass(frozen=True)
class CurveCeiling:
  current: float | None = None
  preview: float | None = None
  curvature: float = 0.
  reason: str = 'OFF'
  # Sustained model curvature along travelled arc distance, including straight.
  samples: tuple = ()
  corroborated_until: float = 0.


class RoadCurveGuard:
  """Retain spatial geometry, not a speed-cap release history.

  Reliable lane boundaries describe the road independently of the selected
  driving path. Maps preview the road; reliable lanes refine nearby curvature.
  """
  def __init__(self, dt=.05):
    self.dt = dt
    self.reset()

  def reset(self):
    self.confirm = 0.
    self.last_model_time = None
    self.result = CurveCeiling()

  def update(self, model, speed, comfort, *, enabled, valid, model_time, now,
             preview_distance=0.):
    if not enabled:
      self.reset()
      return self.result
    values = tuple(_number(v) for v in (speed,model_time,now,preview_distance))
    if (not valid or any(v is None for v in values) or not 5. <= speed <= 40.
        or not 0. <= now-model_time <= .25 or not 0. <= preview_distance <= 200.):
      self.reset()
      return CurveCeiling(reason='INVALID_OR_STALE')
    try:
      lane_changing = str(model.meta.laneChangeState) not in ('off','laneChangeDone')
      ds,_,_ = _path(model.position)
      centers = tuple(float(d) for d in range(15,int(min(140.,ds[-1]-10.))+1,5))
      if len(centers)<3:
        raise ValueError('No curvature support')
      if self.last_model_time is None:
        self.confirm = min(self.dt,.05)
      elif model_time > self.last_model_time:
        self.confirm += min(.10,model_time-self.last_model_time)
      elif model_time < self.last_model_time:
        self.reset()
        return CurveCeiling(reason='CLOCK_RESET')
      self.last_model_time = model_time
      if self.confirm < .25-1e-8:
        return CurveCeiling(reason='CONFIRMING')
      corroborated = 0.
      lanes = getattr(model,'laneLines',())
      probs = getattr(model,'laneLineProbs',())
      stds = getattr(model,'laneLineStds',())
      ystd = tuple(getattr(model.position,'yStd',()))
      road_samples = []
      if (len(lanes)>=3 and len(probs)>=3 and len(stds)>=3
          and all(math.isfinite(v) and v >= .8 for v in (probs[1],probs[2]))
          and all(math.isfinite(v) and 0. <= v <= .5 for v in (stds[1],stds[2]))
          and len(ystd)==len(ds) and all(math.isfinite(v) and v>=0. for v in ystd)):
        try:
          _,left = _curvature(lanes[1],centers)
          _,right = _curvature(lanes[2],centers)
          for i in range(1,min(len(centers),len(left),len(right))-1):
            d = centers[i]
            lk,rk = sorted(left[i-1:i+2])[1],sorted(right[i-1:i+2])[1]
            # Two boundaries must describe the same signed bend. A path's
            # lane-change/recentering motion must not become road curvature.
            if (_interp(d,ds,ystd)>1.5
                or abs(lk-rk)>max(.0005,.35*max(abs(lk),abs(rk)))):
              break
            road_samples.append((d,.5*(lk+rk)))
            corroborated = d
        except (AttributeError,TypeError,ValueError,ZeroDivisionError):
          pass
      if road_samples:
        samples = tuple(road_samples)
      elif lane_changing:
        # There is no confirmed road estimate. This is not proof of straight.
        self.result = CurveCeiling(reason='LANE_CHANGE_UNCONFIRMED')
        return self.result
      else:
        # Compute the path only when lanes cannot supply the road geometry.
        # Avoid a third full curvature fit in the ordinary lane-supported case.
        _,raw = _curvature(model.position,centers)
        samples = tuple((centers[i],sorted(raw[i-1:i+2])[1]) for i in range(1,len(raw)-1))
      # Outside lane changes, the model-path fallback can constrain speed,
      # but cannot confirm an unrelated mapped bend.
      targets = [road_curve_speed(abs(k),comfort) for _,k in samples]
      targets = [max(6.7,v) for v in targets if v is not None]
      # Compatibility diagnostics only; planning uses samples, not this min.
      minimum = min(targets) if targets else None
      self.result = CurveCeiling(minimum,minimum,max(abs(k) for _,k in samples),
                                 'MODEL_CURVE' if targets else 'NO_SUSTAINED_CURVE',
                                 samples,corroborated)
      return self.result
    except (AttributeError,TypeError,ValueError,OverflowError,ZeroDivisionError):
      self.reset()
      return CurveCeiling(reason='INVALID_PATH')


def curve_speed_profile(view, mapped_curves, positions, speed, cruise, comfort,
                        actuator_delay, diagnostic=None):
  """One spatial speed envelope for the existing MPC cruise obstacle.

  Every point has its own entry/exit; passed peaks cannot hold down recovery.
  Spatial map geometry retains advance authority beyond lane coverage;
  nearby camera road geometry refines it through a spatial transition.
  """
  if diagnostic is not None:
    diagnostic.update(map_rejected=0,map_retained=0,map_preview=0)
  if (not all(math.isfinite(v) for v in (speed,cruise,actuator_delay))
      or not 0. <= speed <= 40. or not 0. <= actuator_delay <= 2.
      or not positions or not all(math.isfinite(x) and x>=0. for x in positions)
      or any(b<a for a,b in zip(positions,positions[1:]))):
    return None
  restrictions = []
  for distance,k in view.samples:
    target = road_curve_speed(abs(k),comfort)
    if target is not None:
      restrictions.append((distance,max(6.7,target),distance+5.))
  camera_xs,camera_ks = tuple(zip(*view.samples)) if view.samples else ((),())
  for curve in mapped_curves:
    start = _number(curve.get('start'))
    end = _number(curve.get('end'))
    k = _number(curve.get('curvature'))
    target = _number(curve.get('target'))
    if (start is None or end is None or k is None or target is None
        or not -500. <= start < end <= 500. or end <= 0.
        or not .0018 <= abs(k) <= .12 or not 6.7 <= target <= 45.):
      continue
    if bool(curve.get('spatial',False)):
      # Map sees ahead. Within reliable lane coverage, use the camera road
      # samples already added above. Fade map curvature into the final 20 m
      # of that coverage, avoiding a hard switch at the sensor boundary.
      if view.corroborated_until >= 40. and view.samples:
        center = max(20.,start)
        if center <= view.corroborated_until:
          camera = abs(_interp(center,camera_xs,camera_ks))
          weight = min(1.,max(0.,(center-(view.corroborated_until-20.))/20.))
          k = camera*(1.-weight)+abs(k)*weight
          target = road_curve_speed(k,comfort)
          if target is None:
            if diagnostic is not None:diagnostic['map_rejected'] += 1
            continue
      if diagnostic is not None:diagnostic['map_retained'] += 1
      restrictions.append((max(0.,start),max(6.7,target),end))
      continue
    lo,hi = max(20.,start-10.),end+10.
    evidence = [abs(v) for d,v in view.samples if lo <= d <= hi]
    # Compatibility events lack the full shape; accept only corroborated
    # geometry. The new map producer supplies spatial preview independently.
    confirmed = view.corroborated_until >= hi and len(evidence)>=3
    if not confirmed:
      if diagnostic is not None:diagnostic['map_preview'] += 1
      continue
    road_k = sorted(evidence)[int(.65*(len(evidence)-1))]
    road_target = road_curve_speed(road_k,comfort)
    if road_target is None or abs(k) > 1.5*road_k+.0003:
      if diagnostic is not None:diagnostic['map_rejected'] += 1
      continue
    if diagnostic is not None:diagnostic['map_retained'] += 1
    restrictions.append((max(0.,start),max(target,road_target,6.7),end))
  if not restrictions:
    return None
  # MPC already anticipates its whole horizon. Reserve only actuator travel
  # and one geometry step here; there is no second three-second preview cap.
  reserve = max(5.,speed*actuator_delay)
  # Bounded (horizon x restrictions) arrays avoid an interpreted nested
  # loop for the full map corridor. Speeds are nonnegative, so taking the
  # minimum in squared-speed space preserves the same envelope.
  prepared=np.asarray(restrictions,dtype=float)
  location=np.asarray(positions,dtype=float)[:,None]
  approach=np.maximum(0.,prepared[None,:,0]-reserve-location)
  squared=prepared[None,:,1]**2+1.2*approach
  cruise_squared=cruise*cruise
  squared=np.where(location>prepared[None,:,2],cruise_squared,squared)
  profile=np.sqrt(np.minimum(cruise_squared,np.min(squared,axis=1)))
  return tuple(profile.tolist())


@dataclass(frozen=True)
class ContinuityLead:
  dRel: float
  vLead: float
  aLeadK: float
  aLeadTau: float
  duration: float
  status: bool = True


class LeadRecoveryGuard:
  """Constrain optimistic lead motion briefly inside MPC, never its accel cap.

  Raw closer-lead obstacles always remain authoritative. High uncertainty or
  a short gap alone cannot create a second following controller. Opening
  speed is retained, and each lead slot constrains only its own obstacle.
  """
  def __init__(self,dt=.05):
    self.dt=dt
    self.reset()

  def reset(self):
    self.history=[deque(),deque()]
    self.last_seen=[None,None]
    self.last_lead=[None,None]
    self.last_time=None
    self.reason='OFF'
    self.max_residual=0.
    self.constraints=None

  def update(self,leads,model_leads,speed,*,enabled,valid,now,time_gap,actuator_delay):
    if not enabled:
      self.reset()
      return None
    values=tuple(_number(v) for v in (speed,now,time_gap,actuator_delay))
    if (not valid or any(v is None for v in values) or not 5. <= speed <= 40.
        or not .5 <= time_gap <= 3. or not 0. <= actuator_delay <= 2.):
      self.reset()
      return None
    if self.last_time is not None and (now<self.last_time or now-self.last_time>.25):
      self.reset()
    if now==self.last_time:
      return self.constraints
    self.last_time=now
    constraints=[None,None];reasons=[];self.max_residual=0.
    for i,lead in enumerate(tuple(leads)[:2]):
      if bool(getattr(lead,'radar',False)):
        self.history[i].clear();self.last_seen[i]=self.last_lead[i]=None
        continue
      if not bool(getattr(lead,'status',False)):
        self.history[i].clear()
        old=self.last_lead[i]
        if old is not None and self.last_seen[i] is not None:
          age=now-self.last_seen[i]
          if 0.<=age<.75 and old.dRel<=max(15.,3.*speed):
            distance=max(1.,old.dRel+(old.vLead-speed)*age)
            constraints[i]=ContinuityLead(distance,old.vLead,old.aLeadK,old.aLeadTau,.75-age)
            reasons.append('RECENT_LEAD_LOSS')
        continue
      distance,relative,vlead,accel,tau=(_number(getattr(lead,n,None)) for n in ('dRel','vRel','vLead','aLeadK','aLeadTau'))
      if (any(v is None for v in (distance,relative,vlead,accel,tau))
          or not 0.<distance<250. or abs(relative)>70. or not 0.<=vlead<=80.
          or not 0.<tau<=10.):
        self.history[i].clear()
        continue  # Existing raw MPC validation remains in force.
      old=ContinuityLead(distance,vlead,accel,tau,.75)
      self.last_seen[i],self.last_lead[i]=now,old
      history=self.history[i]
      integral=history[-1][3]+.5*(relative+history[-1][2])*(now-history[-1][0]) if history else 0.
      history.append((now,distance,relative,integral))
      while len(history)>1 and now-history[1][0]>=.5:
        history.popleft()
      if now-history[0][0]<.4:
        continue
      prediction=history[0][1]+integral-history[0][3]
      residual=distance-prediction
      self.max_residual=max(self.max_residual,abs(residual))
      sigma=None
      if i<len(model_leads):
        std=getattr(model_leads[i],'xStd',())
        if len(std):sigma=_number(std[0])
      tolerance=max(3.,.75*(sigma if sigma is not None and sigma>=0. else 4.))
      if residual>tolerance:
        # Bounded optimistic excess only. No penalty for a newly closer
        # estimate, no permanent uncertainty veto, no speed/gap threshold.
        correction=min(residual-tolerance,15.,.25*distance)
        constraints[i]=ContinuityLead(max(1.,distance-correction),vlead,accel,tau,.75)
        reasons.append('RANGE_MOTION_DISAGREEMENT')
    self.constraints=tuple(constraints) if any(v is not None for v in constraints) else None
    self.reason='+'.join(dict.fromkeys(reasons)) if reasons else 'CLEAR'
    return self.constraints
