"""Shared city-turn speed envelope; no actuator or steering commands."""
import math


def path_turn_target(xs, ys, lateral_accel, decel, cruise):
  """Back-propagate each curve's speed through its distance along the path.

  A distant tight corner is not treated as if it is under the vehicle now.
  Arc length also remains meaningful when a right-angle turn stops increasing x.
  """
  if (len(xs) != len(ys) or not 3 <= len(xs) <= 64
      or not all(math.isfinite(v) for v in (*xs, *ys, lateral_accel, decel, cruise))
      or lateral_accel <= 0 or decel <= 0 or cruise <= 0):
    return None
  points=list(zip(xs,ys));distance=[0.]
  for a,b in zip(points,points[1:]):distance.append(distance[-1]+math.dist(a,b))
  if distance[-1] < 8.:return None
  caps=[]
  for i,(a,b,c) in enumerate(zip(points,points[1:],points[2:])):
    if distance[i+2] > 40.:break
    ab,bc,ac=math.dist(a,b),math.dist(b,c),math.dist(a,c)
    if min(ab,bc,ac)<.5:continue
    k=abs(2*((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])))/(ab*bc*ac)
    if .008 <= k <= .25:
      caps.append(math.sqrt(lateral_accel/k + 2*decel*max(0.,distance[i]-2.)))
  return min(cruise,min(caps)) if caps else cruise


class TurnSpeedPlan:
  def __init__(self):
    self.target=None
    self.phase='OFF'
    self.owner='OFF'
    self.last_time=None

  def reset(self):
    self.target=None
    self.phase='OFF'
    self.owner='OFF'
    self.last_time=None

  def update(self, target, speed, actual_curvature, cruise, now, eligible, override, long_ready):
    if not eligible or target is None or not all(math.isfinite(v) for v in (target,speed,actual_curvature,cruise,now)):
      self.reset()
      return None
    dt=now-self.last_time if self.last_time is not None else .05
    previous=self.target
    self.last_time=now
    self.target=min(target,cruise)
    if previous is not None and 0<dt<=.25 and self.target>previous:
      self.target=min(self.target,max(previous,min(speed,self.target))+.6*dt)
    self.phase=('TURN' if actual_curvature>=.008 else
                'APPROACH' if target<cruise-.1 else 'EXIT')
    self.owner='DRIVER' if override else 'PLANNER' if long_ready else 'WAIT_LONG'
    return self.target if self.owner=='PLANNER' else None
