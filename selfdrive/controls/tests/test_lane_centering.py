import math
import unittest
from types import SimpleNamespace as N
from openpilot.selfdrive.controls.lib import lane_centering as m

def curve(y):return N(x=[0,5,10,20,30,40,60],y=[y]*7)
def model(error=.2):return N(position=curve(0),laneLines=[curve(-4),curve(-1.5+error),curve(1.5+error)],laneLineProbs=[.1,.99,.99],meta=N(laneChangeState='off',desireState=[1,0,0,0,0,0,0,0]))
def step(a,md=None,**kw):
 args=dict(enabled=True,active=True,healthy=True,overriding=False,strength=1,dt=.01);args.update(kw)
 return a.update(md or model(),15,0,**args)
class Tests(unittest.TestCase):
 def test_direction_and_bounds(self):
  for err in (-.4,.4):
   a=m.LaneCenteringAssist();out=[step(a,model(err),strength=3) for _ in range(500)]
   self.assertGreater(out[-1]*err,0)
   self.assertLessEqual(max(abs(x)*225 for x in out),a.MAX_ACCEL+1e-9)
   self.assertLessEqual(max(abs(b-c)*225/.01 for b,c in zip(out,out[1:])),a.MAX_JERK+1e-9)
 def test_disable_and_override(self):
  for k,v in [('enabled',False),('active',False),('healthy',False),('overriding',True)]:
   a=m.LaneCenteringAssist()
   for _ in range(100):step(a)
   self.assertEqual(step(a,**{k:v}),0)
   self.assertEqual(a.good_time,0)
 def test_centered(self):
  a=m.LaneCenteringAssist()
  for _ in range(200):self.assertEqual(step(a,model(0)),0)
 def test_bad_geometry(self):
  cases=[]
  for mutation in ('nan','order','short','wide','prob','desire','change','error'):
   md=model()
   if mutation=='nan':md.position.y[2]=math.nan
   if mutation=='order':md.laneLines[1].x[2]=3
   if mutation=='short':md.position.x=[0,5];md.position.y=[0,0]
   if mutation=='wide':md.laneLines[2]=curve(4)
   if mutation=='prob':md.laneLineProbs[2]=.5
   if mutation=='desire':md.meta.desireState[0]=.8
   if mutation=='change':md.meta.laneChangeState='laneChangeStarting'
   if mutation=='error':md=model(.8)
   a=m.LaneCenteringAssist()
   for _ in range(100):self.assertEqual(step(a,md),0,mutation)
 def test_strength(self):
  vals=[]
  for strength in (1,2,3):
   a=m.LaneCenteringAssist()
   for _ in range(300):v=step(a,strength=strength)
   vals.append(v)
  self.assertTrue(0<vals[0]<vals[1]<vals[2])
 def test_speed_and_curve_gates(self):
  for speed,c in [(0,0),(8,0),(35,0),(15,.005),(30,.003),(math.nan,0)]:
   a=m.LaneCenteringAssist()
   self.assertEqual(a.update(model(),speed,c,enabled=True,active=True,healthy=True,overriding=False),0)
 def test_confidence_loss_fades(self):
  a=m.LaneCenteringAssist()
  for _ in range(300):step(a)
  md=model();md.laneLineProbs[2]=.1
  values=[step(a,md) for _ in range(300)]
  self.assertEqual(values[-1],0)
  self.assertTrue(all(b<=a for a,b in zip(values,values[1:])))
if __name__=='__main__':unittest.main()
