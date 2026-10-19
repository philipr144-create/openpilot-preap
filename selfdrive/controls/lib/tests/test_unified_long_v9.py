import json,math,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as N
from openpilot.selfdrive.controls.lib.model_forecast import assess_model_forecast,assess_model_intent
from openpilot.selfdrive.controls.lib.cruise_refinement import model_support_weight
from openpilot.selfdrive.controls.lib.acceleration_handoff import braking_priority
from openpilot.selfdrive.controls.lib.longitudinal_planner import LongitudinalPlanner,map_curve_speed_cap
from openpilot.selfdrive.nap import nap_mapd
from opendbc.car.tesla.preap.pitch_grade_assist import PitchGradeAssist,MAX_SUPPORT,MAX_SLEW_UP

def model(request=-.2,drop=.6,stop=False):
 return N(action=N(desiredAcceleration=request,shouldStop=stop),velocity=N(t=[0.,1.,2.,3.,6.,10.],x=[25.,25.,25.-drop/2,25.-drop,25.-drop,25.-drop]))

class SharedIntent(unittest.TestCase):
 def test_all_layers_use_same_braking_classification(self):
  for request,drop,stop in [(-.21,.6,False),(-.14,.84,False),(-.1,1.6,False),(-.8,.1,False),(-.1,.1,True)]:
   m=model(request,drop,stop);f=assess_model_forecast(m,25.);i=assess_model_intent(m,25.,request,f)
   self.assertEqual(model_support_weight(m,25.,request,f),i.support_weight)
   priority=braking_priority(m,25.,request,.4,False,False,False,28.,28.,forecast=f)
   self.assertEqual(priority,i.reason if i.braking_required else 'ORDINARY')
 def test_old_thresholds_no_longer_switch_coast_into_braking(self):
  for a,b in [(-.199,-.201)]:
   x=assess_model_intent(model(a),25.,a);y=assess_model_intent(model(b),25.,b)
   self.assertFalse(x.braking_required or y.braking_required);self.assertLess(abs(x.support_weight-y.support_weight),.01)
  x=assess_model_intent(model(drop=.49),25.,-.2);y=assess_model_intent(model(drop=.51),25.,-.2)
  self.assertFalse(x.braking_required or y.braking_required)
  self.assertEqual(x.support_weight, 0.)
  self.assertEqual(y.support_weight, 0.)
  z=assess_model_intent(model(request=-.1,drop=.2),25.,-.1)
  self.assertGreater(z.support_weight, 0.)
 def test_invalid_partial_stop_and_near_braking_never_add_support(self):
  cases=[model(stop=True),model(drop=1.6),model(request=-.8)]
  m=model();m.velocity.t[-1]=8.;cases.append(m)
  m=model();m.velocity.x[1]=float('nan');cases.append(m)
  for m in cases:
   i=assess_model_intent(m,25.,m.action.desiredAcceleration)
   self.assertTrue(i.braking_required);self.assertEqual(i.support_weight,0.)
 def test_bounded_recovery_never_overrides_lead_or_stop(self):
  p=LongitudinalPlanner.__new__(LongitudinalPlanner);p.nap_adaptive_accel=True;p.output_a_target=0.0
  m=model(request=-.1,drop=.2);i=assess_model_intent(m,25.,-.1)
  def run(lead=False,stop=False):
   return p._experimental_longitudinal_arbiter(-.1,.4,28.,28.,25.,lead,stop,False,False,False,True,False,False,
     model_braking_required=i.braking_required,recovery_weight=i.support_weight)
  a=run();self.assertGreater(a,-.1);self.assertLessEqual(a,min(.4,-.1+.25))
  self.assertEqual(run(lead=True),-.1);self.assertEqual(run(stop=True),-.1)

class MapHorizon(unittest.TestCase):
 def test_long_edge_clips_extent_and_does_not_move_stop_sign(self):
  node=N(id=123,tags={'highway':'stop'})
  points=[(0.,math.degrees(d/nap_mapd.EARTH_RADIUS_M),node if d==700 else None,1) for d in [0.,100.,400.,700.]]
  geom=nap_mapd.raw_geometry(0.,0.,points)
  self.assertEqual(geom[-1]['distance'],450.);self.assertAlmostEqual(geom[-1]['x'],450.)
  self.assertIsNone(geom[-1]['node']);self.assertEqual(nap_mapd.node_events(geom),[])
  self.assertEqual(nap_mapd.resample_geometry(geom)[-1]['distance'],450.)
 def test_short_extent_is_not_fabricated(self):
  points=[(0.,math.degrees(d/nap_mapd.EARTH_RADIUS_M),None,1) for d in [0.,50.]]
  self.assertAlmostEqual(nap_mapd.raw_geometry(0.,0.,points)[-1]['distance'],50.)
 def test_snapshot_with_clipped_horizon_keeps_curve_available(self):
  state=dict(version=3,enabled=True,valid=True,control_valid=True,status='ok',mono_time=100.,expires_mono=102.,
    match=dict(distance_m=1.,heading_error_deg=1.,confidence=.95),lookahead_m=450.,control_curves=[dict(start_distance_m=40.,end_distance_m=100.,length_m=60.,target_speed_mps=15.,curvature=.01)])
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'map.json';p.write_text(json.dumps(state));reason={}
   self.assertIsNotNone(map_curve_speed_cap(str(p),now=100.,v_ego=25.,include_stops=False,diagnostic=reason))
   self.assertEqual(reason['reason'],'AVAILABLE')
   state['lookahead_m']=700.;p.write_text(json.dumps(state))
   self.assertIsNone(map_curve_speed_cap(str(p),now=100.,v_ego=25.,include_stops=False,diagnostic=reason))
   self.assertEqual(reason['reason'],'INVALID_LOOKAHEAD')

class PitchReference(unittest.TestCase):
 def helper(self,mode='on',offset=.03):
  p=PitchGradeAssist(start_watcher=False);p._settings=(mode,True,offset);p.reset();return p
 def test_reference_offset_does_not_create_false_hill(self):
  p=self.helper()
  for _ in range(600):self.assertEqual(p.update([0.,.03,0.],25.,.05,allowed=True),0.)
 def test_uphill_support_has_filter_settle_ramp_and_cap(self):
  p=self.helper();previous=0.
  for tick in range(600):
   value=p.update([0.,.10,0.],25.,.05,allowed=True)
   if tick<190:self.assertEqual(value,0.)
   self.assertGreaterEqual(value,0.);self.assertLessEqual(value,MAX_SUPPORT)
   self.assertLessEqual(value-previous,MAX_SLEW_UP*p.dt+1e-9);previous=value
  self.assertAlmostEqual(previous,.1)
 def test_on_requires_new_engagement_and_off_is_immediate(self):
  p=self.helper('monitor');p._settings=('on',True,.03)
  for _ in range(600):self.assertEqual(p.update([0.,.1,0.],25.,.05,allowed=True),0.)
  p.reset()
  for _ in range(600):p.update([0.,.1,0.],25.,.05,allowed=True)
  self.assertGreater(p.applied_mps2,0.)
  p._settings=('off',False,0.);self.assertEqual(p.update([0.,.1,0.],25.,.05,allowed=True),0.)
 def test_reference_change_and_braking_remove_support(self):
  p=self.helper()
  for _ in range(600):p.update([0.,.1,0.],25.,.05,allowed=True)
  self.assertEqual(p.update([0.,.1,0.],25.,-.1,allowed=True),0.)
  p._settings=('on',True,.04);self.assertEqual(p.update([0.,.1,0.],25.,.05,allowed=True),0.);self.assertFalse(p.armed)
 def test_verification_requires_bounded_finite_reference(self):
  with tempfile.TemporaryDirectory() as d:
   path=Path(d)/'settings.json';p=PitchGradeAssist(params=N(get=lambda *a,**k:2),verification_file=path,start_watcher=False)
   for offset in [None,float('nan'),.2,True]:
    path.write_text(json.dumps(dict(pitch_assist_verified=True,pitch_reference_offset_rad=offset)));p.refresh_settings();p.reset();self.assertFalse(p.armed)
   path.write_text(json.dumps(dict(pitch_assist_verified=True,pitch_reference_offset_rad=.03)));p.refresh_settings();p.reset();self.assertTrue(p.armed)
