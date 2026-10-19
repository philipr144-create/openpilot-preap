"""Bounded profile settings and actual curve-controller regressions."""
import json
import math
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as N

from openpilot.selfdrive.controls.lib.curve_comfort import PROFILES, read_curve_comfort, write_curve_comfort
from openpilot.selfdrive.controls.lib.corner_assist import CornerAssist
from openpilot.selfdrive.nap.nap_mapd import curve_events, build_control_curves
from openpilot.selfdrive.controls.lib.longitudinal_planner import map_curve_speed_cap


class CurveComfortTests(unittest.TestCase):
  def test_roundtrip_presets_and_invalid_fallback(self):
    with tempfile.TemporaryDirectory() as tmp:
      path=Path(tmp)/'comfort'
      self.assertEqual(read_curve_comfort(path).name,'standard')
      for name in PROFILES:
        write_curve_comfort(name,path)
        self.assertEqual(read_curve_comfort(path),PROFILES[name])
      for raw in ('{','[]','null','{"profile": 2}','{"profile": "fast"}', ' '*1025):
        path.write_text(raw)
        self.assertEqual(read_curve_comfort(path).name,'standard')
      with self.assertRaises(ValueError):write_curve_comfort('fast',path)
      self.assertEqual(sorted(p.name for p in Path(tmp).iterdir()),['comfort'])

  def test_map_range_starts_at_former_mild_and_city_tuning_is_preserved(self):
    self.assertEqual(PROFILES['more'].map_lateral_accel,3.4)
    for name,corner,decel in [('more',1.0,.8),('standard',1.2,.8),('mild',1.6,.6)]:
      self.assertEqual(PROFILES[name].corner_lateral_accel,corner)
      self.assertEqual(PROFILES[name].corner_max_decel,decel)
    self.assertLess(PROFILES['more'].map_lateral_accel,PROFILES['standard'].map_lateral_accel)
    self.assertLess(PROFILES['standard'].map_lateral_accel,PROFILES['mild'].map_lateral_accel)

  def run_corner(self, profile, speed=12., curvature=.02, base=0., cruise=25.):
    c=CornerAssist();c.config['corner_assist']=True;c.comfort=PROFILES[profile]
    for i in range(100):out=c.update(base,speed,curvature,-.3,True,100+i*.05,cruise)
    return c,out

  def test_mild_has_higher_absolute_target_and_less_extra_deceleration(self):
    standard,a_standard=self.run_corner('standard')
    mild,a_mild=self.run_corner('mild')
    self.assertAlmostEqual(mild.target_speed/standard.target_speed,math.sqrt(1.6/1.2))
    self.assertGreater(a_mild,a_standard)
    self.assertGreaterEqual(a_mild,-.6)
    self.assertAlmostEqual(a_standard,-.8)

  def test_all_profiles_release_at_target_and_preserve_stronger_braking(self):
    for name,p in PROFILES.items():
      target=math.sqrt(p.corner_lateral_accel/.02)
      _,out=self.run_corner(name,speed=target)
      self.assertEqual(out,0.)
      _,out=self.run_corner(name,base=-2.)
      self.assertEqual(out,-2.)
      c,out=self.run_corner(name,cruise=5.)
      self.assertEqual(c.target_speed,5.)
      self.assertLess(out,0.)

  def test_switch_to_mild_releases_old_stronger_curve_cap(self):
    c,_=self.run_corner('standard')
    c.comfort=PROFILES['mild']
    self.assertGreaterEqual(c.update(0.,12.,.02,-.3,True,105.,25.),-.6)

  def test_refresh_reads_profile_without_enabling_feature(self):
    c=CornerAssist();c.params=N(get=lambda _:False)
    with patch('openpilot.selfdrive.controls.lib.corner_assist.read_curve_comfort',return_value=PROFILES['mild']):
      c.refresh(100.)
    self.assertEqual(c.comfort.name,'mild')
    self.assertFalse(c.config['corner_assist'])
    self.assertEqual(c.update(.5,10.,.02,-.3,True,100.,25.),.5)

  def test_known_circle_map_targets_and_consumer(self):
    radius=100.
    samples=[dict(x=radius*math.sin(d/radius),y=radius*(1-math.cos(d/radius)),distance=d) for d in range(0,151,5)]
    with tempfile.TemporaryDirectory() as tmp, patch('openpilot.selfdrive.controls.lib.longitudinal_planner.mapped_stop_setting_enabled',return_value=False):
      path=Path(tmp)/'map';last=0.
      for name in ('more','standard','mild'):
        events=curve_events(samples,comfort=PROFILES[name])
        self.assertEqual(len(events),1)
        # Independent roots of v^2/100 = the piecewise lateral budget(v).
        expected = {'more': 12.6519384688, 'standard': 13.4969160446, 'mild': 14.9915098228}
        self.assertAlmostEqual(events[0]['target_speed_mps'],expected[name],places=2)
        self.assertEqual(events[0]['comfort_profile'],name)
        controls=build_control_curves(events)
        state=dict(version=3,enabled=True,valid=True,control_valid=True,status='ok',mono_time=100.,expires_mono=102.,
                   match=dict(distance_m=0.,heading_error_deg=0.,confidence=1.),lookahead_m=450.,control_curves=controls)
        path.write_text(json.dumps(state))
        cap=map_curve_speed_cap(path,now=101.,v_ego=20.)
        self.assertGreater(cap,last);last=cap
        # A fresh absolute curve target is speed-independent. With an aged
        # snapshot, different speed means different travel toward that curve.
        fresh_cap=map_curve_speed_cap(path,now=100.,v_ego=20.)
        self.assertEqual(map_curve_speed_cap(path,now=100.,v_ego=10.),fresh_cap)
        self.assertIsNone(map_curve_speed_cap(path,now=103.,v_ego=20.))
        state['enabled']=False;path.write_text(json.dumps(state))
        self.assertIsNone(map_curve_speed_cap(path,now=101.,v_ego=20.))


if __name__=='__main__':unittest.main()
