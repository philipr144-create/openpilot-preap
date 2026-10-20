import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace as N

from openpilot.selfdrive.ui.engagement_stats import EngagementStats, hud_text, summarize


class StatsTests(unittest.TestCase):
  def setUp(self):
    self.tmp = tempfile.TemporaryDirectory()
    self.addCleanup(self.tmp.cleanup)
    self.path = Path(self.tmp.name)/'time.json'
    self.stats = EngagementStats(self.path, Path(self.tmp.name)/'miles.json')

  def tick(self, t, **updates):
    args=dict(now=t, started=True, fresh=True, in_drive=True,
              lat_active=False, long_active=False, trip_id='route-a')
    args.update(updates)
    self.stats.sample(**args)

  def test_exclusive_modes_percent_and_stopped_in_drive(self):
    self.tick(0.)
    self.tick(1.,lat_active=True)
    self.tick(2.,long_active=True)
    self.tick(3.,lat_active=True,long_active=True)
    self.tick(4.,lat_active=True,long_active=True)
    data=self.stats.snapshot()
    self.assertEqual(data['trip'],dict(manual=1.,steering=1.,speed=1.,both=1.))
    self.assertEqual(data['total'],data['trip'])
    self.assertEqual(summarize(data['trip'])['percent'],75.)
    self.assertIn('TRIP 75%',hud_text(data))

  def test_park_stale_gaps_and_backward_clock_never_add_time(self):
    self.tick(0.,lat_active=True)
    self.tick(.5,lat_active=True)
    self.tick(1.,fresh=False)
    self.tick(1.5)
    self.tick(2.,in_drive=False)
    self.tick(4.)
    self.tick(3.)
    self.tick(5.)
    self.tick(6.,started=False,trip_id='')
    self.assertEqual(sum(self.stats.snapshot()['total'].values()),.5)

  def test_atomic_persistence_and_same_route_ui_restart(self):
    self.tick(0.,lat_active=True,long_active=True)
    self.tick(1.,lat_active=True,long_active=True)
    self.stats.set_show_onroad(False)
    self.stats.flush()
    other=EngagementStats(self.path)
    self.assertFalse(other.show_onroad)
    self.assertEqual(other.snapshot()['trip']['both'],1.)
    other.sample(now=10.,started=True,fresh=True,in_drive=True,
                 lat_active=True,long_active=True,trip_id='route-a')
    other.sample(now=10.5,started=True,fresh=True,in_drive=True,
                 lat_active=True,long_active=True,trip_id='route-a')
    self.assertEqual(other.snapshot()['trip']['both'],1.5)
    self.assertEqual(list(Path(self.tmp.name).glob('time.json.*')),[])

  def test_new_trip_resets_trip_only_and_offroad_keeps_last_trip(self):
    self.tick(0.,lat_active=True)
    self.tick(1.,lat_active=True)
    self.tick(1.5,started=False,trip_id='')
    self.assertEqual(self.stats.snapshot()['trip']['steering'],1.)
    self.tick(2.,trip_id='route-b')
    self.assertEqual(sum(self.stats.snapshot()['trip'].values()),0.)
    self.assertEqual(self.stats.snapshot()['total']['steering'],1.)

  def test_corrupt_or_unsupported_history_is_never_overwritten(self):
    self.tick(0.);self.tick(1.);self.stats.flush()
    valid=json.loads(self.path.read_text())
    bad=json.loads(self.path.read_text());bad['total']['manual']=-1.
    nan=json.loads(self.path.read_text());nan['total']['manual']=float('nan')
    for value in ('{broken',json.dumps(bad),json.dumps(nan),json.dumps(dict(valid,version=99))):
      self.path.write_text(value)
      other=EngagementStats(self.path)
      other.set_show_onroad(False)
      other.flush()
      self.assertEqual(self.path.read_text(),value)
      self.assertTrue(other.snapshot()['storage_error'])

  def test_disk_failure_does_not_destroy_previous_history(self):
    self.tick(0.);self.tick(1.);self.stats.flush()
    before=self.path.read_text()
    self.tick(2.)
    with patch('openpilot.selfdrive.ui.engagement_stats.os.replace',side_effect=OSError('full')):
      with self.assertRaises(OSError):self.stats.flush()
    self.assertEqual(self.path.read_text(),before)
    self.assertTrue(self.stats.snapshot()['storage_error'])
    self.stats.flush()
    self.assertFalse(self.stats.snapshot()['storage_error'])

  def test_legacy_miles_are_read_only_and_cannot_become_hours(self):
    p=Path(self.tmp.name)/'miles.json'
    value='{"manual":250,"lat":0,"long":0,"both":7500}'
    p.write_text(value)
    self.stats._refresh_mileage()
    self.assertEqual(self.stats.snapshot()['mileage']['engaged'],7500)
    self.assertEqual(sum(self.stats.snapshot()['total'].values()),0.)
    self.assertEqual(p.read_text(),value)

  def test_hide_toggle_and_snapshot_are_independent_of_counters(self):
    self.tick(0.);self.tick(1.)
    snap=self.stats.snapshot();snap['total']['manual']=999.
    self.stats.set_show_onroad(False)
    self.assertFalse(self.stats.show_onroad)
    self.assertEqual(self.stats.snapshot()['total']['manual'],1.)

  def test_unknown_started_state_preserves_trip_but_does_not_count_gap(self):
    self.tick(0.,lat_active=True)
    self.tick(.5,lat_active=True)
    self.stats.pause(1.)
    self.tick(1.5,lat_active=True)
    self.assertEqual(self.stats.snapshot()['trip']['steering'],.5)


class UIIntegrationTests(unittest.TestCase):
  def test_ui_sampling_uses_fresh_reported_authority_and_current_route(self):
    from openpilot.selfdrive.ui import ui_state as module
    with tempfile.TemporaryDirectory() as tmp:
      stats=EngagementStats(Path(tmp)/'stats.json')
      stats.start=lambda:None
      import cereal.messaging as messaging
      sm=messaging.SubMaster(['deviceState','carState','carControl'])
      device=messaging.new_message('deviceState');device.deviceState.started=True
      car=messaging.new_message('carState');car.carState.gearShifter='drive'
      control=messaging.new_message('carControl');control.carControl.latActive=True
      for message in (device,car,control):message.valid=True
      ui=N(engagement_stats=stats,sm=sm,started=True,_stats_trip_id=None,
           params=N(get=lambda key:'route-a'))
      for t in (10.,10.5):
        sm.update_msgs(t,[device,car,control])
        with patch.object(module.time,'monotonic',return_value=t):
          module.UIState._update_engagement_stats(ui)
      self.assertEqual(stats.snapshot()['total']['steering'],.5)
      sm.recv_time['carControl']=9.
      with patch.object(module.time,'monotonic',return_value=11.):
        module.UIState._update_engagement_stats(ui)
      self.assertEqual(stats.snapshot()['total']['steering'],.5)

  def test_onroad_strip_is_below_controls_and_away_from_mode_button(self):
    from openpilot.selfdrive.ui.onroad import hud_renderer as hud
    with tempfile.TemporaryDirectory() as tmp:
      stats=EngagementStats(Path(tmp)/'stats.json')
      obj=object.__new__(hud.HudRenderer);obj._font_medium=object()
      rect=hud.rl.Rectangle(0,0,2160,1080)
      with patch.object(hud.ui_state,'engagement_stats',stats,create=True), \
           patch.object(hud,'measure_text_cached',return_value=N(x=700.,y=36.)), \
           patch.object(hud.rl,'draw_rectangle_rounded') as box, \
           patch.object(hud.rl,'draw_text_ex') as text:
        obj._draw_engagement_stats(rect)
        r=box.call_args.args[0]
        self.assertGreater(r.y,975.)
        self.assertLess(r.x+r.width,2160-222)
        self.assertLessEqual(r.y+r.height,1080.)
        self.assertIn('TRIP --%',text.call_args.args[1])
        stats.set_show_onroad(False)
        box.reset_mock();text.reset_mock();obj._draw_engagement_stats(rect)
        box.assert_not_called();text.assert_not_called()
