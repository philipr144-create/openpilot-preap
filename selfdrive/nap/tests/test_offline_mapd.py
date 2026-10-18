"""Real map-daemon offline sourcing; networking is forbidden in covered areas."""
from types import SimpleNamespace as N
from unittest.mock import patch
import pytest
from openpilot.selfdrive.nap import nap_mapd as m
from openpilot.selfdrive.nap.offline_roads import OfflineRoadStore
from openpilot.selfdrive.nap.tests.test_offline_roads import database


def daemon(tmp_path):
  obj=m.NapMapD.__new__(m.NapMapD)
  obj.offline_roads=OfflineRoadStore(tmp_path);obj.offline_metadata=None;obj.offline_error=None
  obj._offline_checked_position=None;obj.map_source='osm_overpass'
  obj.cache=None;obj.nodes={};obj.ways=[];obj.last_error=None;obj.last_fetch_attempt=0.
  return obj


def test_covered_lookup_and_movement_never_call_network(tmp_path):
  database(tmp_path/'missouri.sqlite3');d=daemon(tmp_path)
  with patch.object(m,'query_osm',side_effect=AssertionError('network forbidden')) as query, \
       patch.object(m,'save_cache',side_effect=AssertionError('must not overwrite online cache')):
    for lon in [-92.,-92.,-91.994]:
      d.fetch_if_needed(38.,lon)
      assert d.map_source=='offline_osm' and d.ways and d.last_error is None
    query.assert_not_called()
  assert d.offline_metadata['region']=='Missouri'
  assert d.nodes[2].tags['highway']=='stop'


def test_offline_restart_does_not_depend_on_online_cache(tmp_path):
  database(tmp_path/'missouri.sqlite3')
  with patch.object(m,'query_osm',side_effect=AssertionError('network forbidden')):
    for _ in range(2):
      d=daemon(tmp_path);d.fetch_if_needed(38.,-92.)
      assert d.map_source=='offline_osm' and len(d.ways)==1


def test_no_local_region_preserves_existing_online_fallback(tmp_path):
  d=daemon(tmp_path)
  with patch.object(m,'query_osm',side_effect=RuntimeError('offline')) as query,patch.object(m.time,'monotonic',return_value=100.):
    d.fetch_if_needed(38.,-92.)
    query.assert_called_once()
  assert d.last_error=='offline' and not d.ways


def test_existing_loaded_geometry_survives_failed_download_outside_region(tmp_path):
  database(tmp_path/'missouri.sqlite3');d=daemon(tmp_path);d.fetch_if_needed(38.,-92.)
  original=d.ways
  with patch.object(m,'query_osm',side_effect=RuntimeError('offline')),patch.object(m.time,'monotonic',return_value=100.):
    d.fetch_if_needed(40.,-92.)
  assert d.ways is original and d.last_error=='offline'
