"""Offline region persistence, spatial completeness, and failure isolation."""
import json
from pathlib import Path
import sqlite3
import zlib

import pytest
from openpilot.selfdrive.nap.offline_roads import OfflineRoadStore


def database(path, geometry=None, complete='1', version='1'):
  db=sqlite3.connect(path)
  db.executescript('''CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT);
    CREATE TABLE roads(id INTEGER PRIMARY KEY,tags TEXT,geometry BLOB);
    CREATE VIRTUAL TABLE road_index USING rtree(id,min_lon,max_lon,min_lat,max_lat);
    CREATE TABLE node_features(id INTEGER PRIMARY KEY,tags TEXT);''')
  meta={'schema_version':version,'complete':complete,'bounds':'[-93,-91,37,39]',
        'region':'Missouri','built_at':'2000-01-01T00:00:00Z','source_timestamp':'2000-01-01T00:00:00Z'}
  db.executemany('INSERT INTO metadata VALUES (?,?)',meta.items())
  # Both endpoints outside a small query box, but their segment crosses it.
  points=geometry or [[1,38.,-92.02],[2,38.,-92.],[3,38.,-91.98]]
  db.execute('INSERT INTO roads VALUES (7,?,?)',(json.dumps({'highway':'primary','name':'Test','maxspeed':'55 mph','oneway':'yes'}),zlib.compress(json.dumps(points).encode())))
  db.execute('INSERT INTO road_index VALUES (7,-92.02,-91.98,38,38)')
  db.execute('INSERT INTO node_features VALUES (2,?)',(json.dumps({'highway':'stop','direction':'forward'}),))
  db.commit();db.close()


def test_persistent_old_geometry_and_full_connectivity_survive_reopen(tmp_path):
  database(tmp_path/'missouri.sqlite3')
  for _ in range(2):
    data,info=OfflineRoadStore(tmp_path).lookup(38.,-92.,100.)
    nodes={n['id']:n for n in data['elements'] if n['type']=='node'}
    way=next(n for n in data['elements'] if n['type']=='way')
    assert way['nodes']==[1,2,3] and way['tags']['maxspeed']=='55 mph'
    assert nodes[2]['tags']=={'highway':'stop','direction':'forward'}
    assert info['source_timestamp'].startswith('2000-')


def test_spatial_query_finds_crossing_segments_not_just_nearby_vertices(tmp_path):
  database(tmp_path/'missouri.sqlite3',[[1,38.,-92.02],[3,38.,-91.98]])
  data,_=OfflineRoadStore(tmp_path).lookup(38.,-92.,100.)
  assert any(n['type']=='way' for n in data['elements'])


def test_missing_or_out_of_coverage_region_does_not_create_files(tmp_path):
  reader=OfflineRoadStore(tmp_path/'missing')
  assert reader.lookup(38.,-92.,1100.) is None
  assert not (tmp_path/'missing').exists()
  database(tmp_path/'missouri.sqlite3')
  assert OfflineRoadStore(tmp_path).lookup(40.,-92.,1100.) is None
  assert OfflineRoadStore(tmp_path).lookup(37.1,-92.,1100.) is None


@pytest.mark.parametrize('kwargs',[{'complete':'0'},{'version':'9'}])
def test_incomplete_or_unsupported_database_is_rejected(tmp_path,kwargs):
  database(tmp_path/'missouri.sqlite3',**kwargs)
  reader=OfflineRoadStore(tmp_path)
  assert reader.lookup(38.,-92.,1100.) is None and reader.last_error


def test_corrupt_database_is_preserved_and_other_regions_still_work(tmp_path):
  bad=tmp_path/'a.sqlite3';bad.write_bytes(b'corrupt')
  database(tmp_path/'missouri.sqlite3')
  assert OfflineRoadStore(tmp_path).lookup(38.,-92.,1100.) is not None
  assert bad.read_bytes()==b'corrupt'


def test_atomic_map_replacement_is_seen_without_restart(tmp_path):
  path=tmp_path/'missouri.sqlite3';database(path)
  reader=OfflineRoadStore(tmp_path)
  assert reader.lookup(38.,-92.,1100.)[1]['region']=='Missouri'
  replacement=tmp_path/'new.partial';database(replacement)
  db=sqlite3.connect(replacement);db.execute("UPDATE metadata SET value='Updated Missouri' WHERE key='region'");db.commit();db.close()
  replacement.replace(path)
  assert reader.lookup(38.,-92.,1100.)[1]['region']=='Updated Missouri'


@pytest.mark.parametrize('position',[(float('nan'),-92.,1100.),(38.,float('inf'),1100.),(90.,0.,1100.),(38.,-92.,-1.)])
def test_bad_positions_cannot_select_roads(tmp_path,position):
  assert OfflineRoadStore(tmp_path).lookup(*position) is None


def test_expansion_limit_rejects_partial_geometry(tmp_path):
  path=tmp_path/'missouri.sqlite3';database(path)
  db=sqlite3.connect(path);db.execute('UPDATE roads SET geometry=?',(zlib.compress(b' '*1100000),));db.commit();db.close()
  reader=OfflineRoadStore(tmp_path)
  assert reader.lookup(38.,-92.,1100.) is None and 'oversized' in reader.last_error


def test_way_and_node_budgets_reject_whole_result(tmp_path,monkeypatch):
  from openpilot.selfdrive.nap import offline_roads as module
  database(tmp_path/'missouri.sqlite3')
  monkeypatch.setattr(module,'MAX_NODES',2)
  reader=OfflineRoadStore(tmp_path)
  assert reader.lookup(38.,-92.,1100.) is None and 'node count' in reader.last_error
  monkeypatch.setattr(module,'MAX_NODES',200000)
  monkeypatch.setattr(module,'MAX_WAYS',0)
  assert reader.lookup(38.,-92.,1100.) is None and 'road count' in reader.last_error


def test_existing_road_filter_runs_before_geometry_expansion(tmp_path):
  path=tmp_path/'missouri.sqlite3';database(path)
  db=sqlite3.connect(path);db.execute('UPDATE roads SET geometry=?',(b'invalid geometry for an excluded road',));db.commit();db.close()
  reader=OfflineRoadStore(tmp_path)
  assert reader.lookup(38.,-92.,1100.,way_filter=lambda tags:False) is None
  assert reader.last_error is None


def test_candidate_budget_is_separate_from_selected_road_budget(tmp_path,monkeypatch):
  from openpilot.selfdrive.nap import offline_roads as module
  database(tmp_path/'missouri.sqlite3');monkeypatch.setattr(module,'MAX_CANDIDATES',0)
  reader=OfflineRoadStore(tmp_path)
  assert reader.lookup(38.,-92.,1100.) is None and 'candidate count' in reader.last_error
