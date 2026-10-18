#!/usr/bin/env python3
"""Build a compact read-only region database on a computer, not while driving.

Requires pyosmium only for import. Runtime lookup uses the standard library.
Input is a complete Geofabrik/OSM PBF extract; output replaces atomically only
following integrity and geometry checks. See --help for reproducible inputs.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sqlite3
import tempfile
import time
import zlib

SCHEMA_VERSION = 1


def build(pbf, output, region, source_url):
  import osmium
  pbf,output = Path(pbf),Path(output)
  output.parent.mkdir(parents=True,exist_ok=True)
  fd,temporary = tempfile.mkstemp(prefix=output.name+'.',suffix='.partial',dir=output.parent)
  os.close(fd)
  db = sqlite3.connect(temporary)
  try:
    db.executescript('''PRAGMA journal_mode=OFF; PRAGMA synchronous=OFF;
      PRAGMA cache_size=-16384;
      CREATE TABLE metadata(key TEXT PRIMARY KEY,value TEXT NOT NULL);
      CREATE TABLE roads(id INTEGER PRIMARY KEY,tags TEXT NOT NULL,geometry BLOB NOT NULL);
      CREATE VIRTUAL TABLE road_index USING rtree(id,min_lon,max_lon,min_lat,max_lat);
      CREATE TABLE node_features(id INTEGER PRIMARY KEY,tags TEXT NOT NULL);''')
    class Importer(osmium.SimpleHandler):
      def __init__(self):
        super().__init__()
        self.ways=0;self.features=0;self.skipped=0
      def node(self,n):
        if n.tags.get('highway') in ('stop','traffic_signals','give_way'):
          db.execute('INSERT INTO node_features VALUES (?,?)',(n.id,json.dumps(dict(n.tags),separators=(',',':'))))
          self.features+=1
      def way(self,w):
        if not w.tags.get('highway'):
          return
        if len(w.nodes)<2:
          self.skipped+=1
          return
        if any(not n.location.valid() for n in w.nodes):
          raise ValueError(f'missing geometry in highway {w.id}')
        points=[[n.ref,n.lat,n.lon] for n in w.nodes]
        lats=[n[1] for n in points];lons=[n[2] for n in points]
        geometry=zlib.compress(json.dumps(points,separators=(',',':')).encode(),6)
        db.execute('INSERT INTO roads VALUES (?,?,?)',(w.id,json.dumps(dict(w.tags),separators=(',',':')),geometry))
        db.execute('INSERT INTO road_index VALUES (?,?,?,?,?)',(w.id,min(lons),max(lons),min(lats),max(lats)))
        self.ways+=1
        if self.ways%10000==0:
          db.commit()
          print(json.dumps({'roads_imported':self.ways,'node_features':self.features}),flush=True)
    handler=Importer()
    reader=osmium.io.Reader(str(pbf));header=reader.header();timestamp=header.get('osmosis_replication_timestamp') or header.get('timestamp') or '';reader.close()
    # Disk-backed node-location index keeps import memory independent of
    # statewide node count. This temporary build storage stays on the host.
    with tempfile.TemporaryDirectory(prefix='nap-osm-index-',dir=output.parent) as tmp:
      handler.apply_file(str(pbf),locations=True,idx='sparse_file_array,'+str(Path(tmp)/'nodes.idx'))
    if handler.ways==0:
      raise ValueError('extract contains no road geometry')
    bounds=list(db.execute('SELECT min(min_lon),max(max_lon),min(min_lat),max(max_lat) FROM road_index').fetchone())
    digest=hashlib.sha256()
    with pbf.open('rb') as f:
      for chunk in iter(lambda:f.read(1024*1024),b''):digest.update(chunk)
    metadata={'schema_version':str(SCHEMA_VERSION),'complete':'1','region':region,'source_url':source_url,
      'source_timestamp':timestamp,'source_sha256':digest.hexdigest(),'built_at':time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime()),
      'bounds':json.dumps(bounds),'road_count':str(handler.ways),'node_feature_count':str(handler.features),
      'skipped_degenerate_ways':str(handler.skipped),
      'attribution':'OpenStreetMap contributors; ODbL 1.0; extract by Geofabrik',
      'license_url':'https://www.openstreetmap.org/copyright'}
    db.executemany('INSERT INTO metadata VALUES (?,?)',metadata.items());db.commit()
    if db.execute('PRAGMA integrity_check').fetchone()!=('ok',):raise ValueError('road database integrity check failed')
    if db.execute('SELECT count(*) FROM roads').fetchone()[0]!=db.execute('SELECT count(*) FROM road_index').fetchone()[0]:raise ValueError('incomplete spatial index')
    db.execute('PRAGMA optimize');db.close()
    with open(temporary,'rb') as f:os.fsync(f.fileno())
    os.replace(temporary,output)
    directory=os.open(output.parent,os.O_RDONLY)
    try:os.fsync(directory)
    finally:os.close(directory)
    print(json.dumps({'output':str(output),'bytes':output.stat().st_size,'metadata':metadata}),flush=True)
    return metadata
  finally:
    db.close()
    Path(temporary).unlink(missing_ok=True)


def main():
  parser=argparse.ArgumentParser(description=__doc__)
  parser.add_argument('pbf');parser.add_argument('output')
  parser.add_argument('--region',required=True);parser.add_argument('--source-url',required=True)
  args=parser.parse_args()
  build(args.pbf,args.output,args.region,args.source_url)


if __name__=='__main__':main()
