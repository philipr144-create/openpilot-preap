"""Read nearby OSM roads from persistent, spatially indexed region extracts.

Only geometry is cached indefinitely. The map daemon still matches current
GPS and publishes short-lived control snapshots using its existing rules.
No speed targets or vehicle controls are owned by this module.
"""
import json
import math
from pathlib import Path
import sqlite3
import zlib

MAP_DIRECTORY = '/data/nap_maps'
SCHEMA_VERSION = 1
MAX_WAYS = 6000
MAX_CANDIDATES = 30000
MAX_NODES = 200000
MAX_GEOMETRY_BYTES = 1024 * 1024
EARTH_RADIUS_M = 6371000.


class OfflineRoadStore:
  def __init__(self, directory=MAP_DIRECTORY):
    self.directory = Path(directory)
    self.last_error = None

  def lookup(self, lat, lon, radius_m, way_filter=None):
    """Return (Overpass-compatible data, provenance), or None without coverage.

    Connections are read-only and short-lived, so atomically replacing a
    region file cannot leave a daemon pinned to an obsolete database.
    Errors never return partial geometry and never delete installed maps.
    """
    self.last_error = None
    if not all(math.isfinite(v) for v in (lat, lon, radius_m)) or not -85. <= lat <= 85. or not -180. <= lon <= 180. or not 0. < radius_m <= 10000.:
      return None
    dy = math.degrees(radius_m / EARTH_RADIUS_M)
    dx = dy / math.cos(math.radians(lat))
    bounds = (lon - dx, lon + dx, lat - dy, lat + dy)
    try:
      paths = sorted(self.directory.glob('*.sqlite3'))
    except OSError as e:
      self.last_error = str(e)
      return None
    # Each region is a self-contained extract. Prefer a region containing
    # roads at this position; do not merge conflicting versions of a way.
    for path in paths:
      try:
        result = self._read(path, bounds, way_filter)
        if result is not None:
          self.last_error = None
          return result
      except (OSError, sqlite3.Error, ValueError, TypeError, KeyError, zlib.error) as e:
        self.last_error = f'{path.name}: {e}'
    return None

  def _read(self, path, bounds, way_filter=None):
    db = sqlite3.connect(path.resolve().as_uri() + '?mode=ro&immutable=1', uri=True, timeout=.1)
    try:
      db.execute('PRAGMA query_only=ON')
      db.execute('PRAGMA cache_size=-2048')
      db.execute('PRAGMA mmap_size=0')
      metadata = dict(db.execute('SELECT key,value FROM metadata'))
      if metadata.get('schema_version') != str(SCHEMA_VERSION) or metadata.get('complete') != '1':
        raise ValueError('unsupported or incomplete road database')
      min_lon,max_lon,min_lat,max_lat = bounds
      region_bounds = json.loads(metadata['bounds'])
      if max_lon < region_bounds[0] or min_lon > region_bounds[1] or max_lat < region_bounds[2] or min_lat > region_bounds[3]:
        return None
      rows = db.execute('''SELECT r.id,r.tags FROM road_index AS i
        JOIN roads AS r ON r.id=i.id WHERE i.min_lon<=? AND i.max_lon>=?
        AND i.min_lat<=? AND i.max_lat>=? ORDER BY r.id LIMIT ?''',
        (max_lon,min_lon,max_lat,min_lat,MAX_CANDIDATES+1))
      nodes = {}
      ways = []
      for count,(way_id,tags_raw) in enumerate(rows,1):
        if count>MAX_CANDIDATES:
          raise ValueError('nearby candidate count exceeds lookup budget')
        tags = json.loads(tags_raw)
        if not isinstance(tags,dict):
          raise ValueError('invalid road tags')
        # Apply the caller's existing road predicate before decompressing
        # geometry, so pedestrian-dense city centers stay memory-bounded.
        if way_filter is not None and not way_filter(tags):
          continue
        if len(ways)>=MAX_WAYS:
          raise ValueError('nearby road count exceeds memory budget')
        geometry = db.execute('SELECT geometry FROM roads WHERE id=?',(way_id,)).fetchone()[0]
        inflater = zlib.decompressobj()
        raw = inflater.decompress(geometry, MAX_GEOMETRY_BYTES)
        if not inflater.eof or inflater.unconsumed_tail:
          raise ValueError('invalid or oversized road geometry')
        points = json.loads(raw)
        if not isinstance(points,list) or len(points)<2:
          raise ValueError('invalid road geometry')
        refs = []
        for node_id,node_lat,node_lon in points:
          if type(node_id) is not int or not math.isfinite(node_lat+node_lon) or not -90. <= node_lat <= 90. or not -180. <= node_lon <= 180.:
            raise ValueError('invalid road node')
          refs.append(node_id)
          nodes[node_id] = {'type':'node','id':node_id,'lat':node_lat,'lon':node_lon,'tags':{}}
        if len(nodes)>MAX_NODES:
          raise ValueError('nearby node count exceeds memory budget')
        ways.append({'type':'way','id':way_id,'nodes':refs,'tags':tags})
      if not ways:
        return None
      ids = list(nodes)
      for start in range(0,len(ids),400):
        batch = ids[start:start+400]
        query = 'SELECT id,tags FROM node_features WHERE id IN ('+','.join('?' for _ in batch)+')'
        for node_id,tags in db.execute(query,batch):
          nodes[node_id]['tags'] = json.loads(tags)
      provenance = {key:metadata.get(key) for key in ('region','source_url','source_timestamp','built_at','attribution')}
      provenance['file'] = path.name
      return {'elements':list(nodes.values())+ways}, provenance
    finally:
      db.close()
