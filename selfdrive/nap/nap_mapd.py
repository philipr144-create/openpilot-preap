#!/usr/bin/env python3

import json
import math
import os
import time
from dataclasses import dataclass

import requests
import cereal.messaging as messaging
from openpilot.common.params import Params


SNAPSHOT = "/dev/shm/nap_map_context.json"
CACHE = "/data/nap_osm_cache.json"

OVERPASS = "https://overpass-api.de/api/interpreter"

EARTH_RADIUS_M = 6371000.0

QUERY_RADIUS_M = 1100
REFETCH_DISTANCE_M = 400
CACHE_MAX_AGE = 6 * 3600

LOOKAHEAD_M = 450.0

# Map matching
MATCH_MAX_DISTANCE_M = 35.0
CONTROL_MATCH_MAX_DISTANCE_M = 20.0
CONTROL_HEADING_MAX_DEG = 35.0

# Geometry processing
RESAMPLE_M = 5.0
CURVATURE_HALF_WINDOW_M = 15.0
CURVE_START_K = 0.0025
CURVE_KEEP_K = 0.0018
MIN_CURVE_LENGTH_M = 20.0
MERGE_GAP_M = 15.0

# First active version is intentionally conservative.
LAT_ACCEL_TARGET = 1.8

PUBLISH_HZ = 2.0


@dataclass
class Node:
  id: int
  lat: float
  lon: float
  tags: dict


@dataclass
class Way:
  id: int
  nodes: list
  tags: dict


def clamp(v, lo, hi):
  return max(lo, min(hi, v))


def angle_diff_deg(a, b):
  return (a - b + 180.0) % 360.0 - 180.0


def haversine_m(lat1, lon1, lat2, lon2):
  p1 = math.radians(lat1)
  p2 = math.radians(lat2)
  dp = math.radians(lat2 - lat1)
  dl = math.radians(lon2 - lon1)

  a = (
    math.sin(dp / 2.0) ** 2
    + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
  )

  return 2.0 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def local_xy(lat, lon, lat0, lon0):
  lat0r = math.radians(lat0)
  x = math.radians(lon - lon0) * EARTH_RADIUS_M * math.cos(lat0r)
  y = math.radians(lat - lat0) * EARTH_RADIUS_M
  return x, y


def bearing_deg_xy(x1, y1, x2, y2):
  return (
    math.degrees(math.atan2(x2 - x1, y2 - y1)) + 360.0
  ) % 360.0


def point_segment_projection(px, py, ax, ay, bx, by):
  vx = bx - ax
  vy = by - ay
  vv = vx * vx + vy * vy

  if vv < 1e-9:
    return ax, ay, 0.0, math.hypot(px - ax, py - ay)

  t = ((px - ax) * vx + (py - ay) * vy) / vv
  t = clamp(t, 0.0, 1.0)

  qx = ax + t * vx
  qy = ay + t * vy

  return qx, qy, t, math.hypot(px - qx, py - qy)


def parse_speed_mps(raw):
  if not raw:
    return None

  s = str(raw).strip().lower()

  if s in ("none", "signals", "variable", "walk", "national"):
    return None

  if ";" in s:
    s = s.split(";", 1)[0].strip()

  try:
    if "mph" in s:
      return float(s.replace("mph", "").strip()) * 0.44704

    return float(s.split()[0]) / 3.6

  except Exception:
    return None


def road_name(tags):
  return (
    tags.get("name")
    or tags.get("ref")
    or tags.get("highway")
    or "unnamed"
  )


def way_allowed(tags):
  highway = tags.get("highway")

  if not highway:
    return False

  excluded = {
    "footway",
    "path",
    "cycleway",
    "steps",
    "pedestrian",
    "bridleway",
    "corridor",
    "construction",
    "proposed",
    "service",
  }

  if highway in excluded:
    return False

  if tags.get("access") in ("private", "no"):
    return False

  return True


def query_osm(lat, lon):
  query = f"""
[out:json][timeout:15];
(
  way(around:{QUERY_RADIUS_M},{lat},{lon})["highway"];
  node(around:{QUERY_RADIUS_M},{lat},{lon})["highway"="stop"];
  node(around:{QUERY_RADIUS_M},{lat},{lon})["highway"="traffic_signals"];
  node(around:{QUERY_RADIUS_M},{lat},{lon})["highway"="give_way"];
);
out body;
>;
out skel qt;
"""

  r = requests.post(
    OVERPASS,
    data={"data": query},
    headers={"User-Agent": "NAP-MapDrivingAssist/0.2"},
    timeout=20,
  )

  r.raise_for_status()
  return r.json()


def decode_osm(data):
  nodes = {}
  way_defs = []

  for e in data.get("elements", []):
    if e.get("type") == "node":
      try:
        nodes[int(e["id"])] = Node(
          id=int(e["id"]),
          lat=float(e["lat"]),
          lon=float(e["lon"]),
          tags=e.get("tags", {}) or {},
        )
      except Exception:
        pass

    elif e.get("type") == "way":
      way_defs.append(e)

  ways = []

  for e in way_defs:
    tags = e.get("tags", {}) or {}

    if not way_allowed(tags):
      continue

    ns = []

    for node_id in e.get("nodes", []):
      n = nodes.get(int(node_id))
      if n is not None:
        ns.append(n)

    if len(ns) >= 2:
      ways.append(
        Way(
          id=int(e["id"]),
          nodes=ns,
          tags=tags,
        )
      )

  return nodes, ways


def load_cache():
  try:
    with open(CACHE) as f:
      c = json.load(f)

    if time.time() - float(c.get("wall_time", 0)) > CACHE_MAX_AGE:
      return None

    return c

  except Exception:
    return None


def save_cache(lat, lon, data):
  tmp = CACHE + ".tmp"

  payload = {
    "wall_time": time.time(),
    "lat": lat,
    "lon": lon,
    "data": data,
  }

  with open(tmp, "w") as f:
    json.dump(payload, f)

  os.replace(tmp, CACHE)


def atomic_json(path, payload):
  tmp = path + ".tmp"

  with open(tmp, "w") as f:
    json.dump(payload, f, separators=(",", ":"))

  os.replace(tmp, path)


def heading_weight(speed):
  # GPS bearing is poor while stopped/creeping.
  if speed <= 1.5:
    return 0.0

  if speed >= 8.0:
    return 32.0

  return 32.0 * (speed - 1.5) / (8.0 - 1.5)


def match_way(lat, lon, bearing, speed, ways):
  best = None
  h_weight = heading_weight(speed)

  for way in ways:
    xy = [
      local_xy(n.lat, n.lon, lat, lon)
      for n in way.nodes
    ]

    for i in range(len(xy) - 1):
      ax, ay = xy[i]
      bx, by = xy[i + 1]

      _, _, t, distance = point_segment_projection(
        0.0, 0.0, ax, ay, bx, by
      )

      seg_bearing = bearing_deg_xy(ax, ay, bx, by)

      forward_error = abs(
        angle_diff_deg(seg_bearing, bearing)
      )

      reverse_error = abs(
        angle_diff_deg(
          (seg_bearing + 180.0) % 360.0,
          bearing,
        )
      )

      if reverse_error < forward_error:
        direction = -1
        heading_error = reverse_error
      else:
        direction = 1
        heading_error = forward_error

      heading_penalty = (
        h_weight * min(heading_error, 90.0) / 90.0
      )

      score = distance + heading_penalty

      candidate = {
        "way": way,
        "segment": i,
        "t": t,
        "direction": direction,
        "distance": distance,
        "heading_error": heading_error,
        "score": score,
      }

      if best is None or score < best["score"]:
        best = candidate

  if best is None:
    return None

  if best["distance"] > MATCH_MAX_DISTANCE_M:
    return None

  return best


def node_way_index(ways):
  idx = {}

  for way in ways:
    for n in way.nodes:
      idx.setdefault(n.id, []).append(way)

  return idx


def way_exit_heading(way, node_id, leaving_from_node):
  if len(way.nodes) < 2:
    return None

  if way.nodes[0].id == node_id:
    a = way.nodes[0]
    b = way.nodes[1]

  elif way.nodes[-1].id == node_id:
    a = way.nodes[-1]
    b = way.nodes[-2]

  else:
    return None

  x1, y1 = local_xy(
    a.lat, a.lon, leaving_from_node.lat, leaving_from_node.lon
  )
  x2, y2 = local_xy(
    b.lat, b.lon, leaving_from_node.lat, leaving_from_node.lon
  )

  return bearing_deg_xy(x1, y1, x2, y2)



def nav_route_points(nav_route):
  points = []

  try:
    coords = nav_route.coordinates

    for i, c in enumerate(coords):
      if i >= 600:
        break

      lat = float(c.latitude)
      lon = float(c.longitude)

      if (
        math.isfinite(lat)
        and math.isfinite(lon)
        and abs(lat) <= 90.0
        and abs(lon) <= 180.0
        and not (lat == 0.0 and lon == 0.0)
      ):
        points.append((lat, lon))

  except Exception:
    return []

  return points


def route_heading_at(lat, lon, route_points):
  if len(route_points) < 2:
    return None, None

  best = None

  for i in range(len(route_points) - 1):
    a = route_points[i]
    b = route_points[i + 1]

    ax, ay = local_xy(
      a[0], a[1], lat, lon
    )

    bx, by = local_xy(
      b[0], b[1], lat, lon
    )

    _, _, _, distance = point_segment_projection(
      0.0, 0.0,
      ax, ay,
      bx, by,
    )

    heading = bearing_deg_xy(
      ax, ay,
      bx, by,
    )

    candidate = (
      distance,
      heading,
    )

    if best is None or distance < best[0]:
      best = candidate

  if best is None:
    return None, None

  distance, heading = best

  # A route far away from the current intersection is not
  # trustworthy enough to resolve the road branch.
  if distance > 60.0:
    return None, distance

  return heading, distance


def continuation_score(current_way, candidate, incoming_heading, node):
  heading = way_exit_heading(candidate, node.id, node)

  if heading is None:
    return None

  turn = abs(angle_diff_deg(heading, incoming_heading))

  # Don't choose a near-U-turn continuation.
  if turn > 100.0:
    return None

  score = turn

  cur_name = current_way.tags.get("name")
  new_name = candidate.tags.get("name")

  cur_ref = current_way.tags.get("ref")
  new_ref = candidate.tags.get("ref")

  cur_class = current_way.tags.get("highway")
  new_class = candidate.tags.get("highway")

  if cur_name and new_name and cur_name == new_name:
    score -= 35.0

  if cur_ref and new_ref and cur_ref == new_ref:
    score -= 45.0

  if cur_class == new_class:
    score -= 8.0

  return score


def initial_points(match):
  way = match["way"]
  i = match["segment"]
  t = match["t"]
  direction = match["direction"]

  a = way.nodes[i]
  b = way.nodes[i + 1]

  lat = a.lat + (b.lat - a.lat) * t
  lon = a.lon + (b.lon - a.lon) * t

  pts = [(lat, lon, None, way.id)]

  if direction > 0:
    for n in way.nodes[i + 1:]:
      pts.append((n.lat, n.lon, n, way.id))
  else:
    for n in reversed(way.nodes[:i + 1]):
      pts.append((n.lat, n.lon, n, way.id))

  return pts


def path_distance(points):
  total = 0.0

  for i in range(1, len(points)):
    total += haversine_m(
      points[i - 1][0],
      points[i - 1][1],
      points[i][0],
      points[i][1],
    )

  return total



def extend_path(match, ways, route_points=None):
  points = initial_points(match)

  idx = node_way_index(ways)

  used = {match["way"].id}
  current_way = match["way"]

  path_ambiguous = False
  route_guided = False

  route_points = route_points or []

  while path_distance(points) < LOOKAHEAD_M:
    if len(points) < 2:
      break

    end = points[-1]
    prev = points[-2]

    end_node = end[2]

    if end_node is None:
      break

    x1, y1 = local_xy(
      prev[0], prev[1],
      end[0], end[1],
    )

    incoming_heading = bearing_deg_xy(
      x1, y1,
      0.0, 0.0,
    )

    nav_heading, nav_distance = route_heading_at(
      end[0],
      end[1],
      route_points,
    )

    candidates = []

    for candidate in idx.get(end_node.id, []):
      if candidate.id in used:
        continue

      base_score = continuation_score(
        current_way,
        candidate,
        incoming_heading,
        end_node,
      )

      if base_score is None:
        continue

      candidate_heading = way_exit_heading(
        candidate,
        end_node.id,
        end_node,
      )

      if candidate_heading is None:
        continue

      route_error = None
      score = base_score

      if nav_heading is not None:
        route_error = abs(
          angle_diff_deg(
            candidate_heading,
            nav_heading,
          )
        )

        # Route geometry is an additional hint, not the only
        # factor. Same-road continuity still matters.
        score += 0.85 * route_error

        if route_error <= 20.0:
          score -= 22.0

      candidates.append({
        "score": score,
        "base_score": base_score,
        "route_error": route_error,
        "way": candidate,
      })

    if not candidates:
      break

    candidates.sort(
      key=lambda c: c["score"]
    )

    best = candidates[0]

    # If two roads look nearly equally plausible, do not invent
    # a future road path unless navigation clearly resolves it.
    if len(candidates) >= 2:
      separation = (
        candidates[1]["score"]
        - candidates[0]["score"]
      )

      route_resolves = (
        best["route_error"] is not None
        and best["route_error"] <= 25.0
        and (
          candidates[1]["route_error"] is None
          or candidates[1]["route_error"]
             - best["route_error"] >= 15.0
        )
      )

      if separation < 18.0 and not route_resolves:
        path_ambiguous = True
        break

      if route_resolves:
        route_guided = True

    # Even a single continuation should not require what amounts
    # to a U-turn or an implausible side-road jump.
    if best["base_score"] > 75.0:
      path_ambiguous = True
      break

    next_way = best["way"]

    used.add(next_way.id)

    if next_way.nodes[0].id == end_node.id:
      seq = next_way.nodes[1:]

    elif next_way.nodes[-1].id == end_node.id:
      seq = list(
        reversed(
          next_way.nodes[:-1]
        )
      )

    else:
      break

    for n in seq:
      points.append(
        (
          n.lat,
          n.lon,
          n,
          next_way.id,
        )
      )

      if path_distance(points) >= LOOKAHEAD_M:
        break

    current_way = next_way

  return (
    points,
    used,
    path_ambiguous,
    route_guided,
  )

def raw_geometry(lat0, lon0, points):
  out = []
  cumulative = 0.0
  prev = None

  for lat, lon, node, way_id in points:
    x, y = local_xy(lat, lon, lat0, lon0)

    if prev is not None:
      cumulative += math.hypot(
        x - prev[0],
        y - prev[1],
      )

    out.append({
      "x": x,
      "y": y,
      "distance": cumulative,
      "node": node,
      "way_id": way_id,
    })

    prev = (x, y)

    if cumulative >= LOOKAHEAD_M:
      break

  return out


def resample_geometry(geom):
  if len(geom) < 2:
    return []

  total = geom[-1]["distance"]

  if total < RESAMPLE_M:
    return []

  samples = []

  d = 0.0
  seg = 0

  while d <= total and d <= LOOKAHEAD_M:
    while (
      seg + 1 < len(geom)
      and geom[seg + 1]["distance"] < d
    ):
      seg += 1

    if seg + 1 >= len(geom):
      break

    a = geom[seg]
    b = geom[seg + 1]

    span = b["distance"] - a["distance"]

    if span <= 1e-6:
      d += RESAMPLE_M
      continue

    t = clamp(
      (d - a["distance"]) / span,
      0.0,
      1.0,
    )

    samples.append({
      "distance": d,
      "x": a["x"] + (b["x"] - a["x"]) * t,
      "y": a["y"] + (b["y"] - a["y"]) * t,
    })

    d += RESAMPLE_M

  return samples


def signed_curvature(p0, p1, p2):
  ax = p1["x"] - p0["x"]
  ay = p1["y"] - p0["y"]

  bx = p2["x"] - p1["x"]
  by = p2["y"] - p1["y"]

  a = math.hypot(ax, ay)
  b = math.hypot(bx, by)

  cx = p2["x"] - p0["x"]
  cy = p2["y"] - p0["y"]

  c = math.hypot(cx, cy)

  if min(a, b, c) < 1.0:
    return 0.0

  cross = ax * by - ay * bx

  return 2.0 * cross / (a * b * c)


def curvature_samples(samples):
  if len(samples) < 7:
    return []

  half_n = max(
    2,
    int(round(
      CURVATURE_HALF_WINDOW_M / RESAMPLE_M
    )),
  )

  out = []

  for i in range(half_n, len(samples) - half_n):
    p0 = samples[i - half_n]
    p1 = samples[i]
    p2 = samples[i + half_n]

    k = signed_curvature(p0, p1, p2)

    out.append({
      "distance": p1["distance"],
      "curvature": k,
    })

  return out


def merge_curves(curves):
  segments = []
  active = None

  for sample in curves:
    k = sample["curvature"]
    ak = abs(k)

    if active is None:
      if ak < CURVE_START_K:
        continue

      active = {
        "start": sample["distance"],
        "end": sample["distance"],
        "direction": 1 if k > 0 else -1,
        "samples": [sample],
      }

      continue

    same_direction = (
      (k > 0 and active["direction"] > 0)
      or
      (k < 0 and active["direction"] < 0)
    )

    gap = sample["distance"] - active["end"]

    if (
      same_direction
      and ak >= CURVE_KEEP_K
      and gap <= MERGE_GAP_M
    ):
      active["end"] = sample["distance"]
      active["samples"].append(sample)
      continue

    length = active["end"] - active["start"]

    if length >= MIN_CURVE_LENGTH_M:
      segments.append(active)

    active = None

    if ak >= CURVE_START_K:
      active = {
        "start": sample["distance"],
        "end": sample["distance"],
        "direction": 1 if k > 0 else -1,
        "samples": [sample],
      }

  if active is not None:
    length = active["end"] - active["start"]

    if length >= MIN_CURVE_LENGTH_M:
      segments.append(active)

  return segments


def curve_events(samples):
  raw = curvature_samples(samples)
  merged = merge_curves(raw)

  events = []

  for seg in merged:
    vals = [
      abs(s["curvature"])
      for s in seg["samples"]
    ]

    if not vals:
      continue

    vals_sorted = sorted(vals)

    # Use a strong percentile instead of one maximum point.
    # This suppresses isolated OSM geometry spikes.
    idx = int(
      clamp(
        round(0.75 * (len(vals_sorted) - 1)),
        0,
        len(vals_sorted) - 1,
      )
    )

    k = vals_sorted[idx]

    if k <= 1e-6:
      continue

    target = math.sqrt(
      LAT_ACCEL_TARGET / k
    )

    apex_sample = max(
      seg["samples"],
      key=lambda s: abs(s["curvature"]),
    )

    events.append({
      "start_distance_m": round(seg["start"], 1),
      "apex_distance_m": round(
        apex_sample["distance"], 1
      ),
      "end_distance_m": round(seg["end"], 1),
      "length_m": round(
        seg["end"] - seg["start"], 1
      ),
      "direction": (
        "left"
        if seg["direction"] > 0
        else "right"
      ),
      "curvature": round(k, 6),
      "radius_m": round(1.0 / k, 1),
      "target_speed_mps": round(target, 2),
      "target_speed_mph": round(
        target * 2.236936, 1
      ),
      "sample_count": len(seg["samples"]),
    })

  return events



def build_control_curves(events):
  candidates = []

  for e in events:
    start = e.get("start_distance_m")
    target = e.get("target_speed_mps")

    if (
      not isinstance(start, (int, float))
      or not isinstance(target, (int, float))
      or not math.isfinite(start)
      or not math.isfinite(target)
    ):
      continue

    if start < 10.0:
      continue

    # Geometry indicating > ~69 mph does not need to influence
    # the first version of country/city curve control.
    if target > 31.0:
      continue

    # Protect against isolated map geometry claiming walking-speed
    # road curvature.
    target = max(
      target,
      6.7,
    )

    distance = max(
      0.0,
      start - 12.0,
    )

    approach_cap = math.sqrt(
      target * target
      + 2.0 * 1.2 * distance
    )

    c = dict(e)

    c["target_speed_mps"] = round(
      target, 2
    )

    c["target_speed_mph"] = round(
      target * 2.236936,
      1,
    )

    c["approach_cap_mps"] = round(
      approach_cap, 2
    )

    c["approach_cap_mph"] = round(
      approach_cap * 2.236936,
      1,
    )

    candidates.append(c)

  candidates.sort(
    key=lambda e: e["start_distance_m"]
  )

  return candidates


def node_events(geom):
  events = []

  seen = set()

  for p in geom:
    n = p["node"]

    if n is None or n.id in seen:
      continue

    highway = n.tags.get("highway")

    event_type = None

    if highway == "stop":
      event_type = "stop_sign"

    elif highway == "traffic_signals":
      event_type = "traffic_signal"

    elif highway == "give_way":
      event_type = "give_way"

    if event_type is not None:
      seen.add(n.id)

      events.append({
        "type": event_type,
        "distance_m": round(
          p["distance"], 1
        ),
        "node_id": n.id,
      })

  return events

def match_confidence(match, speed, lookahead):
  if match is None:
    return 0.0

  distance = match["distance"]
  heading = match["heading_error"]

  distance_score = clamp(
    1.0 - distance / CONTROL_MATCH_MAX_DISTANCE_M,
    0.0,
    1.0,
  )

  if speed < 3.0:
    heading_score = 1.0
  else:
    heading_score = clamp(
      1.0 - heading / CONTROL_HEADING_MAX_DEG,
      0.0,
      1.0,
    )

  lookahead_score = clamp(
    lookahead / 200.0,
    0.0,
    1.0,
  )

  return (
    0.50 * distance_score
    + 0.35 * heading_score
    + 0.15 * lookahead_score
  )


class NapMapD:
  def __init__(self):
    self.params = Params()

    self.sm = messaging.SubMaster(
      ["gpsLocation", "navRoute"]
    )

    self.cache = load_cache()

    self.nodes = {}
    self.ways = []

    self.last_fetch_attempt = 0.0
    self.last_error = None

    if self.cache is not None:
      try:
        self.nodes, self.ways = decode_osm(
          self.cache["data"]
        )
      except Exception:
        self.cache = None
        self.nodes = {}
        self.ways = []

  def need_fetch(self, lat, lon):
    if self.cache is None:
      return True

    d = haversine_m(
      lat,
      lon,
      float(self.cache["lat"]),
      float(self.cache["lon"]),
    )

    return d >= REFETCH_DISTANCE_M

  def fetch_if_needed(self, lat, lon):
    if not self.need_fetch(lat, lon):
      return

    now = time.monotonic()

    if now - self.last_fetch_attempt < 20.0:
      return

    self.last_fetch_attempt = now

    try:
      data = query_osm(lat, lon)
      nodes, ways = decode_osm(data)

      if not ways:
        raise RuntimeError(
          "OSM query returned no usable roads"
        )

      save_cache(lat, lon, data)

      self.cache = {
        "wall_time": time.time(),
        "lat": lat,
        "lon": lon,
        "data": data,
      }

      self.nodes = nodes
      self.ways = ways
      self.last_error = None

    except Exception as e:
      self.last_error = str(e)

  def update(self):
    self.sm.update(500)

    enabled = self.params.get_bool(
      "NAPMapDrivingAssist"
    )

    now_mono = time.monotonic()

    payload = {
      "version": 3,
      "wall_time": time.time(),
      "mono_time": now_mono,
      "expires_mono": now_mono + 2.5,
      "enabled": enabled,
      "valid": False,
      "control_valid": False,
      "source": "osm_overpass",
    }

    if (
      not self.sm.alive["gpsLocation"]
      or not self.sm.valid["gpsLocation"]
    ):
      payload["status"] = "waiting_for_gps"
      atomic_json(SNAPSHOT, payload)
      return

    gps = self.sm["gpsLocation"]

    lat = float(gps.latitude)
    lon = float(gps.longitude)
    speed = float(gps.speed)
    bearing = float(gps.bearingDeg)

    payload["gps"] = {
      "latitude": lat,
      "longitude": lon,
      "speed_mps": round(speed, 3),
      "bearing_deg": round(bearing, 2),
    }

    if not enabled:
      payload["status"] = "disabled"
      atomic_json(SNAPSHOT, payload)
      return

    self.fetch_if_needed(lat, lon)

    if not self.ways:
      payload["status"] = "no_map_data"
      payload["error"] = self.last_error
      atomic_json(SNAPSHOT, payload)
      return

    match = match_way(
      lat,
      lon,
      bearing,
      speed,
      self.ways,
    )

    if match is None:
      payload["status"] = "no_road_match"
      payload["error"] = self.last_error
      atomic_json(SNAPSHOT, payload)
      return

    way = match["way"]

    route_points = []

    if (
      self.sm.alive["navRoute"]
      and self.sm.valid["navRoute"]
    ):
      route_points = nav_route_points(
        self.sm["navRoute"]
      )

    (
      points,
      used_ways,
      path_ambiguous,
      route_guided,
    ) = extend_path(
      match,
      self.ways,
      route_points,
    )

    geom = raw_geometry(
      lat,
      lon,
      points,
    )

    samples = resample_geometry(geom)

    curves = curve_events(samples)

    control_curves = build_control_curves(
      curves
    )

    control_curve = (
      min(
        control_curves,
        key=lambda e: e["approach_cap_mps"],
      )
      if control_curves
      else None
    )

    events = node_events(geom)

    lookahead = (
      geom[-1]["distance"]
      if geom
      else 0.0
    )

    confidence = match_confidence(
      match,
      speed,
      lookahead,
    )

    heading_ok = (
      speed < 3.0
      or match["heading_error"]
      <= CONTROL_HEADING_MAX_DEG
    )

    control_valid = (
      match["distance"]
      <= CONTROL_MATCH_MAX_DISTANCE_M
      and heading_ok
      and lookahead >= 100.0
      and confidence >= 0.55
    )

    maxspeed = parse_speed_mps(
      way.tags.get("maxspeed")
    )

    payload.update({
      "valid": True,
      "control_valid": control_valid,
      "status": "ok",
      "road": {
        "way_id": way.id,
        "name": road_name(way.tags),
        "highway": way.tags.get("highway"),
        "maxspeed_raw": way.tags.get("maxspeed"),
        "speed_limit_mps": (
          round(maxspeed, 2)
          if maxspeed is not None
          else None
        ),
        "speed_limit_mph": (
          round(maxspeed * 2.236936, 1)
          if maxspeed is not None
          else None
        ),
      },
      "match": {
        "distance_m": round(
          match["distance"], 1
        ),
        "heading_error_deg": round(
          match["heading_error"], 1
        ),
        "direction": match["direction"],
        "confidence": round(
          confidence, 3
        ),
      },
      "lookahead_m": round(
        lookahead, 1
      ),
      "used_way_count": len(used_ways),
      "path_ambiguous": path_ambiguous,
      "route_guided": route_guided,
      "route_point_count": len(route_points),
      "control_curve": control_curve,
      "control_curves": control_curves[:10],
      "curves": curves[:10],
      # Informational only in v2.
      "events": events[:12],
      "cache": {
        "center_lat": (
          self.cache["lat"]
          if self.cache
          else None
        ),
        "center_lon": (
          self.cache["lon"]
          if self.cache
          else None
        ),
        "age_s": (
          round(
            time.time()
            - float(self.cache["wall_time"]),
            1,
          )
          if self.cache
          else None
        ),
        "ways": len(self.ways),
      },
      "error": self.last_error,
    })

    atomic_json(SNAPSHOT, payload)

  def run(self):
    period = 1.0 / PUBLISH_HZ

    while True:
      started = time.monotonic()

      try:
        self.update()

      except Exception as e:
        try:
          now_mono = time.monotonic()

          atomic_json(
            SNAPSHOT,
            {
              "version": 3,
              "wall_time": time.time(),
              "mono_time": now_mono,
              "expires_mono": now_mono + 1.0,
              "enabled": self.params.get_bool(
                "NAPMapDrivingAssist"
              ),
              "valid": False,
              "control_valid": False,
              "status": "exception",
              "error": str(e),
            },
          )

        except Exception:
          pass

      elapsed = (
        time.monotonic() - started
      )

      time.sleep(
        max(0.0, period - elapsed)
      )


def main():
  NapMapD().run()


if __name__ == "__main__":
  main()
