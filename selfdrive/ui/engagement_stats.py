"""Time-based assistance statistics; independent of vehicle control and mileage.

Sampling is cheap and synchronous with UI telemetry. Persistence is atomic and
runs on a background worker, started only after the manager has forked the UI.
"""
import atexit
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time

STATS_PATH = '/data/nap_engagement_time_v1.json'
MILEAGE_PATH = '/data/nap_miles.json'
MODES = ('manual', 'steering', 'speed', 'both')


def empty_counts():
  return dict.fromkeys(MODES, 0.0)


def validate_counts(value):
  if not isinstance(value, dict):
    raise ValueError('Invalid counters')
  result = {}
  for mode in MODES:
    n = value.get(mode)
    if type(n) not in (int, float) or not math.isfinite(n) or n < 0:
      raise ValueError('Invalid elapsed time')
    result[mode] = float(n)
  return result


def summarize(counts):
  drive = sum(counts.values())
  engaged = drive - counts['manual']
  return dict(drive=drive, engaged=engaged, percent=100.*engaged/drive if drive else None)


def duration(seconds):
  minutes = max(0, int(seconds // 60))
  return f'{minutes // 60}h {minutes % 60:02d}m'


def summary_text(counts):
  s = summarize(counts)
  pct = f"{s['percent']:.1f}%" if s['percent'] is not None else '--%'
  return f"{pct} · {duration(s['engaged'])}"


def hud_text(snapshot):
  values = []
  for key, label in (('trip', 'TRIP'), ('total', 'TOTAL')):
    s = summarize(snapshot[key])
    pct = f"{s['percent']:.0f}%" if s['percent'] is not None else '--%'
    values.append(f"{label} {pct} · {s['engaged'] / 3600:.2f}h")
  return '     '.join(values)


class EngagementStats:
  def __init__(self, path=STATS_PATH, mileage_path=MILEAGE_PATH):
    self.path = Path(path)
    self.mileage_path = Path(mileage_path)
    self._lock = threading.Lock()
    self._flush_lock = threading.Lock()
    self._wake = threading.Event()
    self._stop = threading.Event()
    self._worker = None
    self._generation = 1
    self._saved_generation = 0
    self._previous = None
    self._last_now = None
    self.storage_error = ''
    self._read_failed = False
    self._mileage = None
    self.data = dict(version=1, started_at=time.time(), show_onroad=True,
                     total=empty_counts(), trip=empty_counts(), trip_id='', trip_open=False)
    try:
      if self.path.stat().st_size > 16384:
        raise ValueError('Oversized statistics')
      value = json.loads(self.path.read_text())
      if not isinstance(value, dict) or value.get('version') != 1:
        raise ValueError('Unsupported statistics')
      for key in ('total', 'trip'):
        value[key] = validate_counts(value[key])
      if (type(value.get('started_at')) not in (int, float)
          or not math.isfinite(value['started_at']) or value['started_at'] <= 0
          or type(value.get('show_onroad')) is not bool
          or type(value.get('trip_open')) is not bool
          or not isinstance(value.get('trip_id'), str)):
        raise ValueError('Invalid statistics metadata')
      self.data = {key:value[key] for key in self.data}
    except FileNotFoundError:
      pass
    except (OSError, ValueError, TypeError, KeyError):
      # Preserve an unreadable history instead of overwriting it with zeros.
      self._read_failed = True
      self.storage_error = 'Saved history unavailable; original file preserved'

  def start(self):
    if self._worker is None:
      self._worker = threading.Thread(target=self._run, name='nap_engagement_stats', daemon=True)
      self._worker.start()
      atexit.register(self.close)

  def close(self):
    self._stop.set()
    self._wake.set()
    if self._worker is not None and self._worker is not threading.current_thread():
      self._worker.join(timeout=1.)
    try:
      self.flush()
    except OSError:
      pass

  def pause(self, now):
    with self._lock:
      self._previous = None
      self._last_now = now

  def sample(self, *, now, started, fresh, in_drive, lat_active, long_active, trip_id):
    if not math.isfinite(now):
      return
    with self._lock:
      if started and trip_id:
        if not self.data['trip_open'] or self.data['trip_id'] != trip_id:
          self.data['trip'] = empty_counts()
          self.data['trip_id'] = trip_id
          self.data['trip_open'] = True
          self._previous = None
          self._generation += 1
          self._wake.set()
      elif not started and self.data['trip_open']:
        self.data['trip_open'] = False
        self._generation += 1
        self._wake.set()
      mode = ('both' if lat_active and long_active else 'steering' if lat_active
              else 'speed' if long_active else 'manual') if started and fresh and in_drive and trip_id else None
      dt = now - self._last_now if self._last_now is not None else 0.
      # Credit the preceding valid interval, never gaps or stale telemetry.
      if self._previous is not None and mode is not None and 0. < dt <= 1.:
        self.data['total'][self._previous] += dt
        self.data['trip'][self._previous] += dt
        self._generation += 1
      self._previous = mode if dt >= 0. else None
      self._last_now = now

  @property
  def show_onroad(self):
    with self._lock:
      return self.data['show_onroad']

  def set_show_onroad(self, enabled):
    with self._lock:
      self.data['show_onroad'] = bool(enabled)
      self._generation += 1
      self._wake.set()

  def snapshot(self):
    with self._lock:
      return dict(self.data, total=dict(self.data['total']), trip=dict(self.data['trip']),
                  mileage=dict(self._mileage) if self._mileage else None, storage_error=self.storage_error)

  def _refresh_mileage(self):
    try:
      raw = self.mileage_path.read_text()
      if len(raw) > 8192:
        return
      d = json.loads(raw)
      vals = [d.get(k, 0.) for k in ('manual', 'lat', 'long', 'both')]
      if not all(type(v) in (int, float) and math.isfinite(v) and v >= 0 for v in vals):
        return
      mileage = dict(manual=vals[0], engaged=sum(vals[1:]))
      with self._lock:
        self._mileage = mileage
    except (OSError, ValueError, TypeError, AttributeError):
      pass

  def flush(self):
    with self._flush_lock:
      with self._lock:
        if self._read_failed or self._generation == self._saved_generation:
          return
        generation = self._generation
        data = dict(self.data, total=dict(self.data['total']), trip=dict(self.data['trip']))
      blob = json.dumps(data, allow_nan=False, separators=(',', ':'))
      temporary = None
      try:
        with tempfile.NamedTemporaryFile(mode='w', dir=self.path.parent,
                                         prefix=self.path.name+'.', delete=False) as f:
          temporary = f.name
          f.write(blob)
          f.flush()
          os.fsync(f.fileno())
        os.replace(temporary, self.path)
        temporary = None
        with self._lock:
          self._saved_generation = generation
          self.storage_error = ''
      except OSError:
        with self._lock:
          self.storage_error = 'Time totals are not being saved'
        raise
      finally:
        if temporary is not None:
          try:
            os.unlink(temporary)
          except OSError:
            pass

  def _run(self):
    while not self._stop.is_set():
      try:
        self._refresh_mileage()
        self.flush()
      except OSError:
        pass
      self._wake.wait(5.)
      self._wake.clear()
