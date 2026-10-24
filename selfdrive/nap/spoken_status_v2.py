"""Optional bounded status transport; no driving-state subscriptions or audio I/O."""
import json
import math
import os
import struct
import threading
from pathlib import Path
import time
import uuid

SERVICE = 'customReservedRawData0'
CONFIG = '/data/nap_settings/spoken_status_v2.json'
MAX_AGE = .5
COOLDOWN = 3.
PENDING_AGE = 2.5


def read_enabled(path=CONFIG):
  try:
    p = Path(path)
    if p.is_symlink() or p.stat().st_size > 128:
      return False
    return json.loads(p.read_text()).get('enabled') is True
  except (OSError, ValueError, AttributeError, TypeError):
    return False


def write_enabled(enabled, path=CONFIG):
  p = Path(path)
  p.parent.mkdir(parents=True, exist_ok=True)
  tmp = p.with_name(p.name + '.tmp')
  tmp.write_text(json.dumps({'enabled': bool(enabled)}))
  os.replace(tmp, p)


def publisher_available():
  # Check the existing Linux msgq header without registering a reader. Never
  # take a reserved channel away from an already running publisher.
  prefix = os.environ.get('OPENPILOT_PREFIX')
  q = Path('/dev/shm/msgq_' + (prefix + '/' if prefix else '') + SERVICE)
  try:
    with q.open('rb') as f:
      header = f.read(24)
    if len(header) != 24:
      return False
    tid = struct.unpack('=QQQ', header)[2] & 0xffffffff
    return tid in (0, threading.get_native_id()) or not Path('/proc/' + str(tid)).exists()
  except FileNotFoundError:
    return True
  except (OSError, ValueError):
    return False


class StatusPublisher:
  """Called only by card's existing params thread, never its control loop."""
  def __init__(self):
    self.pm = None
    self.session = uuid.uuid4().hex
    self.seq = 0
    self.failed = False

  def update(self, snapshot, enabled):
    if not enabled or self.failed:
      self.pm = None
      return
    now = time.monotonic()
    if snapshot is None or not 0 <= now - snapshot[0] <= MAX_AGE:
      return
    try:
      from cereal import messaging
      if self.pm is None:
        if not publisher_available():
          self.failed = True
          return
        self.pm = messaging.PubMaster([SERVICE])
        self.session = uuid.uuid4().hex
        self.seq = 0
      stamp, engaged, driver, lateral, longitudinal = snapshot
      self.seq += 1
      payload = json.dumps(dict(v=2, session=self.session, seq=self.seq, time=stamp,
                                engaged=engaged, driver=driver, lateral=lateral,
                                longitudinal=longitudinal), separators=(',', ':')).encode()
      msg = messaging.new_message(None, valid=True)
      setattr(msg, SERVICE, payload)
      self.pm.send(SERVICE, msg)
    except Exception:
      # Optional publisher failures never propagate into card. Do not compete
      # repeatedly with an unexpected publisher on this reserved service.
      self.pm = None
      self.failed = True


def decode(payload, now):
  try:
    if len(payload) > 1024:
      return None
    d = json.loads(payload)
    if d.get('v') != 2 or not isinstance(d.get('session'), str) or not 1 <= len(d['session']) <= 64:
      return None
    if type(d.get('seq')) is not int or d['seq'] <= 0:
      return None
    if type(d.get('time')) not in (float, int) or not math.isfinite(d['time']) or not 0 <= now - d['time'] <= MAX_AGE:
      return None
    if any(type(d.get(k)) is not bool for k in ('engaged', 'driver', 'lateral', 'longitudinal')):
      return None
    return d
  except (ValueError, TypeError, AttributeError, OverflowError):
    return None


class StatusPolicy:
  """One fresh status at a time; startup silent, safety alerts discard events."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.session = None
    self.seq = 0
    self.latest = None
    self.candidate = self.stable = None
    self.changed = 0.
    self.last_spoken = float('-inf')
    self.pending = None

  def ingest(self, data, now):
    d = decode(data, now)
    if d is None:
      self.reset()
      return False
    if d['session'] != self.session:
      self.reset()
      self.session = d['session']
      self.stable = self.candidate = self.state(d)
      self.changed = now
    elif d['seq'] <= self.seq or (self.latest is not None and d['time'] < self.latest['time']):
      return False
    self.seq = d['seq']
    self.latest = d
    return True

  @staticmethod
  def state(d):
    if not d['engaged']:
      return 'off'
    return ('driver' if d['driver'] else 'lateral' if d['lateral'] else 'idle', d['longitudinal'])

  @staticmethod
  def phrase(previous, state):
    if state == 'off':
      return 'off'
    if state[0] == 'driver' and (previous == 'off' or previous[0] != 'driver'):
      return 'driver'
    if previous == 'off':
      return ('assistance' if state == ('lateral', True) else
              'steering_on' if state[0] == 'lateral' else
              'longitudinal' if state[1] else None)
    if state == ('lateral', True) and previous[0] != 'lateral' and not previous[1]:
      return 'assistance'
    if state[0] == 'lateral' and previous[0] != 'lateral':
      return 'lateral'
    if previous[0] == 'lateral' and state[0] != 'lateral':
      return 'steering_paused'
    if state[1] != previous[1]:
      return 'longitudinal' if state[1] else 'speed_paused'
    return None

  def update(self, now, permitted, suppress=False):
    if self.latest is None or not 0 <= now - self.latest['time'] <= MAX_AGE:
      self.reset()
      return None
    state = self.state(self.latest)
    if suppress:
      self.stable = self.candidate = state
      self.changed = now
      self.pending = None
      return None
    if state != self.candidate:
      self.candidate = state
      self.changed = now
      self.pending = None  # Never narrate a superseded state.
    dwell = .2 if state != 'off' and state[0] == 'driver' else .6
    if state != self.stable and now - self.changed >= dwell:
      phrase = self.phrase(self.stable, state)
      self.stable = state
      self.pending = (state, phrase, now) if phrase else None
    if self.pending is not None:
      target, phrase, stamp = self.pending
      if target != state or now - stamp > PENDING_AGE:
        self.pending = None
      elif permitted and now - self.last_spoken >= COOLDOWN:
        self.pending = None
        self.last_spoken = now
        return phrase
    return None
