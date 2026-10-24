"""Prepared optional speech, played by soundd's existing output stream."""
from pathlib import Path
import threading
import time
import wave
import numpy as np
from openpilot.selfdrive.nap.spoken_status_v2 import SERVICE, StatusPolicy, read_enabled


PHRASES = ('driver', 'lateral', 'longitudinal', 'assistance', 'steering_on',
           'steering_paused', 'speed_paused', 'off')
CHIME_GAP = .45
MIN_SPEECH_VOLUME = .30
MAX_SPEECH_VOLUME = .85


class SpokenAudio:
  def __init__(self, basedir):
    self.policy = StatusPolicy()
    self.sock = None
    self.enabled = False
    self.playing = None
    self.callback_clip = None
    self.position = 0
    self.clips = {}
    self.failed = False
    self.allowed = False
    self.updated_at = float('-inf')
    self.quiet_until = 0.
    try:
      for name in PHRASES:
        with wave.open(str(Path(basedir) / 'selfdrive/assets/sounds' / ('nap_' + name + '.wav')), 'rb') as w:
          if w.getnchannels() != 1 or w.getsampwidth() != 2 or w.getframerate() != 48000 or w.getnframes() > 48000 * 4:
            raise ValueError('Unsupported speech clip')
          self.clips[name] = np.frombuffer(w.readframes(w.getnframes()), dtype=np.int16).astype(np.float32) / 32768.
          if not len(self.clips[name]) or not np.all(np.isfinite(self.clips[name])):
            raise ValueError('Empty speech clip')
          if np.max(np.abs(self.clips[name])) > .901:
            raise ValueError('Speech clip exceeds prepared peak limit')
    except Exception:
      self.failed = True
      return
    threading.Thread(target=self._settings, name='nap_speech_settings', daemon=True).start()

  def _settings(self):
    while True:
      self.enabled = read_enabled()
      time.sleep(.5)

  def cancel(self):
    self.playing = None

  def update(self, sm, alert_active, chime_active=False):
    try:
      from cereal import messaging
      if not self.enabled or self.failed:
        self.allowed = False
        self.sock = None
        self.policy.reset()
        self.cancel()
        return
      if self.sock is None:
        self.sock = messaging.sub_sock(SERVICE, conflate=True, timeout=0)
      now = time.monotonic()
      self.updated_at = now
      msg = messaging.recv_one_or_none(self.sock)
      if msg is not None:
        if not msg.valid or not self.policy.ingest(bytes(getattr(msg, SERVICE)), now):
          self.cancel()
      ss = sm['selfdriveState']
      fresh = (sm.seen['selfdriveState'] and sm.valid['selfdriveState']
               and 0 <= now - sm.recv_time['selfdriveState'] <= .5)
      consistent = self.policy.latest is not None and ss.enabled == self.policy.latest['engaged']
      quiet = (ss.alertSound.raw == 0 and ss.alertStatus.raw == 0
               and ss.alertSize.raw == 0 and not ss.alertType)
      # Only ordinary enable/user-disable chimes may defer a status. Any other
      # alert consumes the transition, so speech cannot replay it afterwards.
      benign = (chime_active and ss.alertStatus.raw == 0 and ss.alertSize.raw == 0
                and (not ss.alertType or ss.alertType.rsplit('/', 1)[-1] in ('enable', 'userDisable')))
      suppress = not fresh or not (quiet or benign) or (alert_active and not chime_active)
      if alert_active or suppress:
        self.quiet_until = max(self.quiet_until, now + CHIME_GAP)
      self.allowed = consistent and not suppress and not alert_active and quiet and now >= self.quiet_until
      phrase = self.policy.update(now, self.allowed, suppress=suppress)
      if not self.allowed or self.policy.latest is None:
        self.cancel()
      elif self.playing is not None and self.policy.state(self.policy.latest) != self.policy.stable:
        self.cancel()
      if phrase and self.playing is None:
        self.playing = self.clips[phrase]
    except Exception:
      # Optional audio must never stop critical alert processing.
      self.allowed = False
      self.cancel()
      self.policy.reset()
      self.sock = None
      self.failed = True

  def render(self, output, blocked, ambient_volume=0.):
    # Audio callback: prepared samples only; no I/O, sockets, locks or waits.
    now = time.monotonic()
    if blocked:
      # Include the queued output buffer before the post-chime quiet gap.
      self.quiet_until = max(self.quiet_until, now + len(output) / 48000. + CHIME_GAP)
    blocked = blocked or not self.allowed or not 0 <= now - self.updated_at <= .2 or now < self.quiet_until
    clip = self.playing if self.enabled and not blocked else None
    if clip is None:
      self.callback_clip = None
      self.position = 0
      if blocked:
        self.cancel()
      return
    if clip is not self.callback_clip:
      self.callback_clip = clip
      self.position = 0
    count = min(len(output), len(clip) - self.position)
    if count > 0:
      # Speech has its own floor; cabin-noise attenuation applies only once.
      volume = float(np.clip(ambient_volume if np.isfinite(ambient_volume) else MIN_SPEECH_VOLUME,
                             MIN_SPEECH_VOLUME, MAX_SPEECH_VOLUME))
      output[:count] = clip[self.position:self.position + count] * volume
      self.position += count
    if self.position >= len(clip) and self.playing is clip:
      self.playing = None
