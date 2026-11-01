"""Optional status edges. No queues, I/O, synthesis, or control outputs."""
class SpokenStatus:
  def __init__(self):
    self.previous = None
    self.pending = None
    self.since = 0.0
    self.last_spoken = float('-inf')

  def update(self, now, *, enabled, healthy, alert, driver, lateral, longitudinal):
    state = (bool(driver), bool(lateral), bool(longitudinal))
    if not enabled or not healthy or alert:
      self.previous = None
      self.pending = None
      return None
    if self.previous is None:
      self.previous = state
      return None
    old = self.previous
    self.previous = state
    # Newest event replaces any pending event; never narrate old actions.
    event = None
    if state[0] and not old[0]:
      event = 'driver'
    elif state[1] and not old[1] and not state[0]:
      event = 'lateral'
    elif state[2] and not old[2]:
      event = 'longitudinal'
    if event:
      self.pending, self.since = event, now
    if self.pending:
      applicable = {'driver': state[0], 'lateral': state[1] and not state[0],
                    'longitudinal': state[2]}[self.pending]
      if not applicable or now - self.since > 1.0:
        self.pending = None
      elif now - self.since >= .5:
        event, self.pending = self.pending, None
        if now - self.last_spoken >= 5.0:
          self.last_spoken = now
          return event
    return None
