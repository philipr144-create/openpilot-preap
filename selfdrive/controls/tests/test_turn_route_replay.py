"""Replay recorded, location-free left-intent model samples at 100 Hz."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as N
from unittest.mock import patch

from openpilot.selfdrive.controls.lib import navigation_turn_commit as nt
from openpilot.selfdrive.modeld.navigation_desire import NavigationDesire


class TestRecordedIntent(unittest.TestCase):
  def test_waiting_left_intent_has_no_half_second_opposite_pulses(self):
    rows=json.loads((Path(__file__).parent/'fixtures/left_intent_freshness.json').read_text())
    with tempfile.TemporaryDirectory() as directory, patch.object(nt,'Params'):
      root=Path(directory)
      writer=NavigationDesire(str(root/'nav'),str(root/'decision'),str(root/'signal'))
      reader=nt.NavigationTurnCommit()
      reader.params=N(get_bool=lambda _:False)
      cs=N(vEgo=0.,leftBlinker=True,rightBlinker=False,steeringPressed=False,
           steeringDisengage=False,brakePressed=False,canValid=True,
           gearShifter='drive',leftBlindspot=False,rightBlindspot=False)
      index=0; frame=None; outputs=[]
      with patch.object(nt,'DECISION_PATH',str(root/'decision')):
        for tick in range(round((rows[-1]['t']-rows[0]['t'])*100)+1):
          now=100.+tick*.01
          route_time=rows[0]['t']+tick*.01
          while index+1<len(rows) and rows[index+1]['t']<=route_time: index+=1
          row=rows[index];cs.vEgo=row['v']
          model=N(frameId=row['frame'],position=N(t=row['posT'],x=row['posX'],y=row['posY']),
                  orientation=N(z=row['yaw']),action=N(desiredCurvature=row['modelK']))
          if frame!=row['frame']:
            frame=row['frame']
            writer.decision=dict(received_mono=now,synthetic_indicator_request='none')
            writer.record_model_selection('none','none',True,frame,signal_owner='driver',
                                          manual_turn_direction=1,city_turns_enabled=True)
            writer.publish_diagnostics(now)
          out=reader.update(row['modelK'],cs,True,now,model=model,healthy=True,actual_curvature=0.)
          outputs.append(out)
      self.assertGreater(max(row['modelK'] for row in rows),.004)
      self.assertLessEqual(max(outputs),0.)
      # This scene did not qualify a corner; a veto cannot itself seed a turn.
      self.assertEqual(min(outputs),0.)
