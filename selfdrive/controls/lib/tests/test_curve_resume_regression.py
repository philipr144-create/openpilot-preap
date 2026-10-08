"""Offline curve timing, brake-resume, and final regen regression checks."""
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace as N
from unittest.mock import MagicMock, patch
import pytest
from openpilot.selfdrive.controls.lib.longitudinal_planner import map_curve_speed_cap
from opendbc.car.tesla.preap.engagement import PreAPEngagement
from opendbc.car.tesla.preap import carcontroller as cc


def snapshot(start=100., end=200.):
  return dict(version=3,enabled=True,valid=True,control_valid=True,status='ok',mono_time=10.,expires_mono=12.,
              match=dict(distance_m=1.,heading_error_deg=1.,confidence=1.),lookahead_m=450.,
              control_curves=[dict(start_distance_m=start,end_distance_m=end,length_m=end-start,
                                  target_speed_mps=18.,curvature=3./18.**2)])


def test_curve_distance_moves_between_map_updates_and_refresh_is_continuous():
  with TemporaryDirectory() as tmp:
    p=Path(tmp)/'curve.json';state=snapshot();p.write_text(json.dumps(state))
    values=[map_curve_speed_cap(p,now=10.+i*.05,v_ego=25.) for i in range(11)]
    for i,v in enumerate(values):
      assert v==pytest.approx(math.sqrt(18.**2+2*1.2*(94.-25*i*.05)))
    assert all(b<a for a,b in zip(values,values[1:]))
    state['mono_time']=10.5;state['expires_mono']=12.5
    state['control_curves'][0]['start_distance_m']=87.5
    state['control_curves'][0]['end_distance_m']=187.5
    p.write_text(json.dumps(state))
    assert map_curve_speed_cap(p,now=10.5,v_ego=25.)==pytest.approx(values[-1])
    assert map_curve_speed_cap(p,now=12.5,v_ego=25.) is None


def test_active_curve_absolute_target_and_exit_do_not_ratchet():
  with TemporaryDirectory() as tmp:
    p=Path(tmp)/'curve.json';p.write_text(json.dumps(snapshot(-20.,5.)))
    for speed in (10.,15.,20.,25.):
      assert map_curve_speed_cap(p,now=10.,v_ego=speed)==18.
    assert map_curve_speed_cap(p,now=10.3,v_ego=20.) is None


@pytest.mark.parametrize('duration',[60,300,900,1800,2500])
def test_brake_resume_accepts_press_release_restores_speed(duration):
  e=PreAPEngagement(False,750)
  e.process_buttons(2,0,1000,5.,'KPH',True,True,True,False)
  target=e.pedal_speed_kph
  e.process_buttons(0,0,2000,0.,'KPH',True,True,True,True)
  def step(t,gas=False,speed=0.):
    return e.process_brake_resume(t,speed,False,gas,True,True,True,True)
  assert not step(2100)
  assert step(2200,True)
  assert e.enableLongControl
  assert not step(2200+duration,speed=5.)
  assert e.enableLongControl and e.pedal_speed_kph==target and e.resume_sequence==1
  assert not step(2300+duration)
  assert e.resume_sequence==1


def test_throttle_release_cannot_engage_lateral_only_without_brake_pause():
  e=PreAPEngagement(False,750);e.cruiseEnabled=True
  assert not e.process_brake_resume(1000,0.,False,True,True,True,True,True)
  assert not e.process_brake_resume(1200,0.,False,False,True,True,True,True)
  assert not e.enableLongControl


def test_full_regen_survives_actual_engage_grace_and_negative_jerk_limiter():
  with patch.object(cc,'nap_conf') as conf, patch.object(cc,'get_zero_torque') as zero, \
       patch('opendbc.car.tesla.preap.virtual_das.get_zero_torque') as vz, \
       patch.object(cc,'get_preap_personality',return_value=2):
    conf.use_pedal=True;conf.pedal_factor=1.;conf.di_to_pedal.side_effect=lambda x:x
    zero.return_value.get.return_value=0.;vz.return_value.get.return_value=0.
    controller=cc.PreAPLongController()
    cs=N(cruiseEnabled=True,enableLongControl=True,pedal_interceptor_value=0.,pedal_timeout=False,
         cruise_buttons=0,prev_cruise_buttons=0,pedal=N(torque_level=0.),out=N(vEgo=5.,aEgo=0.,gasPressed=False))
    command=N(longActive=True,actuators=N(accel=-2.5),orientationNED=[])
    can=MagicMock()
    previous=0.
    for frame in range(0,152,2):
      controller.update(command,cs,frame,can,0)
      value=controller.vdas.jerk_limiter.a_limited
      assert value<=previous
      assert previous-value<=2.5*.02+1e-9
      if frame==0: assert value==pytest.approx(-.05)
      previous=value
    assert previous==pytest.approx(-2.5)
