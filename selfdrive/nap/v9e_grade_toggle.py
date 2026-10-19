#!/usr/bin/env python3
"""Toggle V9e predictive grade assistance OFF/ON while parked, or inspect status.

The setting is independent of the legacy NAPPitchAssistMode pitch sensor gate.
No restart is initiated. The controlsd reader polls the setting twice/second.
"""
import argparse
import json
import os
from pathlib import Path
import re
import subprocess

PATH = Path('/data/nap_settings/v9e_predictive_grade.json')


def active_control_processes():
  for line in subprocess.check_output(['ps','-eo','pid=,args='],text=True).splitlines():
    if re.search(r'(?<![\w])(controlsd|modeld|plannerd)(?:\.py)?(?:\s|$)', line):
      yield line.strip()


def main():
  parser = argparse.ArgumentParser(description=__doc__)
  options=parser.add_mutually_exclusive_group()
  options.add_argument('--enable', action='store_true')
  options.add_argument('--disable', action='store_true')
  options.add_argument('--status', action='store_true')
  args=parser.parse_args()
  if args.enable or args.disable:
    if args.enable:
      present = list(active_control_processes())
      if present:
        raise SystemExit('Refusing to enable while driving processes may be running. Park and verify offroad.\n'+'\n'.join(present))
    if not PATH.is_file() or PATH.is_symlink():
      raise SystemExit('V9e is not installed or toggle missing; refusing to create unexpected setting')
    temp=PATH.with_name(PATH.name+'.tmp.'+str(os.getpid()))
    try:
      temp.write_text(json.dumps({'enabled':bool(args.enable)})+'\n')
      os.replace(temp,PATH)
    finally:
      temp.unlink(missing_ok=True)
  try:
    cfg=json.loads(PATH.read_text())
    enabled=isinstance(cfg,dict) and cfg.get('enabled') is True
    print('Predictive grade assistance:', 'ON' if enabled else 'OFF')
    print('Config:', PATH)
    print('Max +0.08 m/s², sub-55 mph, 50–55 mph fade, lead-MPC bounded.')
  except (FileNotFoundError,ValueError,OSError):
    print('Predictive grade assistance: OFF (not installed or malformed)')


if __name__ == '__main__':
  main()
