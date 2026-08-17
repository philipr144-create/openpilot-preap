#!/usr/bin/env python3

import sys
import os
import shutil
import argparse
from pathlib import Path

MODEL_PATH = Path("/data/openpilot/selfdrive/modeld/modeld.py")
BACKUP_PATH = Path("/data/openpilot/selfdrive/modeld/modeld.py.tesla_nav_backup")

MARKER = "# TESLA_NAVI_DESIRE_V03"

INJECT_CLASS = r'''
# TESLA_NAVI_DESIRE_V03
class TeslaNaviDesire:
  CAN_ID = 0x3C8
  
  # The Tesla distance signal is 5 bits * 10m. Max value is 310m.
  # Values outside of the 10m-300m range are safely ignored.
  MAX_DISTANCE_M = 300.0
  MIN_DISTANCE_M = 10.0

  def __init__(self):
    self.branch_distance = 0.0
    self.left_branch = False
    self.right_branch = False
    self.reject_left = False
    self.reject_right = False

  @staticmethod
  def _get_bit(data, bit):
    byte = bit // 8
    bit_in_byte = bit % 8
    if byte >= len(data):
      return 0
    return (data[byte] >> bit_in_byte) & 1

  @staticmethod
  def _get_bits(data, start_bit, length):
    value = 0
    for i in range(length):
      value |= TeslaNaviDesire._get_bit(data, start_bit + i) << i
    return value

  def update_can(self, can_msg):
    data = bytes(can_msg.dat)
    if len(data) < 5:
      return

    # route_active check removed for pre-AP MCUs.
    raw_dist = self._get_bits(data, 32, 5)
    self.branch_distance = float(raw_dist * 10)
    
    self.left_branch = bool(self._get_bit(data, 38))
    self.right_branch = bool(self._get_bit(data, 39))
    self.reject_left = bool(self._get_bit(data, 40))
    self.reject_right = bool(self._get_bit(data, 41))

  def get_desire(self):
    # A distance of 0 means either no active route, or the maneuver is > 310 meters away.
    # A distance of 310 usually indicates "out of bounds/signal not available".
    if self.branch_distance < self.MIN_DISTANCE_M:
      return log.Desire.none

    if self.branch_distance > self.MAX_DISTANCE_M:
      return log.Desire.none

    if self.reject_left and not self.reject_right:
      return log.Desire.keepRight

    if self.reject_right and not self.reject_left:
      return log.Desire.keepLeft

    if self.left_branch and not self.right_branch:
      return log.Desire.keepLeft

    if self.right_branch and not self.left_branch:
      return log.Desire.keepRight

    return log.Desire.none
'''

def patch_file():
  if not MODEL_PATH.exists():
    print(f"ERROR: {MODEL_PATH} does not exist.")
    sys.exit(1)

  text = MODEL_PATH.read_text()

  if MARKER in text:
    print("Tesla Navi Desire v0.3 is already installed in modeld.py!")
    return

  # If a backup doesn't exist, create it. If it does, we assume it's the clean stock backup.
  if not BACKUP_PATH.exists():
    shutil.copy2(MODEL_PATH, BACKUP_PATH)
    print(f"Backup created: {BACKUP_PATH}")
  else:
    # We are upgrading. Restore stock first so we don't inject twice.
    shutil.copy2(BACKUP_PATH, MODEL_PATH)
    text = MODEL_PATH.read_text()

  # 1. Add class immediately before FrameMeta.
  anchor_class = "class FrameMeta:\n"
  if anchor_class not in text:
    print("ERROR: Could not find FrameMeta anchor.")
    sys.exit(1)
  text = text.replace(anchor_class, INJECT_CLASS + "\n\n" + anchor_class, 1)

  # 2. Add Tesla CAN subscription.
  old_sub = '''sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay"])'''
  new_sub = '''sm = SubMaster(["deviceState", "carState", "roadCameraState", "liveCalibration", "driverMonitoringState", "carControl", "liveDelay", "can"])'''
  if old_sub in text:
    text = text.replace(old_sub, new_sub, 1)

  # 3. Instantiate parser and read param securely
  old_init = "DH = DesireHelper()\n"
  new_init = '''DH = DesireHelper()

  # TESLA_NAVI_DESIRE_V03
  tesla_nav = TeslaNaviDesire()
  
  # Read param directly from file to avoid UnknownKeyName crash
  tesla_nav_enabled = False
  try:
    with open("/data/params/d/TeslaNavDesireEnabled", "r") as f:
      tesla_nav_enabled = (f.read().strip() == "1")
  except Exception:
    pass
'''
  if old_init in text:
    text = text.replace(old_init, new_init, 1)

  # 4. Parse CAN immediately after sm.update.
  old_loop = "sm.update(0)\n    desire = DH.desire\n"
  new_loop = '''sm.update(0)

    # TESLA_NAVI_DESIRE_V03
    if tesla_nav_enabled and sm.updated["can"]:
      for can_msg in sm["can"]:
        if can_msg.address == tesla_nav.CAN_ID:
          tesla_nav.update_can(can_msg)
          break

    desire = DH.desire

    if tesla_nav_enabled:
      tesla_desire = tesla_nav.get_desire()
      if tesla_desire != log.Desire.none and desire == log.Desire.none:
        desire = tesla_desire
'''
  if old_loop in text:
    text = text.replace(old_loop, new_loop, 1)

  MODEL_PATH.write_text(text)
  print("\n=== SUCCESS ===")
  print("Tesla Navi Desire v0.3 successfully injected into modeld.py!")
  print("\nRun this script with --enable to turn it on, then restart openpilot.\n")

def restore_file():
  if not BACKUP_PATH.exists():
    print(f"ERROR: Backup file {BACKUP_PATH} not found.")
    sys.exit(1)
  shutil.copy2(BACKUP_PATH, MODEL_PATH)
  print("Restored modeld.py from stock backup.")

def toggle_param(enable: bool):
  val = "1" if enable else "0"
  try:
    os.makedirs("/data/params/d", exist_ok=True)
    with open("/data/params/d/TeslaNavDesireEnabled", "w") as f:
      f.write(val)
      
    state = "ENABLED" if enable else "DISABLED"
    print(f"Success! TeslaNavDesireEnabled is now {state}.")
    print("Please restart Openpilot to apply the change.")
  except Exception as e:
    print(f"Error writing to params: {e}")

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="Tesla MCU2 Navi Desire Bridge Installer")
  parser.add_argument('--install', action='store_true', help="Patch modeld.py with the bridge")
  parser.add_argument('--uninstall', action='store_true', help="Restore modeld.py from backup")
  parser.add_argument('--enable', action='store_true', help="Enable the feature in openpilot params")
  parser.add_argument('--disable', action='store_true', help="Disable the feature in openpilot params")
  
  args = parser.parse_args()

  if not any(vars(args).values()):
    patch_file()
  else:
    if args.uninstall:
      restore_file()
    elif args.install:
      patch_file()
      
    if args.enable:
      toggle_param(True)
    elif args.disable:
      toggle_param(False)
