#!/usr/bin/env python3
import sys
import time
import os
import cereal.messaging as messaging

def is_feature_enabled():
    try:
        with open("/data/params/d/TeslaNavDesireEnabled", "r") as f:
            return f.read().strip() == "1"
    except FileNotFoundError:
        return False

def monitor_nav_can():
    print("========================================")
    print(" Tesla Navi Desire - Live CAN Monitor")
    print("========================================")

    if not is_feature_enabled():
        print("⚠️  WARNING: TeslaNavDesireEnabled is currently OFF in /data/params/d/.")
        print("   The script will still monitor CAN, but modeld won't act on it.")
    else:
        print("✅ Feature is ENABLED in params.")

    print("\nListening for MCU2 Map Data (CAN ID: 0x3C8)...")
    print("Set a destination on your MCU2 screen. (Press Ctrl+C to quit)\n")

    sm = messaging.SubMaster(['can'])
    
    try:
        while True:
            sm.update(100)
            if sm.updated['can']:
                for msg in sm['can']:
                    if msg.address == 0x3C8:  # Decimal 968
                        data = bytes(msg.dat)
                        if len(data) < 6:
                            continue
                        
                        route_active = bool((data[3] >> 5) & 1)
                        dist_m = (data[4] & 0x1F) * 10
                        left_branch = bool((data[4] >> 6) & 1)
                        right_branch = bool((data[4] >> 7) & 1)
                        reject_left = bool((data[5] >> 0) & 1)
                        reject_right = bool((data[5] >> 1) & 1)

                        status = f"Active: {route_active} | Dist: {dist_m:>3}m | LeftFork: {left_branch} | RightFork: {right_branch} | RejectL: {reject_left} | RejectR: {reject_right}"
                        
                        sys.stdout.write(f"\r\033[K{status}")
                        sys.stdout.flush()
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n\nExiting monitor.")

if __name__ == "__main__":
    monitor_nav_can()
