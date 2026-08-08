#!/usr/bin/env python3
import cereal.messaging as messaging
import os

def main():
    print("Starting raw CAN MPP writer...")
    sm = messaging.SubMaster(["can"])
    
    while True:
        sm.update(100)
        
        if sm.updated["can"]:
            for msg in sm["can"]:
                if msg.address == 760:
                    d = msg.dat
                    if len(d) == 8:
                        # Raw CAN math from your probe (calculates MPH)
                        mpp_mph = (d[6] & 0x1F) * 5
                        
                        if mpp_mph > 0 and mpp_mph < 150:
                            # cruise.py expects meters per second
                            mpp_mps = mpp_mph / 2.236936
                            with open("/dev/shm/mpp_speed_limit", "w") as f:
                                f.write(str(mpp_mps))
                        else:
                            with open("/dev/shm/mpp_speed_limit", "w") as f:
                                f.write("0.0")

if __name__ == "__main__":
    main()
