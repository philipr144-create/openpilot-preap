#!/usr/bin/env bash

python3 /data/openpilot/mpp_writer.py &
exec ./launch_chffrplus.sh
