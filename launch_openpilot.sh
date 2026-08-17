#!/usr/bin/env bash

python3 /data/openpilot/mpp_writer.py &
exec ./launch_chffrplus.sh
/data/ngrok http 7070 --url https://statistic-vividly-consent.ngrok-free.dev > /tmp/ngrok.log 2>&1 &
