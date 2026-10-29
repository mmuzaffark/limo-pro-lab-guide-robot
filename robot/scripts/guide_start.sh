#!/bin/bash
# base + lidar + AMCL + Nav2 + laptop bridges (4.6), localization check, then the guide manager. Robot on the home mark.
# Ctrl+C stops everything. The limo-guide service (9.5) runs this same script at power-on.
source ~/guide_env.sh
trap 'kill 0' EXIT
~/guide_ws/scripts/nav_start.sh &
~/guide_ws/scripts/bridge.sh > ~/guide_ws/logs/bridge_start.log 2>&1 &
sleep 25
python3 ~/guide_ws/scripts/loc_check.py --fix || echo "WARNING: localization doubtful: check Lichtblick before guiding"
python3 ~/guide_ws/scripts/guide_manager_foxy.py ~/guide_ws/places_lab.yaml
