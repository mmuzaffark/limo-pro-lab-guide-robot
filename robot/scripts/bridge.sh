#!/bin/bash
# bridge.sh : websocket bridges for the Windows laptop. Ctrl+C stops them.
#   rosbridge :9090       -> guide GUI (lab_gui_win.py)
#   foxglove_bridge :8765 -> Lichtblick; without it, viz_relay.py makes /scan_viz and /map_viz for Lichtblick over rosbridge
source ~/guide_env.sh
L=$HOME/guide_ws/logs; mkdir -p $L
if pgrep -f rosbridge_websocket > /dev/null; then echo "bridges already running"; exit 0; fi
IP=$(ip -4 -o addr show wlan0 | awk '{print $4}' | cut -d/ -f1)
trap 'kill $(jobs -p) 2>/dev/null' EXIT
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > $L/rosbridge.log 2>&1 &
if ros2 pkg prefix foxglove_bridge > /dev/null 2>&1; then
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > $L/foxglove.log 2>&1 &
  echo "bridges up: GUI ws://$IP:9090, Lichtblick Foxglove WebSocket ws://$IP:8765"
else
  python3 $HOME/guide_ws/scripts/viz_relay.py > $L/viz_relay.log 2>&1 &
  echo "bridges up: GUI ws://$IP:9090, Lichtblick Rosbridge ws://$IP:9090 (topics /scan_viz, /map_viz)"
fi
wait
