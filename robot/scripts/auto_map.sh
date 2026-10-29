#!/bin/bash
# One-command autonomous mapping: base + lidar, SLAM, Nav2, then auto_map.py.
# Robot on the home mark, doors closed. Ctrl+C stops everything. Options go to auto_map.py (e.g. --max-min 3).
source ~/guide_env.sh
L=$HOME/guide_ws/logs; T=$(date +%Y%m%d_%H%M); mkdir -p $L
if pgrep -f "limo_base|slam_toolbox|bt_navigator" > /dev/null; then
  echo "Robot programs are already running (another tmux window?). Stop them first."; exit 1
fi
trap 'pkill -INT -f "lib/explore_lite/explore"; kill 0' EXIT
echo "[$T] starting base + lidar, SLAM, Nav2 (logs: $L/am_${T}_*.log)"
ros2 launch $HOME/guide_ws/launch/limo_start_guide.launch.py > $L/am_${T}_limo.log 2>&1 &
sleep 6
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $HOME/guide_ws/params/slam_lab.yaml > $L/am_${T}_slam.log 2>&1 &
sleep 4
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=$HOME/guide_ws/params/nav2_lab_foxy.yaml > $L/am_${T}_nav.log 2>&1 &
python3 $HOME/guide_ws/scripts/auto_map.py --tag $T "$@"
