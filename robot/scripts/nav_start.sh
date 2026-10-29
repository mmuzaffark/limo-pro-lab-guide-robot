#!/bin/bash
# base + lidar + AMCL on lab_v1 + Nav2, logs in ~/guide_ws/logs; Ctrl+C stops all
source ~/guide_env.sh
P=$HOME/guide_ws/params/nav2_lab_foxy.yaml; M=$HOME/guide_ws/maps/lab_v1.yaml; L=$HOME/guide_ws/logs
trap 'kill 0' EXIT
ros2 launch $HOME/guide_ws/launch/limo_start_guide.launch.py > $L/limo.log 2>&1 &
sleep 5
ros2 launch nav2_bringup localization_launch.py use_sim_time:=False map:=$M params_file:=$P > $L/loc.log 2>&1 &
sleep 5
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False map_subscribe_transient_local:=true params_file:=$P > $L/nav.log 2>&1 &
sleep 12
bash ~/guide_ws/scripts/set_home_pose.sh
echo "NAV READY (logs: $L)"; wait
