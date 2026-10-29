#!/bin/bash
# usage: goto.sh PLACE_NAME
source ~/guide_env.sh
read X Y W Z < <(python3 -c "import yaml,math,sys; p=yaml.safe_load(open('$HOME/guide_ws/places_lab.yaml'))['places']['$1']; print(p[0],p[1],math.cos(p[2]/2),math.sin(p[2]/2))") || exit 1
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $Z, w: $W}}}}"
