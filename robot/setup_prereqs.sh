#!/bin/bash
# setup_prereqs.sh : one-time software setup of an AgileX Limo Pro (Jetson, Ubuntu 20.04, ROS 2 Foxy factory image)
# for the lab guide. Run on the robot from a clone of this repo, with internet:
#   git clone https://github.com/mmuzaffark/limo-pro-lab-guide-robot.git ~/limo-pro-lab-guide-robot
#   bash ~/limo-pro-lab-guide-robot/robot/setup_prereqs.sh
# Does (each step is skipped when already done): checks the factory image (Foxy + limo_ros2_ws with Nav2 0.4.x),
# apt packages, tmux setup, ~/guide_env.sh, explore_lite (m-explore-ros2 @ e40e857) in ~/guide_ws,
# the Nav2 RPP fix (patches/fix_rpp_goal_dist_tol.sh), then copies the demo map. Asks the sudo password (Limo: AGX).
# Never runs apt upgrade (the factory Nav2 source build depends on the installed libraries).
set -e
REPO=$(cd "$(dirname "$0")/.." && pwd)
ok() { echo "  OK   $*"; }
echo "== 1 factory image"
[ -f /opt/ros/foxy/setup.bash ] || { echo "  FAIL /opt/ros/foxy missing: this needs the Limo Pro ROS 2 Foxy image"; exit 1; }
[ -f ~/limo_ros2_ws/install/setup.bash ] || { echo "  FAIL ~/limo_ros2_ws/install missing: this needs the AgileX limo_ros2 workspace (factory image)"; exit 1; }
set +e; source /opt/ros/foxy/setup.bash; source ~/limo_ros2_ws/install/setup.bash; set -e
V=$(grep -m1 -o "<version>[^<]*" "$(ros2 pkg prefix nav2_bringup 2>/dev/null)/share/nav2_bringup/package.xml" 2>/dev/null | cut -d'>' -f2)
case "$V" in 0.4.*) ok "Nav2 $V (Foxy)";; *) echo "  FAIL nav2_bringup version '${V:-missing}', need 0.4.x from ~/limo_ros2_ws"; exit 1;; esac
ros2 pkg prefix nav2_regulated_pure_pursuit_controller > /dev/null 2>&1 && ok "RPP controller present" || { echo "  FAIL nav2_regulated_pure_pursuit_controller missing"; exit 1; }

echo "== 2 apt packages (no upgrade)"
sudo timedatectl set-ntp true || true
sudo apt update 2>&1 | tail -2
PK="tmux git ros-foxy-slam-toolbox ros-foxy-teleop-twist-keyboard ros-foxy-rmw-cyclonedds-cpp ros-foxy-robot-localization ros-foxy-laser-filters ros-foxy-rosbridge-server"
sudo apt install -y $PK 2>&1 | tail -2
python3 -c "import yaml" && ok "python3-yaml"

echo "== 3 tmux (terminals that survive Wi-Fi drops)"
[ -f ~/.guide_bashrc ] || printf 'source ~/.bashrc\n[ -f ~/guide_env.sh ] && source ~/guide_env.sh\n' > ~/.guide_bashrc
[ -f ~/.tmux.conf ] || printf 'set -g mouse off\nset -g history-limit 20000\nset -g default-command "bash --rcfile ~/.guide_bashrc"\n' > ~/.tmux.conf
ok "~/.tmux.conf, ~/.guide_bashrc"

echo "== 4 ~/guide_env.sh"
mkdir -p ~/guide_ws/{src,params,maps,scripts,logs,bags,launch}
[ -f ~/guide_env.sh ] || cp "$REPO/robot/guide_env.sh" ~/guide_env.sh
ok "$(grep -c . ~/guide_env.sh) lines (ROS_DOMAIN_ID 30, Cyclone DDS)"

echo "== 5 explore_lite (m-explore-ros2 @ e40e857: newer commits need Humble)"
if [ ! -d ~/guide_ws/src/m-explore-ros2 ]; then
  git clone -q https://github.com/robo-friends/m-explore-ros2.git ~/guide_ws/src/m-explore-ros2
fi
git -C ~/guide_ws/src/m-explore-ros2 checkout -q e40e857
if [ ! -d ~/guide_ws/install/explore_lite ]; then
  (cd ~/guide_ws && colcon build --symlink-install --packages-select explore_lite 2>&1 | tail -3)
fi
[ -d ~/guide_ws/install/explore_lite ] && ok "explore_lite built" || { echo "  FAIL explore_lite build"; exit 1; }

echo "== 6 Nav2 RPP fix"
bash "$REPO/robot/patches/fix_rpp_goal_dist_tol.sh"

echo "== 7 demo map"
for f in lab_v1.yaml lab_v1.pgm; do [ -f ~/guide_ws/maps/$f ] || cp "$REPO/robot/maps/$f" ~/guide_ws/maps/; done
ok "maps/lab_v1 (the original lab; make your own map for a new room: README)"
echo
echo "PREREQS OK. Next: bash $REPO/install_guide.sh   then   ~/guide_ws/scripts/make_cyclone.sh LAPTOP_IP"
