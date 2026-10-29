source /opt/ros/foxy/setup.bash
source ~/limo_ros2_ws/install/setup.bash
[ -f ~/guide_ws/install/setup.bash ] && source ~/guide_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
export CYCLONEDDS_URI=file://$HOME/guide_ws/cyclone.xml
