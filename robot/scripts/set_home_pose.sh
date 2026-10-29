#!/bin/bash
source ~/guide_env.sh
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, covariance: [0.04,0.0,0.0,0.0,0.0,0.0, 0.0,0.04,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.03]}}"
