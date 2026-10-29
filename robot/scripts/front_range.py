#!/usr/bin/env python3
# front_range.py : mean range of beams within +-3 deg of straight ahead, over 30 scans
import math, statistics, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('front_range'); vals = []
def cb(m):
    r = [x for i, x in enumerate(m.ranges)
         if abs(m.angle_min + i * m.angle_increment) <= math.radians(3) and 0.02 < x < 10]
    if r: vals.append(statistics.median(r))
n.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)
while len(vals) < 30: rclpy.spin_once(n, timeout_sec=1.0)
print('front range: mean %.3f m  std %.3f m  (30 scans)' % (statistics.mean(vals), statistics.pstdev(vals)))
