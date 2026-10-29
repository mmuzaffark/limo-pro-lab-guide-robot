#!/usr/bin/env python3
# viz_relay.py : for Lichtblick over rosbridge (Foxy) when foxglove_bridge is not installed.
# /scan (best effort) -> /scan_viz (reliable); /map (latched) -> /map_viz every 2 s.
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node('viz_relay'); last = {}
ps = n.create_publisher(LaserScan, '/scan_viz', 10)
pm = n.create_publisher(OccupancyGrid, '/map_viz', 1)
n.create_subscription(LaserScan, '/scan', lambda m: ps.publish(m), qos_profile_sensor_data)
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid, '/map', lambda m: last.__setitem__('map', m), q)
n.create_timer(2.0, lambda: pm.publish(last['map']) if 'map' in last else None)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
