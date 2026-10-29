#!/usr/bin/env python3
# once.py TOPIC : print one message (best-effort QoS, works with any publisher)
import sys, time, rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message
from rosidl_runtime_py import message_to_yaml
topic = sys.argv[1]
rclpy.init(); n = rclpy.create_node('once'); typ = None; t0 = time.time()
while typ is None and time.time() - t0 < 10:
    rclpy.spin_once(n, timeout_sec=0.5)
    for name, types in n.get_topic_names_and_types():
        if name == topic: typ = types[0]
if typ is None: print('topic not found:', topic); sys.exit(1)
done = []
def cb(m):
    if hasattr(m, 'ranges'):
        print({k: round(getattr(m, k), 4) for k in
               ['angle_min', 'angle_max', 'angle_increment', 'range_min', 'range_max']},
              'points:', len(m.ranges))
    else:
        print(message_to_yaml(m))
    done.append(1)
n.create_subscription(get_message(typ), topic, cb, qos_profile_sensor_data)
t0 = time.time()
while not done and time.time() - t0 < 10: rclpy.spin_once(n, timeout_sec=0.5)
if not done: print('no message in 10 s on', topic, typ)
