import rclpy, math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
class S(Node):
    def __init__(s):
        super().__init__('scan_check'); s.n = 0
        s.create_subscription(LaserScan, '/scan', s.cb, qos_profile_sensor_data)
    def cb(s, m):
        s.n += 1
        if s.n < 5: return
        near = [(round(math.degrees(m.angle_min + i*m.angle_increment)), round(r, 2))
                for i, r in enumerate(m.ranges) if m.range_min < r < 0.35]
        print('range_min %.2f  angle %.0f..%.0f deg  points %d  near(<0.35 m) %d'
              % (m.range_min, math.degrees(m.angle_min), math.degrees(m.angle_max), len(m.ranges), len(near)))
        print(near[:60]); rclpy.shutdown()
rclpy.init(); n = S()
try: rclpy.spin(n)
except Exception: pass
