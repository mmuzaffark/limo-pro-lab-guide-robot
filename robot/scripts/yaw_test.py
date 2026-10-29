import rclpy, rclpy.time, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def wrap(a): return math.atan2(math.sin(a), math.cos(a))
class T(Node):
    def __init__(s):
        super().__init__('yaw_test'); s.d = {}; s.gyro = 0.0; s.tg = None; s.absacc = 0.0; s.la = None
        s.buf = Buffer(); s.tl = TransformListener(s.buf, s)
        s.create_subscription(Odometry, '/odom', lambda m: s.upd('wheel', yaw(m.pose.pose.orientation)), 20)
        s.create_subscription(Odometry, '/odometry/filtered', lambda m: s.upd('ekf', yaw(m.pose.pose.orientation)), 20)
        s.create_subscription(Imu, '/imu', s.imu, qos_profile_sensor_data)
        s.create_timer(0.1, s.tf); s.create_timer(1.0, s.show)
    def upd(s, k, y):
        if k not in s.d: s.d[k] = [y, 0.0]; return
        last, tot = s.d[k]; s.d[k] = [y, tot + wrap(y - last)]
    def imu(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec*1e-9
        if s.tg is not None and 0 < t - s.tg < 0.5: s.gyro += m.angular_velocity.z*(t - s.tg)
        s.tg = t
    def tf(s):
        try: t = s.buf.lookup_transform('odom', 'base_link', rclpy.time.Time())
        except Exception: return
        y = yaw(t.transform.rotation); s.upd('tf', y)
        if s.la is not None: s.absacc += abs(wrap(y - s.la))
        s.la = y
    def show(s):
        o = ['%s %7.1f' % (k, math.degrees(v[1])) for k, v in sorted(s.d.items())]
        print('  '.join(o + ['gyro %7.1f' % math.degrees(s.gyro), 'tf_abs %7.1f' % math.degrees(s.absacc)]), flush=True)
rclpy.init(); n = T()
try: rclpy.spin(n)
except KeyboardInterrupt: pass
