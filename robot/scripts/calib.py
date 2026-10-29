#!/usr/bin/env python3
# calib.py straight [DIST_m] | ccw | cw
# Odometry check with a flat wall as ground truth (no tape). Lidar fits the wall line
# x = a*y + b over beams within +-25 deg: heading to wall psi = atan(a), distance d = b*cos(psi).
import sys, math, time, statistics, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Calib:
    def __init__(s):
        s.n = rclpy.create_node('calib')
        s.scan = None; s.pose = None; s.prev = None; s.yaw = 0.0; s.imu = 0.0; s.ti = None
        s.n.create_subscription(LaserScan, '/scan', s.on_scan, qos_profile_sensor_data)
        s.n.create_subscription(Odometry, '/odom', s.on_odom, 10)
        s.n.create_subscription(Imu, '/imu', s.on_imu, qos_profile_sensor_data)
        s.pub = s.n.create_publisher(Twist, '/cmd_vel', 10)

    def on_scan(s, m): s.scan = m

    def on_odom(s, m):
        p = m.pose.pose.position; q = m.pose.pose.orientation
        y = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if s.prev is not None:
            s.yaw += math.atan2(math.sin(y - s.prev), math.cos(y - s.prev))
        s.prev = y; s.pose = (p.x, p.y)

    def on_imu(s, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if s.ti is not None and 0 < t - s.ti < 0.5:
            s.imu += m.angular_velocity.z * (t - s.ti)
        s.ti = t

    def wait(s, sec):
        t = time.time()
        while time.time() - t < sec: rclpy.spin_once(s.n, timeout_sec=0.02)

    def points(s, deg=25, rmax=4.0):
        m = s.scan; out = []
        for i, r in enumerate(m.ranges):
            a = m.angle_min + i * m.angle_increment
            if abs(a) <= math.radians(deg) and 0.1 < r < rmax:
                out.append((r * math.cos(a), r * math.sin(a)))
        return out

    def wall(s, scans=20):
        ds, ps = [], []
        while len(ds) < scans:
            s.scan = None
            while s.scan is None: rclpy.spin_once(s.n, timeout_sec=0.1)
            P = s.points()
            if len(P) < 15: continue
            mx = statistics.mean(p[0] for p in P); my = statistics.mean(p[1] for p in P)
            syy = sum((p[1] - my) ** 2 for p in P)
            sxy = sum((p[0] - mx) * (p[1] - my) for p in P)
            a = sxy / syy; b = mx - a * my; psi = math.atan(a)
            ds.append(b * math.cos(psi)); ps.append(math.degrees(psi))
        return statistics.mean(ds), statistics.mean(ps), statistics.pstdev(ds)

def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'straight'
    rclpy.init(); c = Calib(); c.wait(1.5)
    if c.pose is None or c.scan is None:
        print('no /odom or /scan: is limo_start_guide running, ROS_DOMAIN_ID 30?'); return
    d0, p0, sd0 = c.wall()
    print('start: wall %.3f m (std %.3f), heading to wall %+.2f deg' % (d0, sd0, p0))
    x0 = c.pose; y0 = c.yaw; i0 = c.imu; tw = Twist()
    try:
        if mode == 'straight':
            D = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0
            if d0 - D < 0.5:
                print('too close: start at least %.1f m from the wall' % (D + 0.6)); return
            while True:
                rclpy.spin_once(c.n, timeout_sec=0.02)
                od = math.hypot(c.pose[0] - x0[0], c.pose[1] - x0[1])
                front = [p[0] for p in c.points(5, 10.0) if abs(p[1]) < 0.15]
                if od >= D: break
                if front and min(front) < 0.45: print('safety stop: wall < 0.45 m'); break
                tw.linear.x = 0.15 if D - od > 0.15 else 0.06; c.pub.publish(tw)
        else:
            sg = 1.0 if mode == 'ccw' else -1.0
            while True:
                rclpy.spin_once(c.n, timeout_sec=0.02)
                done = abs(math.degrees(c.yaw - y0))
                if done >= 360.0: break
                tw.angular.z = sg * (0.5 if 360.0 - done > 30.0 else 0.2); c.pub.publish(tw)
    finally:
        for _ in range(5): c.pub.publish(Twist()); time.sleep(0.02)
    c.wait(1.5)
    d1, p1, sd1 = c.wall()
    print('end:   wall %.3f m (std %.3f), heading to wall %+.2f deg' % (d1, sd1, p1))
    if mode == 'straight':
        od = math.hypot(c.pose[0] - x0[0], c.pose[1] - x0[1]); real = d0 - d1
        print('RESULT straight: real %.3f m  odom %.3f m  error %+.1f %%  heading change %+.1f deg'
              % (real, od, (od - real) / real * 100, p1 - p0))
    else:
        N = 360.0 if mode == 'ccw' else -360.0
        real = N + (p1 - p0)
        oy = math.degrees(c.yaw - y0); iy = math.degrees(c.imu - i0)
        print('RESULT %s: real %+.1f deg  odom %+.1f deg (%+.1f %%)  imu %+.1f deg (%+.1f %%)'
              % (mode, real, oy, (oy - real) / abs(real) * 100, iy, (iy - real) / abs(real) * 100))

if __name__ == '__main__':
    main()
