#!/usr/bin/env python3
# loc_check.py [--fix] [--min 60] : is the robot localized? Share of lidar points that land on a map wall.
# --fix: if the score is low, turn once in place (AMCL updates while turning) and check again.
# Exit code 0 = localized, 1 = not localized. Needs nav_start.sh (map_server + AMCL) running.
import argparse, math, sys, time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Check:
    def __init__(s):
        s.n = rclpy.create_node('loc_check'); s.map = None; s.scan = None
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        s.n.create_subscription(OccupancyGrid, '/map', lambda m: setattr(s, 'map', m), q)
        s.n.create_subscription(LaserScan, '/scan', lambda m: setattr(s, 'scan', m), qos_profile_sensor_data)
        s.buf = Buffer(); s.tl = TransformListener(s.buf, s.n)
        s.cmd = s.n.create_publisher(Twist, '/cmd_vel', 10)

    def tf(s, parent, child):
        try:
            t = s.buf.lookup_transform(parent, child, Time()).transform
            return t.translation.x, t.translation.y, yaw_of(t.rotation)
        except Exception:
            return None

    def score(s, scans=5):
        m = s.map; r = m.info.resolution; w, h = m.info.width, m.info.height
        ox, oy = m.info.origin.position.x, m.info.origin.position.y; hit = tot = got = 0
        while got < scans:
            s.scan = None
            while s.scan is None: rclpy.spin_once(s.n, timeout_sec=0.1)
            L = s.tf('map', 'laser_link')
            if L is None: continue
            got += 1; sc = s.scan
            for k, rng in enumerate(sc.ranges):
                if not (sc.range_min < rng < min(8.0, sc.range_max * 0.99)): continue
                a = L[2] + sc.angle_min + k * sc.angle_increment
                i = int((L[0] + rng * math.cos(a) - ox) / r); j = int((L[1] + rng * math.sin(a) - oy) / r)
                tot += 1
                if any(0 <= i + di < w and 0 <= j + dj < h and m.data[(j + dj) * w + i + di] >= 65
                       for di in (-2, -1, 0, 1, 2) for dj in (-2, -1, 0, 1, 2)):
                    hit += 1
        return 100.0 * hit / tot if tot else 0.0

    def turn(s, speed=0.3):
        tw = Twist(); tw.angular.z = speed; acc = 0.0; prev = None; t0 = time.time()
        while acc < 2 * math.pi and time.time() - t0 < 2 * 2 * math.pi / speed:
            s.cmd.publish(tw); rclpy.spin_once(s.n, timeout_sec=0.05)
            p = s.tf('odom', 'base_link')
            if p is not None:
                if prev is not None: acc += abs(math.atan2(math.sin(p[2] - prev), math.cos(p[2] - prev)))
                prev = p[2]
        for _ in range(5): s.cmd.publish(Twist()); time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--fix', action='store_true'); ap.add_argument('--min', type=float, default=60.0)
    ap.add_argument('--wait', type=float, default=60.0, help='seconds to wait for map, scan and TF')
    a = ap.parse_args()
    rclpy.init(); c = Check(); t0 = time.time()
    while time.time() - t0 < a.wait and (c.map is None or c.scan is None or c.tf('map', 'laser_link') is None):
        rclpy.spin_once(c.n, timeout_sec=0.2)
    if c.map is None or c.scan is None or c.tf('map', 'laser_link') is None:
        print('LOC CHECK: no map/scan/TF after %.0f s: is nav_start.sh running?' % a.wait); return 1
    sc = c.score(); p = c.tf('map', 'base_link')
    print('LOC CHECK: %.0f %% of lidar points on map walls (need %.0f %%), robot at (%.2f, %.2f) %.0f deg'
          % (sc, a.min, p[0], p[1], math.degrees(p[2])), flush=True)
    if sc < a.min and a.fix:
        print('LOC CHECK: turning once in place to let AMCL converge ...', flush=True)
        c.turn(); time.sleep(2.0)
        for _ in range(20): rclpy.spin_once(c.n, timeout_sec=0.1)
        sc = c.score(); p = c.tf('map', 'base_link')
        print('LOC CHECK: after the turn %.0f %%, robot at (%.2f, %.2f) %.0f deg'
              % (sc, p[0], p[1], math.degrees(p[2])), flush=True)
    print('LOC CHECK: %s' % ('OK' if sc >= a.min else 'NOT LOCALIZED: put the robot on the home mark and restart, '
                                                          'or set 2D Pose Estimate in RViz2'))
    return 0 if sc >= a.min else 1


if __name__ == '__main__':
    sys.exit(main())
