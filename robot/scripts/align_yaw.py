#!/usr/bin/env python3
# align_yaw.py [YAW_DEG] [--tol 2] : turn in place until the map heading equals YAW_DEG (default 0 = home heading).
# Needs nav_start.sh (AMCL) running and no active Nav2 goal. Exit 0 = aligned.
import argparse, math, sys, time
import rclpy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('yaw', nargs='?', type=float, default=0.0)
    ap.add_argument('--tol', type=float, default=5.0, help='deg')
    ap.add_argument('--timeout', type=float, default=20.0)
    ap.add_argument('--min-w', type=float, default=0.20, help='rad/s, lowest speed that still turns on carpet')
    ap.add_argument('--max-w', type=float, default=0.30)
    ap.add_argument('--k', type=float, default=1.0)
    a = ap.parse_args()
    rclpy.init(); n = rclpy.create_node('align_yaw')
    buf = Buffer(); n.tl = TransformListener(buf, n)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    tgt = math.radians(a.yaw); t0 = time.time(); ok = False; err = None; inside = 0
    try:
        while time.time() - t0 < a.timeout:
            rclpy.spin_once(n, timeout_sec=0.05)
            try:
                q = buf.lookup_transform('map', 'base_link', Time()).transform.rotation
            except Exception:
                continue
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            err = math.atan2(math.sin(tgt - yaw), math.cos(tgt - yaw))
            tw = Twist()
            if abs(err) < math.radians(a.tol):
                inside += 1; pub.publish(tw)
                if inside >= 10: ok = True; break      # steady for about 0.5 s
                continue
            inside = 0
            tw.angular.z = math.copysign(max(a.min_w, min(a.max_w, a.k * abs(err))), err)
            pub.publish(tw)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5): pub.publish(Twist()); time.sleep(0.05)
    print('ALIGN: %s, heading error %.1f deg' % ('OK' if ok else 'FAILED', math.degrees(err) if err is not None else float('nan')))
    n.destroy_node(); rclpy.shutdown()
    return 0 if ok else 1

if __name__ == '__main__':
    sys.exit(main())
