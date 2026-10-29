#!/usr/bin/env python3
# calib_all.py [DIST_m] : the whole Stage 3 calibration in one command (flat wall = ground truth).
# 3 x (square up to the wall, drive DIST toward it, measure, reverse back), then near the wall
# 3 spins CCW + 3 spins CW. Writes logs/calib_<time>.csv and prints the gate result.
import csv, math, os, statistics, sys, time
sys.path.insert(0, os.path.expanduser('~/guide_ws/scripts'))
import rclpy
from geometry_msgs.msg import Twist
from calib import Calib

LOG = os.path.expanduser('~/guide_ws/logs')


def stop(c):
    for _ in range(5):
        c.pub.publish(Twist()); time.sleep(0.02)


def wall(c, scans=20, timeout=10.0):
    # like Calib.wall, but gives up when the wall is not visible
    ds, ps = [], []; t0 = time.time()
    while len(ds) < scans:
        if time.time() - t0 > timeout:
            raise RuntimeError('wall not visible: face a flat wall, 1-4 m away')
        c.scan = None
        while c.scan is None: rclpy.spin_once(c.n, timeout_sec=0.1)
        P = c.points()
        if len(P) < 15: continue
        mx = statistics.mean(p[0] for p in P); my = statistics.mean(p[1] for p in P)
        syy = sum((p[1] - my) ** 2 for p in P)
        sxy = sum((p[0] - mx) * (p[1] - my) for p in P)
        a = sxy / syy; b = mx - a * my; psi = math.atan(a)
        ds.append(b * math.cos(psi)); ps.append(math.degrees(psi))
    return statistics.mean(ds), statistics.mean(ps)


def rotate(c, deg, fast=0.5, slow=0.2):
    y0 = c.yaw; tw = Twist(); sg = 1.0 if deg > 0 else -1.0
    while True:
        rclpy.spin_once(c.n, timeout_sec=0.02)
        done = abs(math.degrees(c.yaw - y0))
        if done >= abs(deg): break
        tw.angular.z = sg * (fast if abs(deg) - done > 30.0 else slow); c.pub.publish(tw)
    stop(c)


def drive(c, dist, v=0.15, guard=True):
    x0 = c.pose; tw = Twist(); sg = 1.0 if dist > 0 else -1.0
    while True:
        rclpy.spin_once(c.n, timeout_sec=0.02)
        od = math.hypot(c.pose[0] - x0[0], c.pose[1] - x0[1])
        if od >= abs(dist): break
        if guard and sg > 0:
            front = [p[0] for p in c.points(5, 10.0) if abs(p[1]) < 0.15]
            if front and min(front) < 0.45:
                print('  safety stop: wall < 0.45 m'); break
        tw.linear.x = sg * (v if abs(dist) - od > 0.15 else 0.06); c.pub.publish(tw)
    stop(c)
    return math.hypot(c.pose[0] - x0[0], c.pose[1] - x0[1])


def square_up(c, tol=1.0):
    # turn until the robot faces the wall squarely (heading to wall within tol deg)
    for _ in range(5):
        d, p = wall(c)
        if abs(p) <= tol: return d, p
        rotate(c, -p, fast=0.2, slow=0.1); c.wait(1.0)
    return wall(c)


def main():
    D = float(sys.argv[1]) if len(sys.argv) > 1 else 2.0
    rclpy.init(); c = Calib(); c.wait(1.5)
    if c.pose is None or c.scan is None:
        print('no /odom or /scan: is limo_start_guide running, ROS_DOMAIN_ID 30?'); return 2
    rows = []; S = []; Y = {'ccw': [], 'cw': []}; I = {'ccw': [], 'cw': []}; H = []
    try:
        d, p = square_up(c)
        print('start: wall %.3f m, heading %+.1f deg' % (d, p))
        if not (D + 0.8 <= d <= D + 1.6):
            print('move the robot to %.1f-%.1f m from the wall, facing it, and run again' % (D + 0.8, D + 1.4))
            return 2
        for i in (1, 2, 3):
            square_up(c); c.wait(0.5); d0, p0 = wall(c)
            od = drive(c, D); c.wait(1.5); d1, p1 = wall(c)
            real = (d0 - d1) / math.cos(math.radians((p0 + p1) / 2))
            e = (od - real) / real * 100; S.append(e); H.append(p1 - p0)
            rows.append(['straight', i, '%.3f' % real, '%.3f' % od, '', '%+.2f' % e, '', '%+.1f' % (p1 - p0)])
            print('straight %d: real %.3f m  odom %.3f m  error %+.1f %%  heading change %+.1f deg'
                  % (i, real, od, e, p1 - p0))
            if i == 3:  # spins about 1 m from the wall
                for mode, sg in (('ccw', 1.0), ('cw', -1.0)):
                    for k in (1, 2, 3):
                        square_up(c); c.wait(0.5); a0 = wall(c)[1]; y0 = c.yaw; i0 = c.imu
                        rotate(c, sg * 360.0); c.wait(1.5); a1 = wall(c)[1]
                        real = sg * 360.0 + (a1 - a0)
                        oy = math.degrees(c.yaw - y0); iy = math.degrees(c.imu - i0)
                        eo = (oy - real) / abs(real) * 100; ei = (iy - real) / abs(real) * 100
                        Y[mode].append(eo); I[mode].append(ei)
                        rows.append([mode, k, '%+.1f' % real, '%+.1f' % oy, '%+.1f' % iy, '%+.2f' % eo, '%+.2f' % ei, ''])
                        print('%-3s %d: real %+.1f deg  odom %+.1f deg (%+.1f %%)  imu %+.1f deg (%+.1f %%)'
                              % (mode, k, real, oy, eo, iy, ei))
            square_up(c)
            drive(c, -od, v=0.12, guard=False)  # back over the path just driven
            c.wait(1.0)
    except KeyboardInterrupt:
        print('stopped by Ctrl+C')
    except RuntimeError as ex:
        print('STOPPED: %s' % ex)
    finally:
        stop(c)
    if not rows: return 2
    os.makedirs(LOG, exist_ok=True)
    out = os.path.join(LOG, 'calib_%s.csv' % time.strftime('%Y%m%d_%H%M'))
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['test', 'run', 'real', 'odom', 'imu', 'err_odom_pct', 'err_imu_pct', 'heading_change_deg'])
        w.writerows(rows)
    m = lambda v: statistics.mean(v) if v else float('nan')
    s, yo = m(S), max(abs(m(Y['ccw'])), abs(m(Y['cw'])))
    yi = max(abs(m(I['ccw'])), abs(m(I['cw'])))
    ok_s, ok_y = abs(s) <= 2.0, yo <= 3.0
    print('\nSUMMARY (%d straight, %d+%d spins)  ->  %s' % (len(S), len(Y['ccw']), len(Y['cw']), out))
    print('straight odom error mean %+.2f %%   gate <= 2 %%: %s' % (s, 'PASS' if ok_s else 'FAIL'))
    print('yaw odom error ccw %+.2f %%  cw %+.2f %%   gate <= 3 %%: %s' % (m(Y['ccw']), m(Y['cw']), 'PASS' if ok_y else 'FAIL'))
    print('yaw imu  error ccw %+.2f %%  cw %+.2f %%' % (m(I['ccw']), m(I['cw'])))
    if not ok_y and yi <= 3.0:
        print('ADVICE: odometry yaw is off but the IMU is good -> add the EKF (robot_localization) with IMU yaw')
    if not ok_s:
        print('ADVICE: linear scale off; real/odom factor = %.4f (send this to me)' % (1 / (1 + s / 100)))
    if H and abs(m(H)) > 2.0:
        print('ADVICE: robot curves %+.1f deg per %.1f m: check tyres and wheel mode' % (m(H), D))
    return 0 if ok_s and ok_y else 1


if __name__ == '__main__':
    sys.exit(main())
