#!/usr/bin/env python3
# nav_test.py [--rounds N] [--via-home] [--ask] [PLACE ...] : Stage 8 goal test in one command.
# Sends every place (or the ones named) in turn and finally home, through Nav2. For each goal it logs
# result, time, the robot's own parking error (map pose vs place), battery and, with --ask, your tape
# measurement, to logs/nav_test_<time>.csv. Needs nav_start.sh running. Ctrl+C cancels and summarizes.
import argparse, csv, math, os, statistics, subprocess, sys, time, yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from tf2_ros import Buffer, TransformListener
try:
    from limo_msgs.msg import LimoStatus
except ImportError:
    LimoStatus = None

WS = os.path.expanduser('~/guide_ws')
NAMES = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_CANCELED: 'CANCELED', GoalStatus.STATUS_ABORTED: 'ABORTED'}


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('places', nargs='*'); ap.add_argument('--rounds', type=int, default=1)
    ap.add_argument('--via-home', action='store_true', help='return home between places')
    ap.add_argument('--ask', action='store_true', help='ask for the tape-measured parking error')
    ap.add_argument('--timeout', type=float, default=180.0)
    ap.add_argument('--no-align', action='store_true', help='skip align_yaw.py after each goal')
    a = ap.parse_args()
    P = yaml.safe_load(open(os.path.join(WS, 'places_lab.yaml')))['places']
    names = a.places or [p for p in P if p != 'home']
    bad = [p for p in names if p not in P]
    if bad: sys.exit('unknown place(s): %s  (have: %s)' % (', '.join(bad), ', '.join(P)))
    seq = []
    for _ in range(a.rounds):
        for p in names: seq += [p, 'home'] if a.via_home else [p]
    if not seq or seq[-1] != 'home': seq.append('home')
    rclpy.init(); n = rclpy.create_node('nav_test'); ac = ActionClient(n, NavigateToPose, 'navigate_to_pose')
    buf = Buffer(); n.tl = TransformListener(buf, n); volt = [None]
    if LimoStatus is not None:
        n.create_subscription(LimoStatus, '/limo_status', lambda m: volt.__setitem__(0, m.battery_voltage), 10)
    if not ac.wait_for_server(timeout_sec=20.0): sys.exit('Nav2 not running: start nav_start.sh first')
    rows = []; h = None
    print('%d goals: %s' % (len(seq), ' -> '.join(seq)), flush=True)
    try:
        for k, name in enumerate(seq, 1):
            x, y, yaw = P[name]
            g = NavigateToPose.Goal(); g.pose = PoseStamped(); g.pose.header.frame_id = 'map'
            g.pose.header.stamp = n.get_clock().now().to_msg()
            g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
            g.pose.pose.orientation.z = math.sin(yaw / 2); g.pose.pose.orientation.w = math.cos(yaw / 2)
            t0 = time.time(); f = ac.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10)
            h = f.result()
            if h is None or not h.accepted:
                st = 'REJECTED'
            else:
                r = h.get_result_async()
                while not r.done() and time.time() - t0 < a.timeout: rclpy.spin_once(n, timeout_sec=0.2)
                if not r.done():
                    h.cancel_goal_async(); st = 'TIMEOUT'
                else:
                    st = NAMES.get(r.result().status, str(r.result().status))
                h = None
            dt = time.time() - t0
            if st == 'SUCCEEDED' and not a.no_align:
                subprocess.run(['python3', os.path.join(WS, 'scripts', 'align_yaw.py'), '%.1f' % math.degrees(yaw)])
            te = time.time() + 2.0
            while time.time() < te: rclpy.spin_once(n, timeout_sec=0.05)
            try:
                tr = buf.lookup_transform('map', 'base_link', Time()).transform
                ex = math.hypot(tr.translation.x - x, tr.translation.y - y)
                ey = math.degrees(abs(math.atan2(math.sin(yaw_of(tr.rotation) - yaw), math.cos(yaw_of(tr.rotation) - yaw))))
            except Exception:
                ex = ey = float('nan')
            tape = ''
            if a.ask and st == 'SUCCEEDED' and name != 'home':
                tape = input('   tape: distance robot centre -> %s mark in m (Enter = skip): ' % name).strip()
            rows.append([k, name, st, '%.1f' % dt, '%.3f' % ex, '%.1f' % ey, tape, '%.2f' % volt[0] if volt[0] else ''])
            print('%2d/%d %-16s %-9s %5.1f s  parking (own estimate) %.2f m %4.1f deg  battery %s'
                  % (k, len(seq), name, st, dt, ex, ey, '%.2f V' % volt[0] if volt[0] else '?'), flush=True)
    except KeyboardInterrupt:
        if h is not None: h.cancel_goal_async()
        print('\nstopped by Ctrl+C')
    os.makedirs(os.path.join(WS, 'logs'), exist_ok=True)
    out = os.path.join(WS, 'logs', 'nav_test_%s.csv' % time.strftime('%Y%m%d_%H%M'))
    with open(out, 'w', newline='') as fh:
        w = csv.writer(fh); w.writerow(['n', 'place', 'result', 'time_s', 'est_err_m', 'est_yaw_err_deg', 'tape_err_m', 'battery_v'])
        w.writerows(rows)
    ok = [r for r in rows if r[2] == 'SUCCEEDED' and r[1] != 'home']; tot = [r for r in rows if r[1] != 'home']
    if not tot: return 1
    em = [float(r[4]) for r in ok if r[4] != 'nan']; ym = [float(r[5]) for r in ok if r[5] != 'nan']
    tp = [float(r[6]) for r in ok if r[6].replace('.', '', 1).isdigit()]
    mean = lambda v: statistics.mean(v) if v else float('nan')
    park = mean(tp) if tp else mean(em)
    gate = len(ok) == len(tot) and park <= 0.25 and (max(ym) if ym else 99) <= 11.5
    print('\nNAV TEST: %d/%d places reached; parking %s mean %.3f m (max %.3f); heading error max %.1f deg -> %s'
          % (len(ok), len(tot), 'tape' if tp else 'estimate', park, max(tp or em or [float('nan')]),
             max(ym) if ym else float('nan'), out))
    print('GATE (all reached, parking <= 0.25 m, heading <= 11.5 deg): %s' % ('PASS' if gate else 'FAIL'))
    for r in rows:
        if r[2] != 'SUCCEEDED': print('  failed: %s %s -> look at ~/guide_ws/logs/nav.log' % (r[1], r[2]))
    return 0 if gate else 1


if __name__ == '__main__':
    sys.exit(main())
