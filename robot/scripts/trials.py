#!/usr/bin/env python3
# trials.py [--trips 30] [--mode home|chain] [--ask] [--pause 5] : automatic guide trials (Stage 11).
# Drives the real guide manager (/guide/command, as the GUI does), records a rosbag, logs each trip to
# logs/trials_<time>.csv and prints the gate summary. Needs guide_start.sh (or the service) running.
#   home  : home -> place -> back home (guide behaviour, return on)
#   chain : place -> place -> place (return off), home at the end
# Ctrl+C: cancel (robot goes home), stop the recording, write the summary.
import argparse, csv, json, math, os, random, signal, statistics, subprocess, sys, time, yaml
import rclpy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

WS = os.path.expanduser('~/guide_ws')
TOPICS = ['/scan', '/odom', '/tf', '/tf_static', '/amcl_pose', '/guide/status', '/guide/command', '/limo_status']


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Link:
    def __init__(s):
        s.n = rclpy.create_node('trials'); s.msgs = []
        s.pub = s.n.create_publisher(String, '/guide/command', 10)
        s.n.create_subscription(String, '/guide/status', s.on_status, 10)
        s.buf = Buffer(); s.tl = TransformListener(s.buf, s.n); s.volt = None

    def on_status(s, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        if d.get('battery'): s.volt = d['battery']
        if not d.get('hb'): s.msgs.append(d)

    def send(s, cmd):
        s.mark = len(s.msgs); m = String(); m.data = cmd; s.pub.publish(m)

    def wait(s, states, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(s.n, timeout_sec=0.2)
            for i in range(s.mark, len(s.msgs)):
                if s.msgs[i].get('state') in states: s.mark = i + 1; return s.msgs[i]
        return None

    def pose(s):
        for _ in range(10): rclpy.spin_once(s.n, timeout_sec=0.05)
        try:
            t = s.buf.lookup_transform('map', 'base_link', Time()).transform
            return t.translation.x, t.translation.y, yaw_of(t.rotation)
        except Exception:
            return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trips', type=int, default=30); ap.add_argument('--mode', choices=['home', 'chain'], default='home')
    ap.add_argument('--ask', action='store_true', help='ask for tape parking error and notes after each arrival')
    ap.add_argument('--pause', type=float, default=5.0, help='seconds between trips')
    ap.add_argument('--min-volt', type=float, default=11.2, help='stop the block below this battery voltage')
    ap.add_argument('--timeout', type=float, default=240.0)
    ap.add_argument('--no-bag', action='store_true'); ap.add_argument('--seed', type=int)
    a = ap.parse_args()
    P = yaml.safe_load(open(os.path.join(WS, 'places_lab.yaml')))['places']
    names = [p for p in P if p != 'home']
    if len(names) < 2: sys.exit('need at least 2 places besides home')
    rnd = random.Random(a.seed); plan = []
    while len(plan) < a.trips:  # every place equally often, random order, never the same twice in a row
        blk = names[:]; rnd.shuffle(blk)
        if plan and blk[0] == plan[-1]: blk.append(blk.pop(0))
        plan += blk
    plan = plan[:a.trips]
    ts = time.strftime('%Y%m%d_%H%M'); rclpy.init(); L = Link()
    L.send('status'); first = L.wait({'idle', 'arrived', 'going', 'info', 'error', 'battery', 'failed'}, 15)
    if first is None: sys.exit('no answer on /guide/status: is guide_start.sh (or the limo-guide service) running?')
    bag = None
    if not a.no_bag:
        os.makedirs(os.path.join(WS, 'bags'), exist_ok=True)
        bag = subprocess.Popen(['ros2', 'bag', 'record', '-o', os.path.join(WS, 'bags', 'trials_' + ts)] + TOPICS,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    rows = []; reason = 'done'; prev = 'home'
    print('%d trips (%s mode): %s' % (len(plan), a.mode, ' '.join(plan)), flush=True)
    try:
        if a.mode == 'chain':
            L.send('return off'); L.wait({'info'}, 10)
        for k, place in enumerate(plan, 1):
            if L.volt is not None and L.volt < a.min_volt:
                reason = 'battery %.2f V below %.1f V: charge, then start a new block' % (L.volt, a.min_volt); break
            t0 = time.time(); L.send('go ' + place)
            d = L.wait({'arrived', 'failed', 'error', 'battery'}, a.timeout)
            t_arr = time.time() - t0; st = d['state'] if d else 'timeout'
            ex = ey = float('nan'); tape = notes = ''; t_home = ''
            if st == 'arrived':
                p = L.pose(); x, y, yaw = P[place]
                if p:
                    ex = math.hypot(p[0] - x, p[1] - y)
                    ey = math.degrees(abs(math.atan2(math.sin(p[2] - yaw), math.cos(p[2] - yaw))))
                if a.ask:
                    tape = input('   tape: robot centre -> %s mark (m, Enter = skip): ' % place).strip()
                    notes = input('   notes / interventions (Enter = none): ').strip()
            elif d is None:
                L.send('cancel')
            if (a.mode == 'home' and st == 'arrived') or st in ('failed', 'timeout', 'battery'):
                h = L.wait({'idle'}, a.timeout)
                t_home = '%.1f' % (time.time() - t0) if h else 'timeout'
            rows.append([k, prev if a.mode == 'chain' else 'home', place, st, '%.1f' % t_arr, t_home, '%.3f' % ex,
                         '%.1f' % ey, tape, '%.2f' % L.volt if L.volt else '', (d or {}).get('msg', ''), notes])
            print('%2d/%d %-16s %-8s %5.1f s  parking (own estimate) %.2f m %4.1f deg  battery %s'
                  % (k, len(plan), place, st, t_arr, ex, ey, '%.2f V' % L.volt if L.volt else '?'), flush=True)
            prev = place if (a.mode == 'chain' and st == 'arrived') else 'home'
            if st in ('battery', 'error') and 'battery' in (d or {}).get('msg', ''):
                reason = 'guide manager reports low battery'; break
            for _ in range(int(a.pause * 5)): rclpy.spin_once(L.n, timeout_sec=0.2)
    except KeyboardInterrupt:
        reason = 'stopped by Ctrl+C'; L.send('cancel')
    finally:
        if a.mode == 'chain' or reason == 'stopped by Ctrl+C':
            try:
                if a.mode == 'chain': L.send('home')
                L.wait({'idle'}, 120)
                if a.mode == 'chain': L.send('return on'); L.wait({'info'}, 10)
            except KeyboardInterrupt:
                pass
        if bag is not None:
            os.killpg(bag.pid, signal.SIGINT)
            try: bag.wait(timeout=15)
            except Exception: os.killpg(bag.pid, signal.SIGKILL)
    os.makedirs(os.path.join(WS, 'logs'), exist_ok=True)
    out = os.path.join(WS, 'logs', 'trials_%s.csv' % ts)
    with open(out, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['trip', 'from', 'place', 'result', 'time_to_place_s', 'time_round_trip_s', 'est_err_m',
                    'est_yaw_err_deg', 'tape_err_m', 'battery_v', 'status_msg', 'notes'])
        w.writerows(rows)
    if not rows: print('no trips run: ' + reason); return 1
    ok = [r for r in rows if r[3] == 'arrived']
    mean = lambda v: statistics.mean(v) if v else float('nan')
    est = [float(r[6]) for r in ok if r[6] != 'nan']; tape = [float(r[8]) for r in ok if r[8].replace('.', '', 1).isdigit()]
    park = mean(tape) if tape else mean(est); need = math.ceil(0.9 * len(rows))
    print('\nTRIALS (%s): %d/%d arrived (%.0f %%); time to place mean %.1f s; parking %s mean %.3f m; %s'
          % (reason, len(ok), len(rows), 100.0 * len(ok) / len(rows), mean([float(r[4]) for r in ok]),
             'tape' if tape else 'estimate', park, out))
    for r in rows:
        if r[3] != 'arrived': print('  trip %s to %s: %s %s' % (r[0], r[2], r[3], r[10]))
    gate = len(rows) >= 30 and len(ok) >= need and park <= 0.15
    print('GATE (>= 30 trips, >= 90 %% arrived, parking <= 0.15 m; collisions you log in notes): %s'
          % ('PASS' if gate else ('NOT YET (%d trips)' % len(rows) if len(rows) < 30 else 'FAIL')))
    if bag is not None: print('rosbag: %s' % os.path.join(WS, 'bags', 'trials_' + ts))
    return 0 if gate else 1


if __name__ == '__main__':
    sys.exit(main())
