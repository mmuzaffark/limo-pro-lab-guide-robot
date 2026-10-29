#!/usr/bin/env python3
# auto_map.py : autonomous lab mapping supervisor (ROS 2 Foxy, slam_toolbox + Nav2 + explore_lite e40e857)
# Sequence: wait for stack -> battery check -> 360 deg scan -> explore -> return home -> save map -> report.
# Started by auto_map.sh, which already runs the base, SLAM and Nav2. Ctrl+C stops the robot.
import argparse, math, os, shutil, signal, subprocess, threading, time
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener
try:
    from limo_msgs.msg import LimoStatus
except ImportError:
    LimoStatus = None

HOME = os.path.expanduser('~/guide_ws')


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Mapper:
    def __init__(s, a):
        s.a = a
        s.n = rclpy.create_node('auto_map')
        s.tf = Buffer(); s.tl = TransformListener(s.tf, s.n)
        s.cmd = s.n.create_publisher(Twist, '/cmd_vel', 10)
        s.resume = s.n.create_publisher(Bool, '/explore/resume', 10)
        s.nav = ActionClient(s.n, NavigateToPose, 'navigate_to_pose')
        q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
        s.map = None; s.volt = None
        s.n.create_subscription(OccupancyGrid, '/map', s.on_map, q)
        if LimoStatus is not None:
            s.n.create_subscription(LimoStatus, '/limo_status', s.on_status, 10)
        s.explore = None; s.ex_done = False; s.ex_lines = []
        s.path = 0.0; s.last_odom = None
        s.log_lines = []

    # ---------- helpers ----------
    def on_map(s, m): s.map = m
    def on_status(s, m): s.volt = m.battery_voltage

    def log(s, txt):
        line = '[%s] %s' % (time.strftime('%H:%M:%S'), txt)
        print(line, flush=True); s.log_lines.append(line)

    def wait(s, sec):
        t = time.time()
        while time.time() - t < sec:
            try:
                rclpy.spin_once(s.n, timeout_sec=0.05)
            except Exception:  # context already shut down after Ctrl+C
                time.sleep(0.05)

    def pose(s, frame='map'):
        try:
            t = s.tf.lookup_transform(frame, "base_link", Time())
        except Exception:
            return None
        p = t.transform.translation
        return (p.x, p.y, yaw_of(t.transform.rotation))

    def track_path(s):
        p = s.pose('odom')
        if p is None: return
        if s.last_odom is not None:
            s.path += math.hypot(p[0] - s.last_odom[0], p[1] - s.last_odom[1])
        s.last_odom = p

    def stop_robot(s):
        for _ in range(5):
            try:
                s.cmd.publish(Twist())
            except Exception:
                pass
            time.sleep(0.05)

    def map_stats(s):
        m = s.map
        if m is None: return {}
        w, h, r = m.info.width, m.info.height, m.info.resolution
        d = m.data
        free = sum(1 for v in d if v == 0)
        occ = sum(1 for v in d if v >= 65)
        xs, ys = [], []
        for i, v in enumerate(d):
            if v >= 65:
                xs.append(i % w); ys.append(i // w)
        size = ((max(xs) - min(xs) + 1) * r, (max(ys) - min(ys) + 1) * r) if xs else (0.0, 0.0)
        return {'free_m2': free * r * r, 'occupied_cells': occ, 'walls_bbox_m': size,
                'grid': '%dx%d @ %.2f m' % (w, h, r)}

    # ---------- steps ----------
    def wait_ready(s, timeout=120):
        s.log('waiting for /map, map->base_link TF and Nav2 (up to %d s) ...' % timeout)
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(s.n, timeout_sec=0.2)
            if s.map is not None and s.pose() is not None and s.nav.server_is_ready():
                s.log('stack ready after %.0f s' % (time.time() - t0)); return True
        s.log('NOT READY: map=%s tf=%s nav2=%s' % (s.map is not None, s.pose() is not None,
                                                    s.nav.server_is_ready()))
        return False

    def battery_ok(s):
        if LimoStatus is None:
            s.log('limo_msgs not found: battery check skipped'); return True
        s.wait(3.0)
        if s.volt is None:
            s.log('no /limo_status: battery check skipped'); return True
        s.log('battery %.2f V (start needs >= %.1f V)' % (s.volt, s.a.min_volt))
        return s.volt >= s.a.min_volt

    def scan_spin(s):
        # one slow turn in place so the 140 deg rear blind sector is mapped before exploring
        s.log('360 deg scan turn at %.2f rad/s' % s.a.spin_speed)
        tw = Twist(); tw.angular.z = s.a.spin_speed
        acc = 0.0; prev = None; t0 = time.time()
        limit = 2 * math.pi / s.a.spin_speed * 2.0
        while acc < 2 * math.pi and time.time() - t0 < limit:
            s.cmd.publish(tw); rclpy.spin_once(s.n, timeout_sec=0.1)
            p = s.pose('odom')
            if p is not None:
                if prev is not None: acc += abs(wrap(p[2] - prev))
                prev = p[2]
        s.stop_robot(); s.wait(2.0)
        s.log('turned %.0f deg' % math.degrees(acc))

    def start_explore(s):
        cmd = ['ros2', 'run', 'explore_lite', 'explore', '--ros-args',
               '--params-file', os.path.join(HOME, 'params/explore_lab.yaml')]
        env = dict(os.environ, RCUTILS_LOGGING_BUFFERED_STREAM='0', RCUTILS_COLORIZED_OUTPUT='0')
        s.explore = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     universal_newlines=True, bufsize=1, env=env, start_new_session=True)
        f = open(os.path.join(HOME, 'logs', 'am_%s_explore.log' % s.a.tag), 'w')

        def reader():
            for line in s.explore.stdout:
                f.write(line); f.flush()
                s.ex_lines.append(line.strip())
                if 'Exploration stopped' in line:
                    s.ex_done = True
        threading.Thread(target=reader, daemon=True).start()
        s.log('explore_lite started (log: logs/am_%s_explore.log)' % s.a.tag)

    def stop_explore(s):
        if s.explore is None or s.explore.poll() is not None: return
        try:
            s.resume.publish(Bool(data=False))  # explore cancels its own Nav2 goal
        except Exception:
            pass
        s.wait(2.0)
        try:
            os.killpg(s.explore.pid, signal.SIGINT); s.explore.wait(timeout=8)
        except Exception:
            try: os.killpg(s.explore.pid, signal.SIGKILL)
            except Exception: pass

    def explore_loop(s):
        t0 = time.time(); last_move = time.time(); anchor = s.pose(); last_print = 0
        while True:
            rclpy.spin_once(s.n, timeout_sec=0.5); s.track_path()
            el = time.time() - t0; p = s.pose()
            if p is not None and anchor is not None:
                if math.hypot(p[0] - anchor[0], p[1] - anchor[1]) > 0.10 or abs(wrap(p[2] - anchor[2])) > 0.3:
                    anchor = p; last_move = time.time()
            if el - last_print >= 30:
                last_print = el; st = s.map_stats()
                s.log('exploring %3.0f s  path %.1f m  free %.1f m2  battery %s' % (
                    el, s.path, st.get('free_m2', 0), '%.2f V' % s.volt if s.volt else '?'))
            if s.ex_done:
                last = [l for l in s.ex_lines if 'stopping' in l or 'stopped' in l]
                return 'complete (%s)' % (last[0].split(']: ')[-1].strip() if last else 'explore stopped')
            if s.explore.poll() is not None:
                return 'explore_lite exited (code %s)' % s.explore.returncode
            if el > s.a.max_min * 60:
                return 'time limit %d min' % s.a.max_min
            if s.volt is not None and s.volt < s.a.abort_volt:
                return 'battery below %.1f V' % s.a.abort_volt
            if time.time() - last_move > s.a.stall_s:
                return 'no movement for %d s' % s.a.stall_s

    def go_home(s):
        for attempt in (1, 2):
            g = NavigateToPose.Goal(); g.pose = PoseStamped()
            g.pose.header.frame_id = 'map'; g.pose.header.stamp = s.n.get_clock().now().to_msg()
            g.pose.pose.orientation.w = 1.0
            s.log('returning home (attempt %d)' % attempt)
            f = s.nav.send_goal_async(g); rclpy.spin_until_future_complete(s.n, f, timeout_sec=10)
            h = f.result()
            if h is None or not h.accepted:
                s.log('home goal rejected'); s.wait(3); continue
            r = h.get_result_async(); t0 = time.time()
            while not r.done() and time.time() - t0 < 240:
                rclpy.spin_once(s.n, timeout_sec=0.5); s.track_path()
            if not r.done():
                h.cancel_goal_async(); s.wait(2); s.log('home goal timed out'); continue
            if r.result().status == GoalStatus.STATUS_SUCCEEDED:
                s.wait(1); p = s.pose()
                d = math.hypot(p[0], p[1]) if p else None
                if d is not None and d <= 0.3:
                    return True
                s.log('Nav2 succeeded but SLAM home error %s m: retry' % ('%.2f' % d if d is not None else '?'))
                s.wait(2); continue
            s.log('home goal ended with status %d' % r.result().status); s.wait(3)
        return False

    def save(s):
        base = os.path.join(HOME, 'maps', 'lab_auto_' + s.a.tag)
        ok = False
        for _ in range(2):
            r = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', base],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
            if os.path.exists(base + '.pgm') and os.path.exists(base + '.yaml'):
                ok = True; break
            s.log('map_saver retry: ' + r.stdout.strip().splitlines()[-1] if r.stdout.strip() else 'map_saver retry')
            s.wait(2)
        try:
            subprocess.run(['ros2', 'service', 'call', '/slam_toolbox/serialize_map',
                            'slam_toolbox/srv/SerializePoseGraph', "{filename: '%s'}" % base],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=60)
        except Exception:
            s.log('pose graph not saved (slam_toolbox did not answer)')
        if ok:  # newest map is also available as lab_auto.* (used by Stage 7)
            latest = os.path.join(HOME, 'maps', 'lab_auto')
            shutil.copy(base + '.pgm', latest + '.pgm')
            y = open(base + '.yaml').read().replace(os.path.basename(base) + '.pgm', 'lab_auto.pgm')
            open(latest + '.yaml', 'w').write(y)
            for ext in ('.posegraph', '.data'):
                if os.path.exists(base + ext): shutil.copy(base + ext, latest + ext)
        return base if ok else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tag', default=time.strftime('%Y%m%d_%H%M'))
    ap.add_argument('--max-min', type=int, default=20, help='exploration time limit (min)')
    ap.add_argument('--stall-s', type=int, default=120, help='stop if the robot has not moved this long (s)')
    ap.add_argument('--min-volt', type=float, default=11.5, help='battery needed to start (V)')
    ap.add_argument('--abort-volt', type=float, default=10.9, help='stop exploring and go home below (V)')
    ap.add_argument('--spin-speed', type=float, default=0.3, help='start-up scan turn (rad/s)')
    ap.add_argument('--no-spin', action='store_true')
    a = ap.parse_args()
    os.makedirs(os.path.join(HOME, 'logs'), exist_ok=True)
    rclpy.init(); m = Mapper(a); t_start = time.time()
    reason = 'not started'; home_ok = False; saved = None; volt0 = None
    try:
        if not m.wait_ready():
            reason = 'stack not ready (see logs/am_%s_*.log)' % a.tag; return
        if not m.battery_ok():
            reason = 'battery too low to start: charge first'; return
        volt0 = m.volt
        m.track_path()
        if not a.no_spin: m.scan_spin()
        m.start_explore()
        reason = m.explore_loop()
        m.log('exploration ended: ' + reason)
        m.stop_explore()
        home_ok = m.go_home()
    except KeyboardInterrupt:
        reason = 'stopped by Ctrl+C'
        m.stop_explore(); m.stop_robot()
    finally:
        m.stop_robot()
        if m.map is not None and reason not in ('not started',) and not reason.startswith(('stack', 'battery too')):
            saved = m.save()
        st = m.map_stats(); p = m.pose(); dur = time.time() - t_start
        rep = [
            'AUTO MAP REPORT %s' % a.tag,
            'result:          %s' % reason,
            'returned home:   %s' % ('yes' if home_ok else 'NO'),
            'home error (SLAM): %s' % ('%.2f m, %.1f deg' % (math.hypot(p[0], p[1]), math.degrees(p[2])) if p else '?'),
            'duration:        %.1f min' % (dur / 60),
            'path driven:     %.1f m' % m.path,
            'free area:       %.1f m2' % st.get('free_m2', 0),
            'walls bbox:      %.2f x %.2f m' % st.get('walls_bbox_m', (0, 0)),
            'grid:            %s' % st.get('grid', '?'),
            'battery:         %s -> %s V' % (volt0, m.volt),
            'map files:       %s' % ((saved + '.{pgm,yaml,posegraph,data}  (copied to maps/lab_auto.*)') if saved else 'NOT SAVED'),
        ]
        out = os.path.join(HOME, 'logs', 'auto_map_%s.txt' % a.tag)
        open(out, 'w').write('\n'.join(rep + ['', 'timeline:'] + m.log_lines) + '\n')
        print('\n'.join(rep)); print('report: ' + out, flush=True)
        try:
            m.n.destroy_node(); rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
