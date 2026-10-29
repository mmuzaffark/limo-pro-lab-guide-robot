#!/bin/bash
# install_guide.sh [--force-params] : rebuilds the whole robot side of the lab guide (Stages 2-11) on a Limo Pro
# (Foxy, Jetson): every script in ~/guide_ws/scripts, the launch files, the parameter files and places_lab.yaml
# (kept if they already exist, unless --force-params), ~/guide_env.sh (only if missing), and installs the
# limo-guide service (not enabled). Nothing drives. Run inside tmux "guide": bash ~/install_guide.sh
# Generated 29 Oct 2026 from github.com/mmuzaffark/limo-pro-lab-guide-robot robot/ (commit 2993d24 2026-10-29 14:01):
# the files are byte-identical to the ones running on the robot after the 29 Oct tests (nav gate 6/6, 12/12 trials).
# Not included (see the guide Doc): ROS 2 Foxy + limo_ros2 + Nav2 0.4.7 source build in ~/limo_ros2_ws with the
# RPP goal_dist_tol patch (28 Oct), explore_lite e40e857, Cyclone DDS (make_cyclone.sh LAPTOP_IP), the maps
# (copy robot/maps/lab_v1.* from the repo to ~/guide_ws/maps), the SSH key and the laptop files.
mkdir -p ~/guide_ws/{scripts,params,maps,logs,bags,launch}
FORCE=0; [ "${1:-}" = "--force-params" ] && FORCE=1
[ -f ~/guide_env.sh ] || {
echo "== ~/guide_env.sh (missing: written)"
cat > ~/guide_env.sh <<'GUIDE_EOF'
source /opt/ros/foxy/setup.bash
source ~/limo_ros2_ws/install/setup.bash
[ -f ~/guide_ws/install/setup.bash ] && source ~/guide_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=30
export CYCLONEDDS_URI=file://$HOME/guide_ws/cyclone.xml
GUIDE_EOF
}
source ~/guide_env.sh

echo "== scripts (19 Python, 9 shell)"
cat > ~/guide_ws/scripts/align_yaw.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/align_yaw.py
cat > ~/guide_ws/scripts/auto_map.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/auto_map.py
cat > ~/guide_ws/scripts/calib.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/calib.py
cat > ~/guide_ws/scripts/calib_all.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/calib_all.py
cat > ~/guide_ws/scripts/compare_maps.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# compare_maps.py A.yaml B.yaml : compare two maps saved from the same home pose
# Prints free area, wall extent and the overlap (IoU) of free space. No numpy needed.
import os, sys, yaml

def load(path):
    m = yaml.safe_load(open(path))
    img = os.path.join(os.path.dirname(os.path.abspath(path)), m['image'])
    with open(img, 'rb') as f:
        assert f.readline().strip() == b'P5', 'not a binary PGM'
        line = f.readline()
        while line.startswith(b'#'): line = f.readline()
        w, h = map(int, line.split()); f.readline()
        px = f.read(w * h)
    r = m['resolution']; ox = round(m['origin'][0] / r); oy = round(m['origin'][1] / r)
    free, occ = set(), set()
    for i, v in enumerate(px):
        c = (ox + i % w, oy + h - 1 - i // w)       # global cell index, origin-aligned
        if v >= 250: free.add(c)                    # 254 = free
        elif v <= 50: occ.add(c)                    # 0 = occupied
    return r, free, occ

def extent(cells, r):
    if not cells: return (0.0, 0.0)
    xs = [c[0] for c in cells]; ys = [c[1] for c in cells]
    return ((max(xs) - min(xs) + 1) * r, (max(ys) - min(ys) + 1) * r)

ra, fa, oa = load(sys.argv[1]); rb, fb, ob = load(sys.argv[2])
assert abs(ra - rb) < 1e-6, 'maps have different resolution'
for name, f, o in ((sys.argv[1], fa, oa), (sys.argv[2], fb, ob)):
    print('%-45s free %6.1f m2   walls %.2f x %.2f m' % (os.path.basename(name), len(f) * ra * ra, *extent(o, ra)))
inter = len(fa & fb); union = len(fa | fb)
diff = abs(len(fa) - len(fb)) / max(len(fa), len(fb)) * 100
print('free-area difference %.1f %%   free-space overlap (IoU) %.1f %%' % (diff, inter / union * 100))
GUIDE_EOF
chmod +x ~/guide_ws/scripts/compare_maps.py
cat > ~/guide_ws/scripts/front_range.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# front_range.py : mean range of beams within +-3 deg of straight ahead, over 30 scans
import math, statistics, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('front_range'); vals = []
def cb(m):
    r = [x for i, x in enumerate(m.ranges)
         if abs(m.angle_min + i * m.angle_increment) <= math.radians(3) and 0.02 < x < 10]
    if r: vals.append(statistics.median(r))
n.create_subscription(LaserScan, '/scan', cb, qos_profile_sensor_data)
while len(vals) < 30: rclpy.spin_once(n, timeout_sec=1.0)
print('front range: mean %.3f m  std %.3f m  (30 scans)' % (statistics.mean(vals), statistics.pstdev(vals)))
GUIDE_EOF
chmod +x ~/guide_ws/scripts/front_range.py
cat > ~/guide_ws/scripts/guide_manager_foxy.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# guide_manager_foxy.py : guide behaviour on the robot. Commands on /guide/command (std_msgs/String):
#   go PLACE | home | cancel | return on | return off | list | status | quit
#   quit = drive home (2 attempts), publish state 'shutdown', then stop all robot software
#   (the limo-guide service, or the guide_start.sh process group). The Jetson stays on.
# Status JSON on /guide/status (repeated every 5 s with "hb": true). Battery safety: below low_volt
# for low_hold_s seconds it refuses new trips and drives home.
import json, math, os, signal, subprocess, sys, time, yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
try:
    from limo_msgs.msg import LimoStatus
except ImportError:
    LimoStatus = None


class Guide(Node):
    def __init__(self, places_file):
        super().__init__('guide_manager')
        self.places = yaml.safe_load(open(places_file))['places']
        self.declare_parameter('wait_s', 10.0)
        self.declare_parameter('low_volt', 10.5)
        self.declare_parameter('low_hold_s', 20.0)
        self.ret = True; self.goal = None; self.timer = None; self.seq = 0
        self.target = 'home'; self.volts = []; self.low_sent = False; self.last = {}
        self.quitting = False; self.home_tries = 0; self.done = False
        self.align = None; self.align_timer = None; self.declare_parameter('align', True)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub = self.create_publisher(String, '/guide/status', 10)
        self.create_subscription(String, '/guide/command', self.on_cmd, 10)
        if LimoStatus is not None:
            self.create_subscription(LimoStatus, '/limo_status', self.on_battery, 10)
        self.create_timer(5.0, self.heartbeat)
        self.create_timer(2.0, self.check_battery)
        self.status('idle', 'home', 'ready: ' + ', '.join(self.places))

    # ---------- battery ----------
    def on_battery(self, m):
        t = time.time(); self.volts.append((t, m.battery_voltage))
        self.volts = [(a, v) for (a, v) in self.volts if t - a < 60]

    def volt(self):
        return self.volts[-1][1] if self.volts else None

    def battery_low(self):
        hold = float(self.get_parameter('low_hold_s').value); t = time.time()
        recent = [v for (a, v) in self.volts if t - a <= hold]
        span = (t - min(a for (a, v) in self.volts if t - a <= hold)) if recent else 0
        return len(recent) >= 5 and span >= hold * 0.8 and max(recent) < float(self.get_parameter('low_volt').value)

    def check_battery(self):
        if self.quitting: return  # already on the way home
        if not self.battery_low():
            self.low_sent = False; return
        if self.low_sent: return
        self.low_sent = True
        if self.target != 'home' or self.timer is not None:
            self.status('battery', self.target, 'battery low (%.2f V), returning home' % self.volt())
            self.send('home')
        else:
            self.status('battery', 'home', 'battery low (%.2f V): please charge me' % self.volt())

    # ---------- status ----------
    def status(self, state, target, msg='', log=True):
        v = self.volt()
        self.last = {'state': state, 'target': target, 'msg': msg, 'battery': round(v, 2) if v else None}
        s = String(); s.data = json.dumps(self.last); self.pub.publish(s)
        if log: self.get_logger().info(s.data)

    def heartbeat(self):
        if self.last:
            d = dict(self.last); d['hb'] = True; v = self.volt(); d['battery'] = round(v, 2) if v else None
            s = String(); s.data = json.dumps(d); self.pub.publish(s)

    # ---------- commands ----------
    def on_cmd(self, m):
        w = m.data.strip().lower().split()
        if not w:
            return
        if self.quitting and w[0] != 'status':
            self.status('quitting', 'home', 'shutting down: command ignored'); return
        if w[0] == 'quit':
            self.quitting = True; self.home_tries = 0
            self.status('quitting', 'home', 'returning home, then stopping the robot software')
            self.send('home'); return
        if w[0] == 'go' and len(w) > 1:
            name = '_'.join(w[1:])
            if name not in self.places:
                self.status('error', name, 'unknown place')
            elif self.battery_low():
                self.status('error', name, 'battery low (%.2f V): please charge me' % self.volt())
            else:
                self.send(name)
        elif w[0] in ('home', 'cancel', 'stop'):
            self.send('home')
        elif w[0] == 'return' and len(w) > 1:
            self.ret = (w[1] == 'on')
            self.status('info', None, 'return home ' + ('on' if self.ret else 'off'))
        elif w[0] == 'list':
            self.status('info', None, ', '.join(self.places))
        elif w[0] == 'status' and self.last:
            self.status(self.last['state'], self.last['target'], self.last['msg'], log=False)

    def clear_timer(self):
        if self.timer is not None:
            self.timer.cancel(); self.destroy_timer(self.timer); self.timer = None

    def send(self, name):
        self.clear_timer(); self.stop_align()
        self.seq += 1; s = self.seq; self.target = name
        self.goal = None  # Foxy: no cancel before a new goal (cancel aborts the new one too); Nav2 preempts
        if not self.ac.wait_for_server(timeout_sec=2.0):
            self.status('error', name, 'Nav2 not running')
            if self.quitting: self.finish(False)
            return
        x, y, yaw = self.places[name]
        g = NavigateToPose.Goal(); g.pose = PoseStamped()
        g.pose.header.frame_id = 'map'; g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(x); g.pose.pose.position.y = float(y)
        g.pose.pose.orientation.z = math.sin(yaw / 2.0); g.pose.pose.orientation.w = math.cos(yaw / 2.0)
        if not self.quitting: self.status('going', name)
        self.ac.send_goal_async(g).add_done_callback(lambda f: self.on_accept(f, name, s))

    def on_accept(self, f, name, s):
        h = f.result()
        if s != self.seq:
            if h.accepted:
                h.cancel_goal_async()
            return
        if not h.accepted:
            self.status('failed', name, 'goal rejected')
            if self.quitting: self.home_failed()
            return
        self.goal = h
        h.get_result_async().add_done_callback(lambda r: self.on_result(r, name, s))

    def on_result(self, r, name, s, aligned=False):
        if s != self.seq:
            return
        self.goal = None
        st = r.result().status
        if st == GoalStatus.STATUS_SUCCEEDED and not aligned and self.get_parameter('align').value:
            self.status('aligning', name, 'turning to the place heading')
            self.align = subprocess.Popen(['python3', os.path.expanduser('~/guide_ws/scripts/align_yaw.py'),
                                           '%.1f' % math.degrees(self.places[name][2])],
                                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
            self.align_timer = self.create_timer(0.3, lambda: self.align_poll(r, name, s))
            return
        if self.quitting:
            if st == GoalStatus.STATUS_SUCCEEDED: self.finish(True)
            else: self.home_failed()
            return
        if st == GoalStatus.STATUS_SUCCEEDED:
            if name == 'home':
                self.status('idle', 'home', 'at home' + (': battery low, please charge me' if self.battery_low() else ''))
            elif self.ret:
                w = float(self.get_parameter('wait_s').value)
                self.status('arrived', name, 'returning home in %d s' % w)
                self.timer = self.create_timer(w, lambda: self.send('home'))
            else:
                self.status('arrived', name, 'staying here')
        else:
            self.status('failed', name, 'navigation status %d' % st)
            if name != 'home':
                self.send('home')

    # ---------- heading alignment (align_yaw.py, non-blocking) ----------
    def stop_align(self):
        if self.align_timer is not None:
            self.align_timer.cancel(); self.destroy_timer(self.align_timer); self.align_timer = None
        if self.align is not None:
            if self.align.poll() is None:
                try: os.killpg(self.align.pid, signal.SIGINT)
                except OSError: pass
            self.align = None

    def align_poll(self, r, name, s):
        if self.align is not None and self.align.poll() is None:
            return
        self.stop_align()
        if s == self.seq:
            self.on_result(r, name, s, aligned=True)

    # ---------- quit ----------
    def home_failed(self):
        self.home_tries += 1
        if self.home_tries < 2:
            self.status('quitting', 'home', 'home not reached, trying again'); self.send('home')
        else:
            self.finish(False)

    def finish(self, at_home):
        self.status('shutdown', 'home', ('at home' if at_home else 'NOT at home: carry the robot to the home mark') +
                    ': stopping the robot software')
        self.timer = self.create_timer(2.0, self.stop_all)  # time for the GUI to receive the last status

    def stop_all(self):
        self.clear_timer()
        if os.environ.get('INVOCATION_ID'):  # started by systemd: a plain exit would be restarted
            subprocess.Popen(['sudo', '-n', 'systemctl', 'stop', 'limo-guide'], start_new_session=True)
        else:
            try:
                ppid = os.getppid()
                if 'guide_start' in open('/proc/%d/cmdline' % ppid).read():
                    os.killpg(os.getpgid(ppid), signal.SIGINT)  # = Ctrl+C in the guide_start.sh window
            except (OSError, ProcessLookupError):
                pass
        self.done = True


def main():
    rclpy.init()
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    node = Guide(args[0] if args else '/home/agilex/guide_ws/places_lab.yaml')
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    try:
        node.destroy_node(); rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
GUIDE_EOF
chmod +x ~/guide_ws/scripts/guide_manager_foxy.py
cat > ~/guide_ws/scripts/loc_check.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/loc_check.py
cat > ~/guide_ws/scripts/map_clean.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# map_clean.py IN.yaml OUT_NAME : fill furniture the lidar only sees as legs (tables, chair groups).
# Small occupied blobs (legs) closer than --link m are grouped; each group with >= --min-legs legs
# is filled solid (its convex hull). Writes OUT_NAME.pgm/.yaml next to IN and OUT_NAME_preview.ppm
# (white free, grey unknown, black walls, red = filled, blue = home). No numpy needed.
import argparse, math, os, yaml


def read_pgm(path):
    with open(path, 'rb') as f:
        assert f.readline().strip() == b'P5', 'not a binary PGM'
        line = f.readline()
        while line.startswith(b'#'): line = f.readline()
        w, h = map(int, line.split()); f.readline()
        return w, h, bytearray(f.read(w * h))


def hull(pts):
    pts = sorted(set(pts))
    if len(pts) < 3: return pts
    cr = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lo, up = [], []
    for p in pts:
        while len(lo) >= 2 and cr(lo[-2], lo[-1], p) <= 0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up) >= 2 and cr(up[-2], up[-1], p) <= 0: up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def inside(poly, x, y):
    n = len(poly)
    for k in range(n):
        (x1, y1), (x2, y2) = poly[k], poly[(k + 1) % n]
        if (x2 - x1) * (y - y1) - (y2 - y1) * (x - x1) < 0: return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('inp'); ap.add_argument('out')
    ap.add_argument('--leg-max', type=float, default=0.03, help='largest leg blob (m2)')
    ap.add_argument('--link', type=float, default=1.3, help='legs closer than this (m) belong together')
    ap.add_argument('--min-legs', type=int, default=3)
    ap.add_argument('--max-area', type=float, default=4.0, help='never fill a group larger than this (m2)')
    ap.add_argument('--keep-clear', type=float, default=0.6, help='free radius kept around home (m)')
    a = ap.parse_args()
    m = yaml.safe_load(open(a.inp)); d = os.path.dirname(os.path.abspath(a.inp))
    w, h, px = read_pgm(os.path.join(d, m['image'])); r = m['resolution']; ox, oy = m['origin'][0], m['origin'][1]
    occ = lambda i, j: px[j * w + i] <= 50
    # connected occupied blobs (8-neighbour)
    seen = bytearray(w * h); legs = []
    for j in range(h):
        for i in range(w):
            if seen[j * w + i] or not occ(i, j): continue
            stack = [(i, j)]; seen[j * w + i] = 1; cells = []
            while stack:
                ci, cj = stack.pop(); cells.append((ci, cj))
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        ni, nj = ci + di, cj + dj
                        if 0 <= ni < w and 0 <= nj < h and not seen[nj * w + ni] and occ(ni, nj):
                            seen[nj * w + ni] = 1; stack.append((ni, nj))
            if len(cells) * r * r <= a.leg_max:
                legs.append(cells)
    # group legs by centroid distance; a group that is too large is split again with a shorter link
    cen = [(sum(c[0] for c in L) / len(L), sum(c[1] for c in L) / len(L)) for L in legs]

    def group(ids, link):
        par = {k: k for k in ids}
        def find(k):
            while par[k] != k: par[k] = par[par[k]]; k = par[k]
            return k
        for x in range(len(ids)):
            for y in range(x + 1, len(ids)):
                p, q = ids[x], ids[y]
                if math.hypot(cen[p][0] - cen[q][0], cen[p][1] - cen[q][1]) <= link / r: par[find(p)] = find(q)
        out = {}
        for k in ids: out.setdefault(find(k), []).append(k)
        return list(out.values())

    # home in pixel coordinates (row 0 = top of the image)
    hi, hj = (0 - ox) / r - 0.5, (h - 1) - ((0 - oy) / r - 0.5)
    filled = set(); n_fill = [0]

    def handle(g, link):
        if len(g) < a.min_legs: return
        corners = [(ci + dx, cj + dy) for k in g for (ci, cj) in legs[k] for dx in (0, 1) for dy in (0, 1)]
        poly = hull(corners)
        area = abs(sum(poly[k][0] * poly[(k + 1) % len(poly)][1] - poly[(k + 1) % len(poly)][0] * poly[k][1]
                       for k in range(len(poly)))) / 2 * r * r
        near_home = min(math.hypot(hi + 0.5 - p[0], hj + 0.5 - p[1]) for p in poly) * r < a.keep_clear \
            or inside(poly, hi + 0.5, hj + 0.5)
        if (area > a.max_area or near_home) and link > 0.5 and len(group(g, link * 0.6)) > 1:
            for sub in group(g, link * 0.6): handle(sub, link * 0.6)
            return
        cx = sum(p[0] for p in poly) / len(poly); cy = sum(p[1] for p in poly) / len(poly)
        wx, wy = ox + cx * r, oy + (h - cy) * r
        xs = [p[0] for p in poly]; ys = [p[1] for p in poly]
        tag = 'group at (%.2f, %.2f) m, %d legs, %.2f x %.2f m, %.2f m2' % (
            wx, wy, len(g), (max(xs) - min(xs)) * r, (max(ys) - min(ys)) * r, area)
        if area > a.max_area: print('  skip (too large)    ' + tag); return
        if near_home: print('  skip (near home)     ' + tag); return
        for j in range(int(min(ys)), int(math.ceil(max(ys)))):
            for i in range(int(min(xs)), int(math.ceil(max(xs)))):
                if inside(poly, i + 0.5, j + 0.5) and px[j * w + i] != 0:
                    filled.add(j * w + i)
        n_fill[0] += 1; print('  filled               ' + tag)

    groups = group(list(range(len(legs))), a.link)
    print('%d leg-sized blobs, %d groups' % (len(legs), len(groups)))
    for g in groups: handle(g, a.link)
    out = bytearray(px)
    for k in filled: out[k] = 0
    base = os.path.join(d, a.out)
    with open(base + '.pgm', 'wb') as f:
        f.write(b'P5\n# map_clean.py\n%d %d\n255\n' % (w, h)); f.write(bytes(out))
    m2 = dict(m); m2['image'] = a.out + '.pgm'
    yaml.safe_dump(m2, open(base + '.yaml', 'w'), default_flow_style=None, sort_keys=False)
    rgb = bytearray()
    for k, v in enumerate(px):
        rgb += b'\xdc\x28\x28' if k in filled else (b'\xff\xff\xff' if v >= 250 else (b'\x00\x00\x00' if v <= 50 else b'\xb4\xb4\xb4'))
    for di in range(-3, 4):  # blue cross at home
        for (i, j) in ((int(hi + 0.5) + di, int(hj + 0.5)), (int(hi + 0.5), int(hj + 0.5) + di)):
            if 0 <= i < w and 0 <= j < h: rgb[3 * (j * w + i):3 * (j * w + i) + 3] = b'\x28\x50\xdc'
    with open(base + '_preview.ppm', 'wb') as f:
        f.write(b'P6\n%d %d\n255\n' % (w, h)); f.write(bytes(rgb))
    print('filled %d groups, %.2f m2 -> %s.pgm/.yaml, preview %s_preview.ppm' % (n_fill[0], len(filled) * r * r, base, base))


if __name__ == '__main__':
    main()
GUIDE_EOF
chmod +x ~/guide_ws/scripts/map_clean.py
cat > ~/guide_ws/scripts/nav_test.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/nav_test.py
cat > ~/guide_ws/scripts/once.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/once.py
cat > ~/guide_ws/scripts/place_click.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# place_click.py : record guide places by clicking in laptop RViz2 (no driving needed)
#   RViz2 tool "Publish Point": click the spot, then click a second point in the direction the
#   robot should face on arrival, then type the name here.  Ctrl+C ends.
#   place_click.py --here NAME    save the robot's current pose instead
#   place_click.py --list         show the places        place_click.py --delete NAME
import argparse, math, os, sys, yaml
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener

FILE = os.path.expanduser('~/guide_ws/places_lab.yaml')


def load():
    if not os.path.exists(FILE): return {'home': [0.0, 0.0, 0.0]}
    return (yaml.safe_load(open(FILE)) or {}).get('places') or {'home': [0.0, 0.0, 0.0]}


def save(pl):
    pl = dict(pl); pl.setdefault('home', [0.0, 0.0, 0.0])
    with open(FILE, 'w') as f:
        f.write('places:\n')
        for k in ['home'] + sorted(x for x in pl if x != 'home'):
            x, y, a = pl[k]; f.write('  %s: [%.2f, %.2f, %.2f]\n' % (k, x, y, a))


def clearance(m, x, y):
    # distance (m) from (x, y) to the nearest occupied or unknown cell, searched up to 1 m
    r = m.info.resolution; w, h = m.info.width, m.info.height
    i0 = int((x - m.info.origin.position.x) / r); j0 = int((y - m.info.origin.position.y) / r)
    if not (0 <= i0 < w and 0 <= j0 < h): return 0.0
    best = 1.0; k = int(1.0 / r)
    for j in range(max(0, j0 - k), min(h, j0 + k + 1)):
        for i in range(max(0, i0 - k), min(w, i0 + k + 1)):
            v = m.data[j * w + i]
            if v >= 65 or v < 0: best = min(best, math.hypot(i - i0, j - j0) * r)
    return best


def clean(name):
    return ''.join(c for c in name.strip().lower().replace(' ', '_') if c.isalnum() or c == '_')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--here'); ap.add_argument('--delete'); ap.add_argument('--list', action='store_true')
    a = ap.parse_args(); pl = load()
    if a.list:
        for k, v in pl.items(): print('  %-18s x %6.2f  y %6.2f  heading %4.0f deg' % (k, v[0], v[1], math.degrees(v[2])))
        return
    if a.delete:
        if clean(a.delete) == 'home': sys.exit('home cannot be deleted')
        pl.pop(clean(a.delete), None); save(pl); print('deleted', clean(a.delete)); return
    rclpy.init(); n = rclpy.create_node('place_click'); clicks = []; mp = []
    q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(OccupancyGrid, '/map', lambda m: mp.append(m), q)
    n.create_subscription(PointStamped, '/clicked_point', lambda m: clicks.append((m.point.x, m.point.y)), 10)
    buf = Buffer(); n.tl = TransformListener(buf, n)
    if a.here:
        t = None
        for _ in range(50):
            rclpy.spin_once(n, timeout_sec=0.1)
            try:
                t = buf.lookup_transform('map', 'base_link', Time()); break
            except Exception:
                pass
        if t is None: sys.exit('no map->base_link transform: is localization (nav_start.sh) running?')
        tr = t.transform.translation; q2 = t.transform.rotation
        yaw = math.atan2(2 * (q2.w * q2.z + q2.x * q2.y), 1 - 2 * (q2.y * q2.y + q2.z * q2.z))
        pl[clean(a.here)] = [tr.x, tr.y, yaw]; save(pl)
        print('saved %s: [%.2f, %.2f, %.2f]' % (clean(a.here), tr.x, tr.y, yaw)); return
    print('RViz2 "Publish Point": click the place, then click the direction to face. Ctrl+C to finish.')
    try:
        while True:
            clicks.clear(); print('waiting for 2 clicks ...', flush=True)
            while len(clicks) < 2: rclpy.spin_once(n, timeout_sec=0.1)
            (x, y), (x2, y2) = clicks[0], clicks[1]; yaw = math.atan2(y2 - y, x2 - x)
            cl = clearance(mp[-1], x, y) if mp else None
            print('spot (%.2f, %.2f), facing %.0f deg, clearance %s' % (
                x, y, math.degrees(yaw), '%.2f m' % cl if cl is not None else '? (no /map)'))
            if cl is not None and cl < 0.30:
                print('  WARNING: less than 0.30 m from an obstacle: the robot (0.38 x 0.28 m) may not reach it')
            name = clean(input('name (Enter = discard): '))
            if not name: print('  discarded'); continue
            if name in pl and input('  %s exists: replace? [y/N] ' % name).lower() != 'y': continue
            pl[name] = [x, y, yaw]; save(pl); print('  saved %s: [%.2f, %.2f, %.2f]' % (name, x, y, yaw))
    except (KeyboardInterrupt, EOFError):
        print('\n%d places in %s' % (len(load()), FILE))


if __name__ == '__main__':
    main()
GUIDE_EOF
chmod +x ~/guide_ws/scripts/place_click.py
cat > ~/guide_ws/scripts/preflight.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# preflight.py : 10-second health check before any session (needs limo_start_guide or a stack running)
# Prints PASS / WARN / FAIL per item; exit code 1 if anything FAILs.
import glob, os, shutil, subprocess, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
try:
    from limo_msgs.msg import LimoStatus
except ImportError:
    LimoStatus = None

res = []


def report(level, item, detail):
    res.append(level); print('%-4s  %-22s %s' % (level, item, detail), flush=True)


def main():
    e = os.environ
    report('PASS' if e.get('ROS_DOMAIN_ID') == '30' else 'FAIL', 'ROS_DOMAIN_ID', '%s (need 30: work inside tmux guide)' % e.get('ROS_DOMAIN_ID'))
    report('PASS' if e.get('RMW_IMPLEMENTATION') == 'rmw_cyclonedds_cpp' else 'FAIL', 'RMW', str(e.get('RMW_IMPLEMENTATION')))
    try:
        ip = subprocess.run(['ip', '-4', '-o', 'addr', 'show', 'wlan0'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            universal_newlines=True).stdout.split()
        report('PASS', 'wlan0 address', ip[ip.index('inet') + 1] if 'inet' in ip else 'none')
    except Exception:
        report('WARN', 'wlan0 address', 'unknown')
    try:
        sync = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                              stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, universal_newlines=True).stdout.strip()
        report('PASS' if sync == 'yes' else 'WARN', 'clock synchronized', sync or '?')
    except Exception:
        report('WARN', 'clock synchronized', '?')
    free = shutil.disk_usage(os.path.expanduser('~')).free / 1e9
    report('PASS' if free > 5 else ('WARN' if free > 2 else 'FAIL'), 'disk free', '%.1f GB' % free)
    temps = []
    for z in glob.glob('/sys/class/thermal/thermal_zone*/temp'):
        try: temps.append(int(open(z).read()) / 1000.0)
        except Exception: pass
    if temps:
        t = max(temps); report('PASS' if t < 70 else ('WARN' if t < 85 else 'FAIL'), 'max temperature', '%.0f C' % t)

    rclpy.init(); n = rclpy.create_node('preflight'); cnt = {'scan': 0, 'odom': 0, 'imu': 0}; st = []
    n.create_subscription(LaserScan, '/scan', lambda m: cnt.__setitem__('scan', cnt['scan'] + 1), qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odom', lambda m: cnt.__setitem__('odom', cnt['odom'] + 1), 10)
    n.create_subscription(Imu, '/imu', lambda m: cnt.__setitem__('imu', cnt['imu'] + 1), qos_profile_sensor_data)
    if LimoStatus is not None:
        n.create_subscription(LimoStatus, '/limo_status', lambda m: st.append(m), 10)
    buf = Buffer(); n.tl = TransformListener(buf, n)
    t0 = time.time()
    while time.time() - t0 < 4.0: rclpy.spin_once(n, timeout_sec=0.05)
    dt = time.time() - t0
    for k, need in (('scan', 8), ('odom', 40), ('imu', 80)):
        hz = cnt[k] / dt
        report('PASS' if hz >= need else ('WARN' if hz > 0 else 'FAIL'), '/%s rate' % k, '%.1f Hz (need >= %d)' % (hz, need))
    if st:
        m = st[-1]; v = m.battery_voltage
        report('PASS' if v >= 11.5 else ('WARN' if v >= 11.0 else 'FAIL'), 'battery', '%.2f V (>= 11.5 to start a session)' % v)
        report('PASS' if m.error_code == 0 else 'FAIL', 'chassis error_code', str(m.error_code))
        report('PASS' if m.motion_mode == 0 else 'FAIL', 'motion_mode', '%d (0 = differential)' % m.motion_mode)
    else:
        report('FAIL', '/limo_status', 'no message: base not running?')
    for parent, child in (('odom', 'base_link'), ('base_link', 'laser_link')):
        try:
            tr = buf.lookup_transform(parent, child, Time()).transform.translation
            note = '' if child != 'laser_link' else ('  (x should be 0.09: use limo_start_guide)' if abs(tr.x - 0.09) > 0.01 else '')
            report('PASS' if not note else 'FAIL', 'TF %s->%s' % (parent, child), 'x %.3f y %.3f%s' % (tr.x, tr.y, note))
        except Exception:
            report('FAIL', 'TF %s->%s' % (parent, child), 'missing')
    n.destroy_node(); rclpy.shutdown()
    bad = res.count('FAIL'); warn = res.count('WARN')
    print('\nPREFLIGHT: %s (%d fail, %d warn)' % ('OK' if bad == 0 else 'NOT OK', bad, warn))
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
GUIDE_EOF
chmod +x ~/guide_ws/scripts/preflight.py
cat > ~/guide_ws/scripts/scan_check.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/scan_check.py
cat > ~/guide_ws/scripts/trials.py <<'GUIDE_EOF'
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
    ap.add_argument('--min-volt', type=float, default=10.8, help='stop the block below this battery voltage')
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
    gate = len(rows) >= 12 and len(ok) >= need and park <= 0.25
    print('GATE (>= 12 trips, >= 90 %% arrived, parking <= 0.25 m; collisions you log in notes): %s'
          % ('PASS' if gate else ('NOT YET (%d trips)' % len(rows) if len(rows) < 30 else 'FAIL')))
    if bag is not None: print('rosbag: %s' % os.path.join(WS, 'bags', 'trials_' + ts))
    return 0 if gate else 1


if __name__ == '__main__':
    sys.exit(main())
GUIDE_EOF
chmod +x ~/guide_ws/scripts/trials.py
cat > ~/guide_ws/scripts/viz_relay.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# viz_relay.py : for Lichtblick over rosbridge (Foxy) when foxglove_bridge is not installed.
# /scan (best effort) -> /scan_viz (reliable); /map (latched) -> /map_viz every 2 s.
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
rclpy.init(); n = rclpy.create_node('viz_relay'); last = {}
ps = n.create_publisher(LaserScan, '/scan_viz', 10)
pm = n.create_publisher(OccupancyGrid, '/map_viz', 1)
n.create_subscription(LaserScan, '/scan', lambda m: ps.publish(m), qos_profile_sensor_data)
q = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
n.create_subscription(OccupancyGrid, '/map', lambda m: last.__setitem__('map', m), q)
n.create_timer(2.0, lambda: pm.publish(last['map']) if 'map' in last else None)
try:
    rclpy.spin(n)
except KeyboardInterrupt:
    pass
GUIDE_EOF
chmod +x ~/guide_ws/scripts/viz_relay.py
cat > ~/guide_ws/scripts/wall_check.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# wall_check.py : no driving. Shows how flat the wall ahead looks to the lidar (+-25 deg, < 4 m).
import os, sys, math, statistics, rclpy
sys.path.insert(0, os.path.expanduser('~/guide_ws/scripts'))
from calib import Calib
rclpy.init(); c = Calib(); c.wait(1.5)
for k in range(5):
    c.scan = None
    while c.scan is None: rclpy.spin_once(c.n, timeout_sec=0.1)
    P = c.points()
    if len(P) < 15:
        print('only %d points: no wall ahead' % len(P)); c.wait(0.5); continue
    mx = statistics.mean(p[0] for p in P); my = statistics.mean(p[1] for p in P)
    a = sum((p[0] - mx) * (p[1] - my) for p in P) / sum((p[1] - my) ** 2 for p in P); b = mx - a * my
    psi = math.atan(a); res = [(p[0] - a * p[1] - b) * math.cos(psi) for p in P]
    rms = math.sqrt(sum(r * r for r in res) / len(res)); worst = max(abs(r) for r in res)
    print('points %3d  wall %.3f m  heading %+5.1f deg  rms %.3f m  worst %.3f m  %s'
          % (len(P), b * math.cos(psi), math.degrees(psi), rms, worst,
             'GOOD' if rms < 0.015 and worst < 0.04 else 'NOT FLAT'))
    c.wait(0.5)
GUIDE_EOF
chmod +x ~/guide_ws/scripts/wall_check.py
cat > ~/guide_ws/scripts/yaw_test.py <<'GUIDE_EOF'
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
GUIDE_EOF
chmod +x ~/guide_ws/scripts/yaw_test.py
cat > ~/guide_ws/scripts/yset.py <<'GUIDE_EOF'
#!/usr/bin/env python3
# yset.py FILE a.b.c=value ...   value 's:text' = keep as string, '__del__' = delete key
import sys, yaml
f = sys.argv[1]
d = yaml.safe_load(open(f))
for a in sys.argv[2:]:
    k, v = a.split('=', 1)
    parts = k.split('.')
    cur = d
    for p in parts[:-1]:
        cur = cur.setdefault(p, {})
    if v == '__del__':
        cur.pop(parts[-1], None)
    elif v.startswith('s:'):
        cur[parts[-1]] = v[2:]
    else:
        cur[parts[-1]] = yaml.safe_load(v)
yaml.safe_dump(d, open(f, 'w'), default_flow_style=None, sort_keys=False)
print('updated', f)
GUIDE_EOF
chmod +x ~/guide_ws/scripts/yset.py
cat > ~/guide_ws/scripts/auto_map.sh <<'GUIDE_EOF'
#!/bin/bash
# One-command autonomous mapping: base + lidar, SLAM, Nav2, then auto_map.py.
# Robot on the home mark, doors closed. Ctrl+C stops everything. Options go to auto_map.py (e.g. --max-min 3).
source ~/guide_env.sh
L=$HOME/guide_ws/logs; T=$(date +%Y%m%d_%H%M); mkdir -p $L
if pgrep -f "limo_base|slam_toolbox|bt_navigator" > /dev/null; then
  echo "Robot programs are already running (another tmux window?). Stop them first."; exit 1
fi
trap 'pkill -INT -f "lib/explore_lite/explore"; kill 0' EXIT
echo "[$T] starting base + lidar, SLAM, Nav2 (logs: $L/am_${T}_*.log)"
ros2 launch $HOME/guide_ws/launch/limo_start_guide.launch.py > $L/am_${T}_limo.log 2>&1 &
sleep 6
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file $HOME/guide_ws/params/slam_lab.yaml > $L/am_${T}_slam.log 2>&1 &
sleep 4
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False params_file:=$HOME/guide_ws/params/nav2_lab_foxy.yaml > $L/am_${T}_nav.log 2>&1 &
python3 $HOME/guide_ws/scripts/auto_map.py --tag $T "$@"
GUIDE_EOF
chmod +x ~/guide_ws/scripts/auto_map.sh
cat > ~/guide_ws/scripts/bridge.sh <<'GUIDE_EOF'
#!/bin/bash
# bridge.sh : websocket bridges for the Windows laptop. Ctrl+C stops them.
#   rosbridge :9090       -> guide GUI (lab_gui_win.py)
#   foxglove_bridge :8765 -> Lichtblick; without it, viz_relay.py makes /scan_viz and /map_viz for Lichtblick over rosbridge
source ~/guide_env.sh
L=$HOME/guide_ws/logs; mkdir -p $L
if pgrep -f rosbridge_websocket > /dev/null; then echo "bridges already running"; exit 0; fi
IP=$(ip -4 -o addr show wlan0 | awk '{print $4}' | cut -d/ -f1)
trap 'kill $(jobs -p) 2>/dev/null' EXIT
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > $L/rosbridge.log 2>&1 &
if ros2 pkg prefix foxglove_bridge > /dev/null 2>&1; then
  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765 > $L/foxglove.log 2>&1 &
  echo "bridges up: GUI ws://$IP:9090, Lichtblick Foxglove WebSocket ws://$IP:8765"
else
  python3 $HOME/guide_ws/scripts/viz_relay.py > $L/viz_relay.log 2>&1 &
  echo "bridges up: GUI ws://$IP:9090, Lichtblick Rosbridge ws://$IP:9090 (topics /scan_viz, /map_viz)"
fi
wait
GUIDE_EOF
chmod +x ~/guide_ws/scripts/bridge.sh
cat > ~/guide_ws/scripts/goto.sh <<'GUIDE_EOF'
#!/bin/bash
# usage: goto.sh PLACE_NAME
source ~/guide_env.sh
read X Y W Z < <(python3 -c "import yaml,math,sys; p=yaml.safe_load(open('$HOME/guide_ws/places_lab.yaml'))['places']['$1']; print(p[0],p[1],math.cos(p[2]/2),math.sin(p[2]/2))") || exit 1
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y}, orientation: {z: $Z, w: $W}}}}"
GUIDE_EOF
chmod +x ~/guide_ws/scripts/goto.sh
cat > ~/guide_ws/scripts/guide_start.sh <<'GUIDE_EOF'
#!/bin/bash
# base + lidar + AMCL + Nav2 + laptop bridges (4.6), localization check, then the guide manager. Robot on the home mark.
# Ctrl+C stops everything. The limo-guide service (9.5) runs this same script at power-on.
source ~/guide_env.sh
trap 'kill 0' EXIT
~/guide_ws/scripts/nav_start.sh &
~/guide_ws/scripts/bridge.sh > ~/guide_ws/logs/bridge_start.log 2>&1 &
sleep 25
python3 ~/guide_ws/scripts/loc_check.py --fix || echo "WARNING: localization doubtful: check Lichtblick before guiding"
python3 ~/guide_ws/scripts/guide_manager_foxy.py ~/guide_ws/places_lab.yaml
GUIDE_EOF
chmod +x ~/guide_ws/scripts/guide_start.sh
cat > ~/guide_ws/scripts/install_autostart.sh <<'GUIDE_EOF'
#!/bin/bash
# install_autostart.sh : service limo-guide = guide_start.sh at power-on (robot on the home mark).
# Installs only; switch on with: sudo systemctl enable limo-guide
set -e
sudo tee /etc/systemd/system/limo-guide.service > /dev/null <<'UNIT'
[Unit]
Description=Limo lab guide robot (base, AMCL, Nav2, guide manager)
After=network-online.target
Wants=network-online.target

[Service]
User=agilex
Environment=HOME=/home/agilex PYTHONUNBUFFERED=1
ExecStartPre=/bin/bash /home/agilex/guide_ws/scripts/wait_wifi.sh
ExecStart=/bin/bash /home/agilex/guide_ws/scripts/guide_start.sh
KillSignal=SIGINT
TimeoutStopSec=20
Restart=on-failure
RestartSec=15

[Install]
WantedBy=multi-user.target
UNIT
C=""; for b in /usr/bin/systemctl /bin/systemctl; do for a in start stop restart; do C="$C${C:+, }$b $a limo-guide"; done; done
echo "agilex ALL=(root) NOPASSWD: $C" | sudo tee /etc/sudoers.d/limo-guide > /dev/null
sudo chmod 440 /etc/sudoers.d/limo-guide
sudo visudo -cf /etc/sudoers.d/limo-guide
sudo systemctl daemon-reload
echo "limo-guide installed (not enabled). Start now: sudo systemctl start limo-guide   At boot: sudo systemctl enable limo-guide"
GUIDE_EOF
chmod +x ~/guide_ws/scripts/install_autostart.sh
cat > ~/guide_ws/scripts/make_cyclone.sh <<'GUIDE_EOF'
#!/bin/bash
# usage: make_cyclone.sh [LAPTOP_IP]
[ -n "$1" ] && echo "$1" > ~/guide_ws/laptop_ip
P=$(cat ~/guide_ws/laptop_ip 2>/dev/null)
PEER=""; [ -n "$P" ] && PEER="<Peers><Peer address=\"$P\"/></Peers>"
cat > ~/guide_ws/cyclone.xml <<XML
<CycloneDDS><Domain id="any">
<General><NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress></General>
<Discovery>$PEER<ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>
</Domain></CycloneDDS>
XML
echo "cyclone.xml written (laptop peer: ${P:-none})"
GUIDE_EOF
chmod +x ~/guide_ws/scripts/make_cyclone.sh
cat > ~/guide_ws/scripts/nav_start.sh <<'GUIDE_EOF'
#!/bin/bash
# base + lidar + AMCL on lab_v1 + Nav2, logs in ~/guide_ws/logs; Ctrl+C stops all
source ~/guide_env.sh
P=$HOME/guide_ws/params/nav2_lab_foxy.yaml; M=$HOME/guide_ws/maps/lab_v1.yaml; L=$HOME/guide_ws/logs
trap 'kill 0' EXIT
ros2 launch $HOME/guide_ws/launch/limo_start_guide.launch.py > $L/limo.log 2>&1 &
sleep 5
ros2 launch nav2_bringup localization_launch.py use_sim_time:=False map:=$M params_file:=$P > $L/loc.log 2>&1 &
sleep 5
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=False map_subscribe_transient_local:=true params_file:=$P > $L/nav.log 2>&1 &
sleep 12
bash ~/guide_ws/scripts/set_home_pose.sh
echo "NAV READY (logs: $L)"; wait
GUIDE_EOF
chmod +x ~/guide_ws/scripts/nav_start.sh
cat > ~/guide_ws/scripts/set_home_pose.sh <<'GUIDE_EOF'
#!/bin/bash
source ~/guide_env.sh
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}, covariance: [0.04,0.0,0.0,0.0,0.0,0.0, 0.0,0.04,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.03]}}"
GUIDE_EOF
chmod +x ~/guide_ws/scripts/set_home_pose.sh
cat > ~/guide_ws/scripts/wait_wifi.sh <<'GUIDE_EOF'
#!/bin/bash
# wait up to 120 s for wlan0 to get an IPv4 address (Cyclone DDS is pinned to wlan0)
for i in $(seq 60); do ip -4 addr show wlan0 | grep -q inet && exit 0; sleep 2; done
exit 0
GUIDE_EOF
chmod +x ~/guide_ws/scripts/wait_wifi.sh

echo "== launch files"
cat > ~/guide_ws/launch/limo_start_guide.launch.py <<'GUIDE_EOF'
# limo_start_guide.launch.py (EKF version): wheel launch with pub_odom_tf false + robot_localization EKF.
# The EKF publishes odom->base_link from wheel speed (vx, vy) + IMU yaw rate. Wheel-only original: limo_start_wheel.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

H = os.path.expanduser('~/guide_ws')


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(H + '/launch/limo_start_wheel.launch.py'),
            launch_arguments={'pub_odom_tf': 'false'}.items()),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             output='screen', parameters=[H + '/params/ekf_lab.yaml']),
    ])
GUIDE_EOF
cat > ~/guide_ws/launch/limo_start_wheel.launch.py <<'GUIDE_EOF'
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rf2o_dir = get_package_share_directory('rf2o_laser_odometry')
    ydlidar_ros_dir = get_package_share_directory("ydlidar_ros2_driver")
    limo_base_dir = get_package_share_directory('limo_base')
    # limo_bringup_dir = get_package_share_directory('limo_bringup')
    # limo_gazebo = get_package_share_directory('limo_car')


    pub_tf=Node(
        package="limo_base",
        executable="tf_pub",   
        output='screen',
        name='tf_pub_node',
    )
    base_link_to_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ydlidar',
        arguments=['0.09','0','0.18','0','0','0','1','base_link','laser_link']
    )
    base_link_to_camera_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ydlidar',
        arguments=['0.18','0','0.18','0','0','0','1','base_link','camera_link']
    )
    base_link_to_imu_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ydlidar',
        arguments=['0','0','0','0','0','0','1','base_link','imu_link']
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([limo_base_dir,'/launch','/limo_base.launch.py']),
        ),  
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([rf2o_dir,'/launch','/rf2o_laser_odometry.launch.py']),
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ydlidar_ros_dir,'/launch','/ydlidar_launch.py']),
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([limo_bringup_dir,'/launch/humble','/ekf_odom.launch.py']),
        # ), 
        # pub_tf
        base_link_to_laser_tf_node,
        base_link_to_camera_tf_node,
        base_link_to_imu_tf_node
    ])
GUIDE_EOF

echo "== parameter files (kept if present; --force-params overwrites)"
if [ $FORCE = 1 ] || [ ! -f ~/guide_ws/params/ekf_lab.yaml ]; then
cat > ~/guide_ws/params/ekf_lab.yaml <<'GUIDE_EOF'
ekf_filter_node:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: true
    publish_tf: true
    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom
    odom0: /odom
    odom0_config: [false, false, false,  false, false, false,
                   true,  true,  false,  false, false, false,
                   false, false, false]
    imu0: /imu
    imu0_config: [false, false, false,  false, false, false,
                  false, false, false,  false, false, true,
                  false, false, false]
    imu0_remove_gravitational_acceleration: true
GUIDE_EOF
fi
if [ $FORCE = 1 ] || [ ! -f ~/guide_ws/params/explore_lab.yaml ]; then
cat > ~/guide_ws/params/explore_lab.yaml <<'GUIDE_EOF'
/**:
  ros__parameters: {robot_base_frame: base_link, return_to_init: false, costmap_topic: map,
    costmap_updates_topic: map_updates, visualize: true, planner_frequency: 0.5, progress_timeout: 30.0,
    potential_scale: 3.0, orientation_scale: 0.0, gain_scale: 1.0, transform_tolerance: 0.5,
    min_frontier_size: 0.15, use_sim_time: false}
GUIDE_EOF
fi
if [ $FORCE = 1 ] || [ ! -f ~/guide_ws/params/nav2_lab_foxy.yaml ]; then
cat > ~/guide_ws/params/nav2_lab_foxy.yaml <<'GUIDE_EOF'
amcl:
  ros__parameters: {use_sim_time: false, alpha1: 0.2, alpha2: 0.2, alpha3: 0.2, alpha4: 0.2,
    alpha5: 0.2, base_frame_id: base_link, beam_skip_distance: 0.5, beam_skip_error_threshold: 0.9,
    beam_skip_threshold: 0.3, do_beamskip: false, global_frame_id: map, lambda_short: 0.1,
    laser_likelihood_max_dist: 2.0, laser_max_range: 8.0, laser_min_range: -1.0, laser_model_type: likelihood_field,
    max_beams: 60, max_particles: 2000, min_particles: 500, odom_frame_id: odom, pf_err: 0.05,
    pf_z: 0.99, recovery_alpha_fast: 0.0, recovery_alpha_slow: 0.0, resample_interval: 1,
    robot_model_type: differential, save_pose_rate: 0.5, sigma_hit: 0.2, tf_broadcast: true,
    transform_tolerance: 1.0, update_min_a: 0.15, update_min_d: 0.1, z_hit: 0.5, z_max: 0.05,
    z_rand: 0.5, z_short: 0.05, scan_topic: scan}
amcl_map_client:
  ros__parameters: {use_sim_time: false}
amcl_rclcpp_node:
  ros__parameters: {use_sim_time: false}
bt_navigator:
  ros__parameters:
    use_sim_time: false
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    enable_groot_monitoring: true
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    default_bt_xml_filename: navigate_w_replanning_and_recovery.xml
    plugin_lib_names: [nav2_compute_path_to_pose_action_bt_node, nav2_follow_path_action_bt_node,
      nav2_back_up_action_bt_node, nav2_spin_action_bt_node, nav2_wait_action_bt_node,
      nav2_clear_costmap_service_bt_node, nav2_is_stuck_condition_bt_node, nav2_goal_reached_condition_bt_node,
      nav2_goal_updated_condition_bt_node, nav2_initial_pose_received_condition_bt_node,
      nav2_reinitialize_global_localization_service_bt_node, nav2_rate_controller_bt_node,
      nav2_distance_controller_bt_node, nav2_speed_controller_bt_node, nav2_truncate_path_action_bt_node,
      nav2_goal_updater_node_bt_node, nav2_recovery_node_bt_node, nav2_pipeline_sequence_bt_node,
      nav2_round_robin_node_bt_node, nav2_transform_available_condition_bt_node, nav2_time_expired_condition_bt_node,
      nav2_distance_traveled_condition_bt_node]
bt_navigator_rclcpp_node:
  ros__parameters: {use_sim_time: false}
controller_server:
  ros__parameters:
    use_sim_time: false
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: progress_checker
    goal_checker_plugin: goal_checker
    controller_plugins: [FollowPath]
    progress_checker: {plugin: 'nav2_controller::SimpleProgressChecker', required_movement_radius: 0.2,
      movement_time_allowance: 30.0}
    goal_checker: {plugin: 'nav2_controller::SimpleGoalChecker', xy_goal_tolerance: 0.25,
      yaw_goal_tolerance: 0.30, stateful: false}
    FollowPath: {plugin: 'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController',
      desired_linear_vel: 0.15, goal_dist_tol: 0.20, lookahead_dist: 0.3, min_lookahead_dist: 0.25, max_lookahead_dist: 0.45,
      use_velocity_scaled_lookahead_dist: false, use_rotate_to_heading: true, rotate_to_heading_angular_vel: 0.3,
      rotate_to_heading_min_angle: 0.785, max_angular_accel: 0.8, transform_tolerance: 0.2,
      use_regulated_linear_velocity_scaling: true}
controller_server_rclcpp_node:
  ros__parameters: {use_sim_time: false}
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: false
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      plugins: [voxel_layer, inflation_layer]
      inflation_layer: {plugin: 'nav2_costmap_2d::InflationLayer', cost_scaling_factor: 4.0,
        inflation_radius: 0.35}
      voxel_layer:
        plugin: nav2_costmap_2d::VoxelLayer
        enabled: true
        publish_voxel_map: true
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan: {topic: /scan, max_obstacle_height: 2.0, clearing: true, marking: true,
          data_type: LaserScan}
      static_layer: {map_subscribe_transient_local: true}
      always_send_full_costmap: true
      footprint: '[[0.19,0.14],[0.19,-0.14],[-0.19,-0.14],[-0.19,0.14]]'
  local_costmap_client:
    ros__parameters: {use_sim_time: false}
  local_costmap_rclcpp_node:
    ros__parameters: {use_sim_time: false}
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: false
      resolution: 0.05
      track_unknown_space: true
      plugins: [static_layer, obstacle_layer, inflation_layer]
      obstacle_layer:
        plugin: nav2_costmap_2d::ObstacleLayer
        enabled: true
        observation_sources: scan
        scan: {topic: /scan, max_obstacle_height: 2.0, clearing: true, marking: true,
          data_type: LaserScan}
      static_layer: {plugin: 'nav2_costmap_2d::StaticLayer', map_subscribe_transient_local: true}
      inflation_layer: {plugin: 'nav2_costmap_2d::InflationLayer', cost_scaling_factor: 4.0,
        inflation_radius: 0.35}
      always_send_full_costmap: true
      footprint: '[[0.19,0.14],[0.19,-0.14],[-0.19,-0.14],[-0.19,0.14]]'
  global_costmap_client:
    ros__parameters: {use_sim_time: false}
  global_costmap_rclcpp_node:
    ros__parameters: {use_sim_time: false}
map_server:
  ros__parameters: {use_sim_time: false, yaml_filename: turtlebot3_world.yaml}
map_saver:
  ros__parameters: {use_sim_time: false, save_map_timeout: 5000, free_thresh_default: 0.25,
    occupied_thresh_default: 0.65, map_subscribe_transient_local: false}
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    use_sim_time: false
    planner_plugins: [GridBased]
    GridBased: {plugin: nav2_navfn_planner/NavfnPlanner, tolerance: 0.5, use_astar: false,
      allow_unknown: true}
planner_server_rclcpp_node:
  ros__parameters: {use_sim_time: false}
recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: [spin, back_up, wait]
    spin: {plugin: nav2_recoveries/Spin}
    back_up: {plugin: nav2_recoveries/BackUp}
    wait: {plugin: nav2_recoveries/Wait}
    global_frame: odom
    robot_base_frame: base_link
    transform_timeout: 0.1
    use_sim_time: false
    simulate_ahead_time: 2.0
    max_rotational_vel: 0.3
    min_rotational_vel: 0.15
    rotational_acc_lim: 0.8
robot_state_publisher:
  ros__parameters: {use_sim_time: false}
GUIDE_EOF
fi
if [ $FORCE = 1 ] || [ ! -f ~/guide_ws/params/robot_facts.txt ]; then
cat > ~/guide_ws/params/robot_facts.txt <<'GUIDE_EOF'
footprint: [[0.19, 0.14], [0.19, -0.14], [-0.19, -0.14], [-0.19, 0.14]]   # 0.322x0.220 spec + 0.03 margin
laser_tf: x 0.090 y 0.000 z 0.180 (wall test 26 Oct: front range 0.367 m at 0.297 m gap; factory 0.18 wrong)
scan: fov +-110 deg (220 deg), 247 pts, 10 Hz; use max_laser_range 8.0
odom 50 Hz, imu 100 Hz, motion_mode 0, self-hits none
battery: 10.4 V on 26 Oct (charge before Stage 3)
GUIDE_EOF
fi
if [ $FORCE = 1 ] || [ ! -f ~/guide_ws/params/slam_lab.yaml ]; then
cat > ~/guide_ws/params/slam_lab.yaml <<'GUIDE_EOF'
slam_toolbox:
  ros__parameters: {solver_plugin: 'solver_plugins::CeresSolver', ceres_linear_solver: SPARSE_NORMAL_CHOLESKY,
    ceres_preconditioner: SCHUR_JACOBI, ceres_trust_strategy: LEVENBERG_MARQUARDT,
    ceres_dogleg_type: TRADITIONAL_DOGLEG, ceres_loss_function: None, odom_frame: odom,
    map_frame: map, base_frame: base_link, scan_topic: /scan, mode: mapping, debug_logging: false,
    throttle_scans: 1, transform_publish_period: 0.02, map_update_interval: 2.0, resolution: 0.05,
    max_laser_range: 8.0, minimum_time_interval: 0.5, transform_timeout: 0.2, tf_buffer_duration: 30.0,
    stack_size_to_use: 40000000, enable_interactive_mode: true, use_scan_matching: true,
    use_scan_barycenter: true, minimum_travel_distance: 0.2, minimum_travel_heading: 0.2,
    scan_buffer_size: 10, scan_buffer_maximum_scan_distance: 10.0, link_match_minimum_response_fine: 0.1,
    link_scan_maximum_distance: 1.5, loop_search_maximum_distance: 3.0, do_loop_closing: true,
    loop_match_minimum_chain_size: 10, loop_match_maximum_variance_coarse: 3.0, loop_match_minimum_response_coarse: 0.35,
    loop_match_minimum_response_fine: 0.45, correlation_search_space_dimension: 0.5,
    correlation_search_space_resolution: 0.01, correlation_search_space_smear_deviation: 0.1,
    loop_search_space_dimension: 8.0, loop_search_space_resolution: 0.05, loop_search_space_smear_deviation: 0.03,
    distance_variance_penalty: 0.5, angle_variance_penalty: 1.0, fine_search_angle_offset: 0.00349,
    coarse_search_angle_offset: 0.349, coarse_angle_resolution: 0.0349, minimum_angle_penalty: 0.9,
    minimum_distance_penalty: 0.5, use_response_expansion: true, use_sim_time: false}
GUIDE_EOF
fi
if [ $FORCE = 1 ] || [ ! -f ~/guide_ws/places_lab.yaml ]; then
cat > ~/guide_ws/places_lab.yaml <<'GUIDE_EOF'
places:
  home: [0.0, 0.0, 0.0]
  auditorium: [-0.1, 1.7, 3.14]
  cafeteria: [1.45, 0.35, 0.0]
  library: [1.2, 1.75, 0.0]
GUIDE_EOF
fi

echo "== 9.5 limo-guide service (installed, not enabled; asks the sudo password)"
~/guide_ws/scripts/install_autostart.sh

echo "== check"
cd ~/guide_ws/scripts; bad=0
for f in align_yaw.py auto_map.py calib.py calib_all.py compare_maps.py front_range.py guide_manager_foxy.py loc_check.py map_clean.py nav_test.py once.py place_click.py preflight.py scan_check.py trials.py viz_relay.py wall_check.py yaw_test.py yset.py; do
  python3 -m py_compile $f 2>/dev/null || { echo "BAD  $f"; bad=1; }; done
for f in auto_map.sh bridge.sh goto.sh guide_start.sh install_autostart.sh make_cyclone.sh nav_start.sh set_home_pose.sh wait_wifi.sh; do
  bash -n $f 2>/dev/null || { echo "BAD  $f"; bad=1; }; done
for f in ~/guide_ws/launch/*.launch.py; do python3 -m py_compile $f 2>/dev/null || { echo "BAD  $f"; bad=1; }; done
for f in ekf_lab.yaml explore_lab.yaml nav2_lab_foxy.yaml slam_lab.yaml; do [ -s ~/guide_ws/params/$f ] || { echo "MISSING params/$f"; bad=1; }; done
[ -s ~/guide_ws/places_lab.yaml ] || { echo "MISSING places_lab.yaml"; bad=1; }
[ -f ~/guide_ws/maps/lab_v1.yaml ] && [ -f ~/guide_ws/maps/lab_v1.pgm ] || { echo "MISSING maps/lab_v1.{yaml,pgm}: copy them from the repo (robot/maps)"; bad=1; }
[ -f /etc/systemd/system/limo-guide.service ] || { echo "MISSING limo-guide.service"; bad=1; }
grep -q "map_subscribe_transient_local:=true" nav_start.sh || { echo "BAD  nav_start.sh (transient_local fix missing)"; bad=1; }
cd ~
[ $bad = 0 ] && echo "INSTALL OK: 19 Python + 9 shell scripts, 2 launch files, 5 parameter files, places, limo-guide service (not enabled: sudo systemctl enable limo-guide)" || echo "INSTALL NOT OK: send me the lines above"
