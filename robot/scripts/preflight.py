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
