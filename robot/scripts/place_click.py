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
