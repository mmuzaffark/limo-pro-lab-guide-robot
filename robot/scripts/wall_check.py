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
