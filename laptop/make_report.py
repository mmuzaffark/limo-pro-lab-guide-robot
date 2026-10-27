#!/usr/bin/env python3
# make_report.py LOG_DIR > RESULTS.md : collect calibration, mapping, navigation and trial logs into
# one Markdown results page (tables for the progress report). Plain Python, runs on the laptop.
import csv, glob, math, os, re, statistics, sys

D = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/guide_ws/logs')


def rows(path):
    with open(path) as f: return list(csv.DictReader(f))


def num(v):
    try: return float(v)
    except (TypeError, ValueError): return None


def mean(v):
    v = [x for x in v if x is not None and not math.isnan(x)]
    return statistics.mean(v) if v else None


def fmt(v, f='%.2f'):
    return f % v if v is not None else '–'


def stamp(p):
    m = re.search(r'(\d{8})_(\d{4})', p)
    return '%s-%s-%s %s:%s' % (m.group(1)[:4], m.group(1)[4:6], m.group(1)[6:], m.group(2)[:2], m.group(2)[2:]) if m else p


out = ['# Real Limo Pro: results', '', 'Generated from `%s` by `make_report.py`.' % D, '']
cal = sorted(glob.glob(os.path.join(D, 'calib_*.csv')))
out += ['## Odometry calibration (Stage 3)', '']
if cal:
    out += ['| Run | Straight odom err % | Yaw odom err % CCW / CW | Yaw IMU err % CCW / CW | Gate (≤ 2 % / ≤ 3 %) |', '| --- | --- | --- | --- | --- |']
    for p in cal:
        R = rows(p); g = lambda t, k: mean([num(r[k]) for r in R if r['test'] == t])
        s, yc, yw = g('straight', 'err_odom_pct'), g('ccw', 'err_odom_pct'), g('cw', 'err_odom_pct')
        ok = s is not None and abs(s) <= 2 and yc is not None and max(abs(yc), abs(yw or 0)) <= 3
        out.append('| %s | %s | %s / %s | %s / %s | %s |' % (stamp(p), fmt(s, '%+.2f'), fmt(yc, '%+.2f'), fmt(yw, '%+.2f'),
                   fmt(g('ccw', 'err_imu_pct'), '%+.2f'), fmt(g('cw', 'err_imu_pct'), '%+.2f'), 'PASS' if ok else 'FAIL'))
else:
    out.append('No `calib_*.csv` yet.')
out += ['', '## Autonomous mapping (Stage 5)', '']
maps = sorted(glob.glob(os.path.join(D, 'auto_map_*.txt')))
if maps:
    out += ['| Run | Result | Returned home | Duration (min) | Path (m) | Free area (m²) | Walls extent (m) |', '| --- | --- | --- | --- | --- | --- | --- |']
    for p in maps:
        t = open(p).read(); f = lambda k: (re.search(k + r':\s*(.*)', t) or [None, '–'])[1].strip()
        out.append('| %s | %s | %s | %s | %s | %s | %s |' % (stamp(p), f('result'), f('returned home'), f('duration').split()[0],
                   f('path driven').split()[0], f('free area').split()[0], f('walls bbox').replace(' m', '')))
else:
    out.append('No `auto_map_*.txt` yet.')
for title, pat, ok_word, err_col in (('Navigation test (Stage 8)', 'nav_test_*.csv', 'SUCCEEDED', 'est_err_m'),
                                      ('Guide trials (Stage 11)', 'trials_*.csv', 'arrived', 'est_err_m')):
    out += ['', '## ' + title, '']
    files = sorted(glob.glob(os.path.join(D, pat)))
    if not files:
        out.append('No `%s` yet.' % pat); continue
    out += ['| Run | Goals | Reached | Success % | Parking own estimate mean / max (m) | Parking tape mean (m) | Battery start → end (V) |',
            '| --- | --- | --- | --- | --- | --- | --- |']
    allr = []
    for p in files:
        R = [r for r in rows(p) if r.get('place') != 'home']; allr += R
        ok = [r for r in R if r['result'] == ok_word]; e = [num(r[err_col]) for r in ok]
        e = [x for x in e if x is not None and not math.isnan(x)]; tp = [num(r['tape_err_m']) for r in ok if r['tape_err_m']]
        b = [num(r['battery_v']) for r in R if r['battery_v']]
        out.append('| %s | %d | %d | %.0f | %s / %s | %s | %s → %s |' % (stamp(p), len(R), len(ok), 100.0 * len(ok) / len(R) if R else 0,
                   fmt(mean(e), '%.3f'), fmt(max(e) if e else None, '%.3f'), fmt(mean(tp), '%.3f'),
                   fmt(b[0] if b else None), fmt(b[-1] if b else None)))
    if len(files) > 1:
        ok = [r for r in allr if r['result'] == ok_word]
        out.append('| **All** | %d | %d | %.0f | %s | %s | |' % (len(allr), len(ok), 100.0 * len(ok) / len(allr),
                   fmt(mean([num(r[err_col]) for r in ok]), '%.3f'), fmt(mean([num(r['tape_err_m']) for r in ok if r['tape_err_m']]), '%.3f')))
    fails = [r for r in allr if r['result'] != ok_word]
    if fails:
        out += ['', 'Failures: ' + '; '.join('%s %s' % (r['place'], r['result']) for r in fails)]
print('\n'.join(out))