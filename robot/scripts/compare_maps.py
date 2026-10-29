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
