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
