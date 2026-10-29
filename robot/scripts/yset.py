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
