#!/bin/bash
# fix_rpp_goal_dist_tol.sh : patch for the Nav2 0.4.7 (Foxy) source build in ~/limo_ros2_ws, applied on the robot 28 Oct 2026.
# Bug: in the copy shipped in limo_ros2_ws, line ~145 of regulated_pure_pursuit_controller.cpp did not read the
# parameter "goal_dist_tol" into goal_dist_tol_ (typo), so it stayed at the 0.25 m default. With xy_goal_tolerance
# smaller than 0.25 m the robot stopped driving 0.25 m from the goal and only turned (endless rotation).
# Upstream navigation2 tag 0.4.7 has the correct line; this script restores it and rebuilds the package.
# Idempotent: prints "already correct" if nothing to do. Backup: <file>.bak (kept if it exists).
set -e
F=~/limo_ros2_ws/src/navigation2/nav2_regulated_pure_pursuit_controller/src/regulated_pure_pursuit_controller.cpp
GOOD='  node->get_parameter(plugin_name_ + ".goal_dist_tol", goal_dist_tol_);'
[ -f "$F" ] || { echo "not found: $F"; exit 1; }
if grep -qF "$GOOD" "$F"; then echo "already correct: $(grep -nF "$GOOD" "$F")"; exit 0; fi
[ -f "$F.bak" ] || cp "$F" "$F.bak"
python3 - "$F" "$GOOD" <<'PY'
import re, sys
f, good = sys.argv[1], sys.argv[2]; L = open(f).read().split('\n')
bad = [i for i, l in enumerate(L) if 'get_parameter' in l and 'goal_dist_to' in l]
if bad:
    print('replaced line %d: %s' % (bad[0] + 1, L[bad[0]].strip())); L[bad[0]] = good
else:
    k = [i for i, l in enumerate(L) if 'get_parameter' in l and '.max_angular_accel"' in l]
    if not k: sys.exit('anchor line (max_angular_accel) not found: fix by hand')
    L.insert(k[0] + 1, good); print('inserted after line %d' % (k[0] + 1))
open(f, 'w').write('\n'.join(L))
PY
grep -nF "$GOOD" "$F"
diff "$F.bak" "$F" > ~/guide_ws/logs/rpp_goal_dist_tol.diff || true
source /opt/ros/foxy/setup.bash
cd ~/limo_ros2_ws && colcon build --symlink-install --packages-select nav2_regulated_pure_pursuit_controller
echo "RPP fixed and rebuilt; diff in ~/guide_ws/logs/rpp_goal_dist_tol.diff. Restart nav_start.sh / the limo-guide service."
