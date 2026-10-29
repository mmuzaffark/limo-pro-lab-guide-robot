# Real Limo Pro: results

Generated from `C:\Users\mmuza\limo_lab_repo\robot\logs` by `make_report.py`.

## Odometry calibration (Stage 3)

| Run | Straight odom err % | Yaw odom err % CCW / CW | Yaw IMU err % CCW / CW | Gate (≤ 2 % / ≤ 3 %) |
| --- | --- | --- | --- | --- |
| 2026-10-27 10:38 | -1.32 | – / – | – / – | FAIL |
| 2026-10-27 11:43 | +0.15 | -3.62 / +1.89 | +0.44 / -1.07 | FAIL |
| 2026-10-27 11:55 | +0.80 | +0.62 / -0.69 | +0.45 / -0.48 | PASS |

## Autonomous mapping (Stage 5)

| Run | Result | Returned home | Duration (min) | Path (m) | Free area (m²) | Walls extent (m) |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-10-27 13:15 | time limit 25 min | NO | 28.2 | 111.7 | 193.4 | 19.60 x 19.35 |
| 2026-10-27 13:57 | complete (No frontiers found, stopping.) | yes | 0.7 | 0.0 | 7.2 | 3.35 x 3.55 |
| 2026-10-28 10:11 | complete (All frontiers traversed/tried out, stopping.) | yes | 0.8 | 1.2 | 3.9 | 3.00 x 2.50 |
| 2026-10-28 10:20 | complete (All frontiers traversed/tried out, stopping.) | NO | 4.9 | 2.6 | 4.4 | 3.35 x 3.20 |

## Navigation test (Stage 8)

| Run | Goals | Reached | Success % | Parking own estimate mean / max (m) | Parking tape mean (m) | Battery start → end (V) |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-10-29 10:31 | 5 | 0 | 0 | – / – | – | 12.20 → 12.10 |
| 2026-10-29 10:37 | 2 | 0 | 0 | – / – | – | 12.10 → 12.10 |
| 2026-10-29 10:48 | 1 | 0 | 0 | – / – | – | 11.90 → 11.90 |
| 2026-10-29 10:52 | 1 | 0 | 0 | – / – | – | 11.90 → 11.90 |
| 2026-10-29 11:07 | 6 | 4 | 67 | 0.132 / 0.143 | – | 11.60 → 11.30 |
| 2026-10-29 12:25 | 6 | 5 | 83 | 0.196 / 0.238 | – | 12.40 → 12.20 |
| 2026-10-29 12:32 | 2 | 2 | 100 | 0.226 / 0.246 | – | 12.30 → 12.30 |
| 2026-10-29 12:44 | 6 | 6 | 100 | 0.179 / 0.233 | – | 12.20 → 12.10 |
| **All** | 29 | 17 | 59 | 0.177 | – | |

Failures: auditorium ABORTED; cafeteria ABORTED; library ABORTED; auditorium ABORTED; cafeteria ABORTED; auditorium ABORTED; cafeteria ABORTED; auditorium ABORTED; auditorium ABORTED; cafeteria TIMEOUT; cafeteria TIMEOUT; cafeteria TIMEOUT

## Guide trials (Stage 11)

| Run | Goals | Reached | Success % | Parking own estimate mean / max (m) | Parking tape mean (m) | Battery start → end (V) |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-10-29 13:23 | 12 | 12 | 100 | 0.175 / 0.269 | – | 11.60 → 11.20 |
