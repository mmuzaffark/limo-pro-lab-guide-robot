# Limo Pro Lab Guide Robot (real robot)

A guide robot on the AgileX Limo Pro (ROS 2 Foxy, Jetson): a visitor picks a place by button or voice on a laptop GUI, the robot drives there with Nav2, turns to face the place, waits 10 s and returns home. Demo room 3.0 × 3.4 m with four places (Home, Auditorium, Library, Cafeteria). The Gazebo simulation of the same guide is in [limo-ros2-campus-guide-robot](https://github.com/mmuzaffark/limo-ros2-campus-guide-robot).

## Results (29 Oct 2026)

| Test | Result |
| --- | --- |
| Navigation test, 2 rounds (`robot/logs/nav_test_20261029_1244.csv`) | 6/6 places; parking error mean 0.18 m, max 0.23 m; heading error max 5.3° |
| Guide trials (`robot/logs/trials_20261029_1323.csv`) | 12/12 arrived, 0 collisions; parking mean 0.175 m; heading mean 4.4° |
| Power-on start (systemd service `limo-guide`) | ready ~40 s after boot, localization 100 % |
| Autonomous mapping (explore_lite) | limitation: stops early in the small room and smears frame-style furniture in the main lab; the demo map was made by hand |

`RESULTS.md` has all logged runs (calibration, mapping, navigation tests, trials).

## Layout

| Path | Contents |
| --- | --- |
| `install_guide.sh` | Rebuilds the whole robot side (`~/guide_ws`): scripts, launch files, parameters, places, service. Byte-identical to the robot's files |
| `robot/scripts` | Guide manager, Nav2 start, localization check, heading alignment, nav test, trials, mapping, calibration |
| `robot/params`, `robot/launch` | Nav2 (RPP controller), SLAM, explore, EKF parameters; base launch with EKF |
| `robot/maps` | `lab_v1` = demo map; other maps from the mapping runs |
| `robot/patches` | `fix_rpp_goal_dist_tol.sh`: fix for the Nav2 RPP controller in the robot's source build |
| `robot/logs` | CSV logs of every test |
| `laptop` | Windows GUI with voice (`lab_gui_win.py`), start and sync scripts |

## Rebuild a robot

1. Limo Pro with ROS 2 Foxy and AgileX `limo_ros2_ws` (Nav2 0.4.7 source build); run `robot/patches/fix_rpp_goal_dist_tol.sh`.
2. Build `m-explore-ros2` at commit `e40e857` in `~/guide_ws/src` (newer commits need Humble).
3. Copy `install_guide.sh` to the robot and run `bash ~/install_guide.sh` inside tmux; copy `robot/maps/lab_v1.*` to `~/guide_ws/maps`.
4. Robot on the Home mark: `~/guide_ws/scripts/guide_start.sh` (or `sudo systemctl enable --now limo-guide`); laptop: `start_gui.ps1`.

## Built on (not included here)

| Project | Used for |
| --- | --- |
| [agilexrobotics/limo_ros2](https://github.com/agilexrobotics/limo_ros2) | Limo base driver and bring-up |
| [ros-planning/navigation2](https://github.com/ros-planning/navigation2) 0.4.7 (Foxy) | AMCL, map server, planner, Regulated Pure Pursuit controller |
| [SteveMacenski/slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) | Mapping |
| [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) `e40e857` | Frontier exploration |
| [cra-ros-pkg/robot_localization](https://github.com/cra-ros-pkg/robot_localization) | EKF: wheel speed + IMU yaw rate |
| [RobotWebTools/rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite), [roslibpy](https://github.com/gramaziokohler/roslibpy) | Laptop link |
| [faster-whisper](https://github.com/SYSTRAN/faster-whisper), [RapidFuzz](https://github.com/rapidfuzz/RapidFuzz) | Voice commands |
