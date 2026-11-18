# Visual Acceptance

Remote evidence directory: /home/la/catkin_ws/acceptance_logs/visual_final_20261028_055557

## Changes

- complex_warehouse.world: replace white external ground/sun includes with local grey ground and controlled directional lighting; reduce point-light intensity; replace opaque rack visual shells with dark posts; place separate brown/blue cargo visuals above shelves; raise non-colliding floor markings above the floor; correct overview camera pitch.
- warehouse_patrol.rviz: keep Fixed Frame odom, use cyan real LaserScan points (3 pixels, 0.25-second decay), yellow real global path (0.06 m width), dark background, root-frame overview instead of robot-following view.
- spawn_turtlebot3_if_needed.py: use root robot_namespace "/" in SpawnModel rather than empty namespace. This reconnects odom -> base_footprint -> base_link -> base_scan and restores root joint_states.
- Navigation algorithms, patrol points, sensor sample count/range, driver dynamics and existing obstacle collisions are unchanged.

## Verified

- catkin_make and XML/YAML/Python checks pass.
- Existing warehouse model poses and collision XML compare equal before/after. Floor markings have no collisions.
- Exactly one turtlebot3_burger; one real gzserver and one real gzclient (shell wrappers are not extra simulator instances).
- Final gzclient and gzserver continuously alive for over five minutes with software rendering.
- 65.97 wall seconds of telemetry: 5.466 m travelled; waypoint index 2 -> 3; state RUNNING.
- Sensor-time interval during that window: 34.833 seconds. Simulation runs slower than wall clock; do not describe it as real-time speed.
- LaserScan: 7.987 Hz in simulation timestamps, 540 samples, 6 m maximum, frame base_scan, 523 finite returns in final sampled scan.
- Odometry: 30.000 Hz in simulation timestamps; frame odom.
- Map and path frame odom; map 73 x 73 cells; path 345 poses; zero sampled path poses in occupied/outside cells.
- Zero TF lookup errors during the telemetry window. End TF/Gazebo planar difference approximately 0.00086 m.
- Real visible desktop screenshots: gazebo_detail_visible.png and rviz_detail_visible.png. RViz Global Status and Map Status visibly Ok; robot, cyan returns and yellow path visible together.
- Sources/version hashes: version.sha256. Backups: backups.txt. Build: build_final.log. Motion: motion_65s.json. Summary: verification.json.

## Limitations

- Native VMware rendering previously produced gzclient exit 134. Exact abort stack was not captured; software-rendering mode passed. Do not claim the native graphics driver is repaired.
- GUI real-time factor approximately 0.47-0.55, RViz approximately 12-21 FPS depending on view.
- No SLAM or costmap was added. OccupancyGrid is the existing known-world A* representation, not the reference image's SLAM/costmap layers.
- Warehouse uses local SDF geometry rather than photorealistic textured assets. Collision envelopes intentionally remain conservative solid racks.
- Service/Action behavior and forced obstacle challenge were not re-exercised during this visual-only round.

## Reproduce

```bash
source /opt/ros/melodic/setup.bash
source /home/la/catkin_ws/devel/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
export QT_X11_NO_MITSHM=1
export SVGA_VGPU10=0
export GAZEBO_MODEL_DATABASE_URI=
roslaunch warehouse_patrol_v2 patrol.launch gui:=true headless:=false use_rviz:=true autostart:=true loop_patrol:=true
```

Do not start this command while the current demonstration instance is running.
Normal shutdown: Ctrl+C in the launching terminal. For the SSH-started instance,
send SIGINT to the exact PID recorded in launch.pid; the separately restarted
RViz PID is recorded in rviz.pid.
