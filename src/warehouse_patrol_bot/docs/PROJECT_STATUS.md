# Warehouse Patrol Bot — integration package

## What is included
- Deterministic 11×11 maze generation and A* path planning (`maze_core.py`).
- Gazebo maze spawn and robot repositioning (`build_maze_gazebo.py`).
- One controller node (`patrol_manager.py`) owning `/cmd_vel`, consuming `/odom` and `/scan`.
- ROS message `/warehouse_patrol/status`, state topic `/warehouse_patrol/state`, service `/warehouse_patrol/control`, action `/warehouse_patrol/patrol`.
- Combined launch file `warehouse_demo.launch`, plus diagnostics script.

## Install
1. Copy this bundle's contents to a folder in the Ubuntu VM. If it is in the VMware shared folder `S`, for example, extract it under `~/warehouse_patrol_bundle`.
2. Run:
   ```bash
   cd ~/warehouse_patrol_bundle
   bash scripts/install_in_vm.sh
   ```
   The installer backs up the existing package before updating it, then runs `catkin_make`.
3. Start simulation:
   ```bash
   source /opt/ros/melodic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   export TURTLEBOT3_MODEL=burger
   roslaunch warehouse_patrol_bot warehouse_demo.launch
   ```
4. In another terminal, control it:
   ```bash
   source /opt/ros/melodic/setup.bash
   source ~/catkin_ws/devel/setup.bash
   rosservice call /warehouse_patrol/control "command: start"
   rosservice call /warehouse_patrol/control "command: pause"
   rosservice call /warehouse_patrol/control "command: resume"
   rosservice call /warehouse_patrol/control "command: stop"
   ```
5. Run diagnostics:
   ```bash
   bash ~/catkin_ws/src/warehouse_patrol_bot/scripts/diagnose_warehouse.sh
   ```

## Important single-controller rule
Do not launch `patrol.py`, `maze_patrol.py`, or `obstacle_avoidance.py` alongside `patrol_manager.py`: all may publish `/cmd_vel`, creating conflicting commands.

## Verification boundary
The message/service/action definitions and manager interface have already been exercised in the user's VM, including Action goal activation and cancellation. This bundle was statically inspected and syntax-checked in the build environment, but Gazebo physics, the generated maze, real robot motion through every waypoint, collision avoidance, and RViz must still be confirmed in the user's VM. The maze uses a deterministic grid; if the robot stops near the entrance or a wall, inspect `/odom`, `/scan`, and the Gazebo model placement before claiming full completion.
