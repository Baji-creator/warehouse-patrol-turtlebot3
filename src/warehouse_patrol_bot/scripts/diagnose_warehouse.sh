#!/usr/bin/env bash
set +e
source /opt/ros/melodic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
echo '========== ROS MASTER =========='
rosparam get /rosdistro
echo '========== PACKAGE =========='
rospack find warehouse_patrol_bot
echo '========== NODES =========='
rosnode list
echo '========== REQUIRED TOPICS =========='
for t in /odom /scan /cmd_vel /warehouse_patrol/status /warehouse_patrol/state /warehouse_patrol/patrol/goal /warehouse_patrol/patrol/feedback /warehouse_patrol/patrol/result; do
  if rostopic list 2>/dev/null | grep -Fxq "$t"; then echo "OK   $t"; else echo "MISS $t"; fi
done
echo '========== REQUIRED SERVICE =========='
rosservice list | grep -Fx '/warehouse_patrol/control' || echo 'MISS /warehouse_patrol/control'
echo '========== ONE SAMPLE =========='
echo '-- odom --'; timeout 3 rostopic echo -n 1 /odom
 echo '-- scan metadata/sample --'; timeout 3 rostopic echo -n 1 /scan/header
 echo '-- patrol status --'; timeout 3 rostopic echo -n 1 /warehouse_patrol/status
echo '========== TF =========='
rosrun tf tf_echo odom base_footprint 2>&1 | head -12
echo '========== END =========='
