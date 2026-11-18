#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 确认 Gazebo 服务可用
rosservice call /gazebo/get_world_properties "{}" >/dev/null

WORK=/tmp/warehouse_models
mkdir -p "$WORK"

# 参数：名称 x y z 长 宽 高
make_box() {
    NAME="$1"
    X="$2"
    Y="$3"
    Z="$4"
    L="$5"
    W="$6"
    H="$7"

    FILE="$WORK/${NAME}.sdf"

    cat > "$FILE" <<EOF
<sdf version="1.6">
  <model name="${NAME}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>${L} ${W} ${H}</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>${L} ${W} ${H}</size></box>
        </geometry>
        <material>
          <ambient>0.55 0.60 0.65 1</ambient>
          <diffuse>0.55 0.60 0.65 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
EOF

    rosrun gazebo_ros spawn_model \
      -sdf -file "$FILE" -model "$NAME" \
      -x "$X" -y "$Y" -z "$Z" >/dev/null
    echo "已创建：$NAME"
}

echo "========== 创建仓库外围墙壁 =========="
make_box warehouse_wall_north  0  3.0 1.0 6.0 0.10 2.0
make_box warehouse_wall_south  0 -3.0 1.0 6.0 0.10 2.0
make_box warehouse_wall_west  -3.0 0 1.0 0.10 6.0 2.0
make_box warehouse_wall_east   3.0 0 1.0 0.10 6.0 2.0

echo "========== 创建左右两组货架 =========="
make_box shelf_left_1  -1.2  1.5 0.45 0.65 1.5 0.9
make_box shelf_left_2  -1.2 -0.5 0.45 0.65 1.5 0.9
make_box shelf_right_1  1.2  1.5 0.45 0.65 1.5 0.9
make_box shelf_right_2  1.2 -0.5 0.45 0.65 1.5 0.9

echo "========== 创建货物 =========="
make_box cargo_1 -1.2  1.5 1.0 0.30 0.30 0.20
make_box cargo_2 -1.2 -0.5 1.0 0.30 0.30 0.20
make_box cargo_3  1.2  1.5 1.0 0.30 0.30 0.20
make_box cargo_4  1.2 -0.5 1.0 0.30 0.30 0.20

echo "========== 放置独立测试障碍物 =========="
make_box patrol_obstacle 0.0 0.0 0.25 0.35 0.35 0.5

echo "========== 重置机器人到仓库入口 =========="
rosservice call /gazebo/set_model_state "model_state:
  model_name: 'turtlebot3_burger'
  pose:
    position: {x: -2.2, y: -2.2, z: 0.01}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  twist:
    linear: {x: 0.0, y: 0.0, z: 0.0}
    angular: {x: 0.0, y: 0.0, z: 0.0}
  reference_frame: 'world'" >/dev/null

echo "========== 仓库场景创建完成 =========="
rosservice call /gazebo/get_world_properties "{}"
