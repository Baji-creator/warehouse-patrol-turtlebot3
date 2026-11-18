#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

export TURTLEBOT3_MODEL=burger
export SVGA_VGPU10=0
export GAZEBO_MODEL_DATABASE_URI=""

PKG="$HOME/catkin_ws/src/warehouse_patrol_bot"
GAZEBO_PID=""

cleanup() {
    if [ -n "$GAZEBO_PID" ]; then
        kill "$GAZEBO_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 如果 Gazebo 未运行，则自动启动
if ! rosservice list 2>/dev/null | grep -q '^/gazebo/get_world_properties$'; then
    echo "[1/3] 启动 Gazebo..."
    roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch \
        > /tmp/warehouse_gazebo.log 2>&1 &
    GAZEBO_PID=$!

    READY=0
    for i in $(seq 1 60); do
        if rosservice list 2>/dev/null | grep -q '^/gazebo/get_world_properties$'; then
            READY=1
            break
        fi
        sleep 1
    done

    if [ "$READY" -ne 1 ]; then
        echo "Gazebo 启动失败。日志：/tmp/warehouse_gazebo.log"
        exit 1
    fi
else
    echo "[1/3] 使用已运行的 Gazebo..."
fi

# 清理旧场景模型，防止重复生成
echo "[2/3] 重建仓库场景..."
for model in \
    warehouse_wall_north warehouse_wall_south warehouse_wall_west warehouse_wall_east \
    shelf_left_1 shelf_left_2 shelf_right_1 shelf_right_2 \
    cargo_1 cargo_2 cargo_3 cargo_4 patrol_obstacle warehouse_box_1; do
    rosservice call /gazebo/delete_model "{model_name: '$model'}" \
        >/dev/null 2>&1 || true
done

bash "$PKG/scripts/build_warehouse.sh"

# 确保没有旧巡逻节点占用速度控制
rosnode kill /warehouse_patrol >/dev/null 2>&1 || true
sleep 1

echo "[3/3] 启动巡逻与避障..."
rosrun warehouse_patrol_bot patrol.py
