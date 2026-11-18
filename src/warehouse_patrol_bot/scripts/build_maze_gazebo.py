#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion
from maze_core import build_maze, wall_segments, world_xy, GRID

MODEL_NAME = 'maze_structure'

def make_sdf(segments):
    parts = [
        "<?xml version='1.0'?>",
        "<sdf version='1.6'><model name='%s'><static>true</static>" % MODEL_NAME
    ]

    for i, (x, y, sx, sy, yaw) in enumerate(segments):
        parts.append("""
<link name="wall_%03d">
  <pose>%.4f %.4f 0.75 0 0 %.5f</pose>
  <collision name="collision">
    <geometry><box><size>%.4f %.4f 1.5</size></box></geometry>
  </collision>
  <visual name="visual">
    <geometry><box><size>%.4f %.4f 1.5</size></box></geometry>
    <material>
      <ambient>0.65 0.68 0.72 1</ambient>
      <diffuse>0.65 0.68 0.72 1</diffuse>
    </material>
  </visual>
</link>
""" % (i, x, y, yaw, sx, sy, sx, sy))

    parts.append("</model></sdf>")
    return ''.join(parts)

def main():
    rospy.init_node('build_complex_warehouse_maze')

    rospy.loginfo("Waiting for Gazebo services...")
    rospy.wait_for_service('/gazebo/get_world_properties', timeout=60)
    rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=60)
    rospy.wait_for_service('/gazebo/delete_model', timeout=60)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=60)

    get_world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    spawn = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    # Remove previous maze and old warehouse objects, but keep the robot and ground.
    names = get_world().model_names
    for name in names:
        if (name == MODEL_NAME or name.startswith('maze_wall_') or
            name.startswith('warehouse_wall_') or name.startswith('shelf_') or
            name.startswith('cargo_') or name.startswith('patrol_obstacle')):
            try:
                delete(name)
                rospy.loginfo("Removed old model: %s", name)
            except Exception as e:
                rospy.logwarn("Could not remove %s: %s", name, str(e))

    walls = build_maze()
    segments = wall_segments(walls)
    rospy.loginfo("Maze: %dx%d, %d wall segments", GRID, GRID, len(segments))

    pose = Pose()
    pose.orientation.w = 1.0
    result = spawn(MODEL_NAME, make_sdf(segments), '', pose, 'world')
    if not result.success:
        rospy.logerr("Maze spawn failed: %s", result.status_message)
        return

    x, y = world_xy((0, 0))
    state = ModelState()
    state.model_name = 'turtlebot3_burger'
    state.pose.position = Point(x, y, 0.10)
    state.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
    state.reference_frame = 'world'
    moved = set_state(state)

    rospy.loginfo("Maze model spawned successfully.")
    rospy.loginfo("Robot moved to start cell (0,0): x=%.2f y=%.2f", x, y)
    rospy.loginfo("Entrance is at the top-left boundary; exit is at bottom-right.")
    if not moved.success:
        rospy.logwarn("Robot position update failed: %s", moved.status_message)

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        rospy.logerr("Maze generator failed: %s", str(e))
        raise
