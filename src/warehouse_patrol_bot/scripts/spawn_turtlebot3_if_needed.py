#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.srv import GetWorldProperties, SpawnModel


def main():
    rospy.init_node('spawn_turtlebot3_if_needed', anonymous=False)
    model_name = rospy.get_param('~model_name', 'turtlebot3_burger')
    robot_description_param = rospy.get_param('~robot_description_param', 'robot_description')
    x = float(rospy.get_param('~x', -7.0))
    y = float(rospy.get_param('~y', 0.0))
    z = float(rospy.get_param('~z', 0.05))
    reference_frame = rospy.get_param('~reference_frame', 'world')

    rospy.wait_for_service('/gazebo/get_world_properties', timeout=30.0)
    rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=30.0)
    world = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    spawn = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    model_names = set(world().model_names)
    if model_name in model_names:
        rospy.logwarn('Model %s already exists in Gazebo; skip respawn to avoid duplicate entity conflict.', model_name)
        return

    robot_xml = rospy.get_param(robot_description_param)
    from geometry_msgs.msg import Pose
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.w = 1.0

    # Keep plugin TF and joint states in the same root namespace as the URDF.
    result = spawn(model_name, robot_xml, '/', pose, reference_frame)
    if not result.success:
        raise rospy.ROSException('Spawn failed for %s: %s' % (model_name, result.status_message))
    rospy.loginfo('Spawn guard created model %s successfully.', model_name)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
