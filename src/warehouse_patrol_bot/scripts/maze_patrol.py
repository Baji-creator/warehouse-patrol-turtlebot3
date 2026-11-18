#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from maze_core import build_maze, astar, world_xy, GRID


class MazePatrol(object):
    def __init__(self):
        rospy.init_node('maze_patrol_controller')

        self.pose = None
        self.yaw = 0.0
        self.scan = None
        self.last_odom = rospy.Time(0)
        self.last_scan = rospy.Time(0)
        self.last_log = rospy.Time(0)
        self.index = 0

        walls = build_maze()
        self.path = astar(walls, (0, 0), (GRID - 1, GRID - 1))

        if not self.path:
            raise RuntimeError("A* could not find a route")

        self.points = [world_xy(cell) for cell in self.path]

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.on_shutdown(self.stop)

        rospy.loginfo("A* route: %d cells", len(self.path))
        rospy.loginfo("Start: %s; goal: %s",
                      str(self.points[0]), str(self.points[-1]))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.pose = (p.x, p.y)
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.last_odom = rospy.Time.now()

    def scan_cb(self, msg):
        self.scan = msg
        self.last_scan = rospy.Time.now()

    def stop(self):
        if hasattr(self, 'pub'):
            self.pub.publish(Twist())

    def angle_error(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def front_distance(self):
        scan = self.scan
        if scan is None:
            return 0.0

        values = []
        for i, distance in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(self.angle_error(angle)) > math.radians(15):
                continue
            if math.isnan(distance) or math.isinf(distance):
                continue
            if scan.range_min < distance < scan.range_max:
                values.append(distance)

        return min(values) if values else scan.range_max

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.pose is not None and self.scan is not None:
                break
            self.stop()
            rate.sleep()

        rospy.loginfo("Navigation started")

        while not rospy.is_shutdown():
            now = rospy.Time.now()

            if self.pose is None or self.scan is None:
                self.stop()
                rate.sleep()
                continue

            if ((now - self.last_odom).to_sec() > 1.0 or
                    (now - self.last_scan).to_sec() > 1.0):
                self.stop()
                rospy.logwarn_throttle(2.0, "Sensor data stale; stopped")
                rate.sleep()
                continue

            if self.index >= len(self.points):
                self.stop()
                rospy.loginfo("GOAL REACHED: patrol completed")
                break

            tx, ty = self.points[self.index]
            x, y = self.pose
            dx, dy = tx - x, ty - y
            distance = math.hypot(dx, dy)

            if distance < 0.18:
                cell = self.path[self.index]
                rospy.loginfo("Reached cell %d/%d: %s",
                              self.index + 1, len(self.path), str(cell))
                self.index += 1
                self.stop()
                rate.sleep()
                continue

            desired = math.atan2(dy, dx)
            error = self.angle_error(desired - self.yaw)
            front = self.front_distance()
            cmd = Twist()

            # Stop rather than drive into a nearby obstacle.
            if front < 0.22:
                self.stop()
                rospy.logwarn_throttle(2.0,
                                       "Obstacle too close; waiting safely")
                rate.sleep()
                continue

            if abs(error) > 0.30:
                cmd.linear.x = 0.0
                cmd.angular.z = max(-0.80, min(0.80, 2.0 * error))
            else:
                cmd.linear.x = min(0.25, max(0.08, 0.8 * distance))
                cmd.angular.z = max(-0.25, min(0.25, 0.8 * error))

                if front < 0.40:
                    cmd.linear.x = min(cmd.linear.x, 0.04)

            self.pub.publish(cmd)

            if (now - self.last_log).to_sec() >= 3.0:
                rospy.loginfo("Waypoint %d/%d, distance=%.2f, front=%.2f",
                              self.index + 1, len(self.points),
                              distance, front)
                self.last_log = now

            rate.sleep()

        self.stop()


if __name__ == '__main__':
    try:
        MazePatrol().run()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        rospy.logerr("Maze patrol failed: %s", str(exc))
        raise
