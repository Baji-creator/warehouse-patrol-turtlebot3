#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance:
    def __init__(self):
        rospy.init_node('warehouse_obstacle_avoidance')

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.last_scan_time = None

        self.stop_distance = 0.45
        self.turn_distance = 0.75
        self.speed = 0.12
        self.turn_speed = 0.45

        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)
        rospy.loginfo("Warehouse obstacle avoidance started.")

    def sector_min(self, msg, start_deg, end_deg):
        values = []
        for i, distance in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle)
            if start_deg <= angle_deg <= end_deg:
                if (not math.isnan(distance) and not math.isinf(distance)) and msg.range_min <= distance <= msg.range_max:
                    values.append(distance)
        return min(values) if values else float('inf')

    def scan_callback(self, msg):
        # TurtleBot3 雷达角度从 0 到 360 度
        self.front_distance = min(
            self.sector_min(msg, 345, 360),
            self.sector_min(msg, 0, 15)
        )
        self.left_distance = self.sector_min(msg, 45, 135)
        self.right_distance = self.sector_min(msg, 225, 315)
        self.last_scan_time = rospy.Time.now()

    def control_loop(self, event):
        cmd = Twist()

        # 雷达数据失联时，默认停车
        if self.last_scan_time is None or \
           (rospy.Time.now() - self.last_scan_time).to_sec() > 1.0:
            self.pub.publish(cmd)
            return

        # 前方太近：原地转向，优先选择更空旷的一侧
        if self.front_distance < self.stop_distance:
            if self.left_distance >= self.right_distance:
                cmd.angular.z = self.turn_speed
            else:
                cmd.angular.z = -self.turn_speed

        # 前方较近：减速并转向
        elif self.front_distance < self.turn_distance:
            cmd.linear.x = 0.04
            if self.left_distance >= self.right_distance:
                cmd.angular.z = self.turn_speed
            else:
                cmd.angular.z = -self.turn_speed

        # 前方通畅：向前行驶
        else:
            cmd.linear.x = self.speed

        self.pub.publish(cmd)

    def shutdown(self):
        self.pub.publish(Twist())


if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
