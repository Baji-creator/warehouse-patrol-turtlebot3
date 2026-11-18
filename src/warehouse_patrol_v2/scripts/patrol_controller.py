#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class PatrolController:
    def __init__(self):
        rospy.init_node("warehouse_patrol_controller")

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.scan = None
        self.last_turn = 1.0

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        self.rate = rospy.Rate(10)
        rospy.loginfo("仓库巡逻控制器已启动，等待雷达数据")

        while not rospy.is_shutdown():
            self.control()
            self.rate.sleep()

    def scan_callback(self, msg):
        self.scan = msg

    def sector_min(self, scan, center_deg, width_deg=25):
        vals = []
        n = len(scan.ranges)

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            diff = math.atan2(
                math.sin(angle - math.radians(center_deg)),
                math.cos(angle - math.radians(center_deg))
            )

            if abs(math.degrees(diff)) <= width_deg:
                if not math.isnan(r) and not math.isinf(r) and scan.range_min < r < scan.range_max:
                    vals.append(r)

        return min(vals) if vals else float("inf")

    def control(self):
        cmd = Twist()

        if self.scan is None:
            self.pub.publish(cmd)
            return

        front = self.sector_min(self.scan, 0, 22)
        left = self.sector_min(self.scan, 60, 30)
        right = self.sector_min(self.scan, -60, 30)

        # 前方遇到障碍物：原地转向，优先选择空间更宽的一侧
        if front < 0.65:
            cmd.linear.x = 0.0
            if abs(left - right) > 0.05:
                cmd.angular.z = 0.65 if left > right else -0.65
            else:
                cmd.angular.z = self.last_turn * 0.65
            self.last_turn = 1.0 if cmd.angular.z > 0 else -1.0

        # 前方畅通：前进；侧面过近时轻微调整方向
        else:
            cmd.linear.x = 0.12
            if left < 0.38:
                cmd.angular.z = -0.35
            elif right < 0.38:
                cmd.angular.z = 0.35
            else:
                cmd.angular.z = 0.0

        self.pub.publish(cmd)

    def shutdown(self):
        self.pub.publish(Twist())

if __name__ == "__main__":
    try:
        PatrolController()
    except rospy.ROSInterruptException:
        pass
