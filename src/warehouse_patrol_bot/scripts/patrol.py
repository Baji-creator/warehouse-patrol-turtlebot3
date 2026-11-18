#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class Patrol:
    def __init__(self):
        rospy.init_node('warehouse_patrol_controller')
        self.pose = None
        self.yaw = 0.0
        self.scan = None
        self.last_scan = rospy.Time(0)
        self.mode = 'WAIT'
        self.turn_sign = 1.0
        self.clear_count = 0
        self.obstacle_since = rospy.Time(0)
        self.last_log = rospy.Time(0)

        # 内侧通道巡逻点，避开仓库四角墙体
        self.points = [
            (-2.0, -2.0), (2.0, -2.0),
            (2.0, 2.0), (-2.0, 2.0)
        ]
        self.target = 0
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.scan_cb, queue_size=1)
        rospy.on_shutdown(self.stop)
        rospy.loginfo("巡航控制器启动：等待里程计与雷达数据")

    def stop(self):
        self.pub.publish(Twist())

    def odom_cb(self, msg):
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y),
                              1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_cb(self, msg):
        self.scan = msg
        self.last_scan = rospy.Time.now()

    def sector_min(self, center_deg, width_deg):
        s = self.scan
        if not s or not s.ranges:
            return 0.0
        vals = []
        for i, r in enumerate(s.ranges):
            a = math.degrees(s.angle_min + i*s.angle_increment)
            # 将角度归一化到 [-180, 180]
            a = (a + 180.0) % 360.0 - 180.0
            if abs((a-center_deg+180.0) % 360.0-180.0) <= width_deg:
                if (not math.isnan(r) and not math.isinf(r)) and s.range_min < r:
                    vals.append(min(r, s.range_max))
                elif r == float('inf'):
                    vals.append(s.range_max)
        return min(vals) if vals else s.range_max

    def angle_error(self, a):
        return (a + math.pi) % (2.0*math.pi) - math.pi

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd = Twist()
            now = rospy.Time.now()

            if self.pose is None or self.scan is None:
                self.pub.publish(cmd)
                rate.sleep()
                continue

            if (now-self.last_scan).to_sec() > 1.0:
                rospy.logwarn_throttle(2.0, "雷达数据已超时，停止机器人")
                self.pub.publish(cmd)
                rate.sleep()
                continue

            front = self.sector_min(0, 22)
            left = self.sector_min(55, 28)
            right = self.sector_min(-55, 28)

            # 避障采用明确的转向状态，避免每一帧左右反复切换
            if self.mode != 'AVOID' and front < 0.65:
                self.mode = 'AVOID'
                self.obstacle_since = now
                self.clear_count = 0
                self.turn_sign = 1.0 if left >= right else -1.0
                rospy.logwarn("前方障碍 %.2fm，开始向%s避让",
                              front, "左侧" if self.turn_sign > 0 else "右侧")

            if self.mode == 'AVOID':
                cmd.linear.x = 0.0
                cmd.angular.z = 0.48 * self.turn_sign
                if front > 0.95:
                    self.clear_count += 1
                else:
                    self.clear_count = 0

                # 连续清空一段时间才恢复导航；长时间受阻则换方向
                if self.clear_count >= 8:
                    self.mode = 'NAV'
                    self.clear_count = 0
                    rospy.loginfo("前方通畅，恢复巡航")
                elif (now-self.obstacle_since).to_sec() > 7.0:
                    self.turn_sign *= -1.0
                    self.obstacle_since = now
                    self.clear_count = 0
                    rospy.logwarn("避障持续受阻，切换转向")
                self.pub.publish(cmd)
                rate.sleep()
                continue

            self.mode = 'NAV'
            gx, gy = self.points[self.target]
            dx, dy = gx-self.pose[0], gy-self.pose[1]
            dist = math.hypot(dx, dy)

            if dist < 0.22:
                old = self.target
                self.target = (self.target+1) % len(self.points)
                rospy.loginfo("到达航点 %d，前往航点 %d",
                              old+1, self.target+1)
                self.pub.publish(cmd)
                rate.sleep()
                continue

            desired = math.atan2(dy, dx)
            err = self.angle_error(desired-self.yaw)

            if abs(err) > 0.38:
                cmd.angular.z = max(-0.65, min(0.65, 1.25*err))
                cmd.linear.x = 0.0
            else:
                cmd.angular.z = max(-0.35, min(0.35, 0.8*err))
                # 近障时进一步减速，雷达阈值仍由避障状态接管
                cmd.linear.x = min(0.20, 0.12 + 0.12*dist)
                if front < 0.90:
                    cmd.linear.x = min(cmd.linear.x, 0.07)

            self.pub.publish(cmd)
            if (now-self.last_log).to_sec() > 3.0:
                rospy.loginfo("巡航：航点%d/%d，距离%.2fm，前方%.2fm",
                              self.target+1, len(self.points), dist, front)
                self.last_log = now
            rate.sleep()

if __name__ == '__main__':
    try:
        Patrol().run()
    except rospy.ROSInterruptException:
        pass
