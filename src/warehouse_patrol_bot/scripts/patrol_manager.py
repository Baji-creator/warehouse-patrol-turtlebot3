#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math, threading, rospy, actionlib
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from warehouse_patrol_bot.msg import PatrolStatus, PatrolAction, PatrolFeedback, PatrolResult
from warehouse_patrol_bot.srv import PatrolControl, PatrolControlResponse
from maze_core import build_maze, astar, world_xy, GRID

class Manager:
 def __init__(self):
  rospy.init_node('warehouse_patrol_manager'); self.lock=threading.RLock(); self.pose=None; self.yaw=0.; self.scan=None; self.last_odom=rospy.Time(0); self.last_scan=rospy.Time(0); self.state='IDLE'; self.message='等待开始'; self.index=0; self.repeat=False; self.cancel=False; self.action_active=False; self.last_status=rospy.Time(0)
  self.path=astar(build_maze(),(0,0),(GRID-1,GRID-1)); self.points=[world_xy(c) for c in self.path]
  self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1); self.path_pub=rospy.Publisher('/warehouse_patrol/global_path',Path,queue_size=1,latch=True); self.status_pub=rospy.Publisher('/warehouse_patrol/status',PatrolStatus,queue_size=10,latch=True); self.state_pub=rospy.Publisher('/warehouse_patrol/state',String,queue_size=10,latch=True)
  rospy.Subscriber('/odom',Odometry,self.odom_cb,queue_size=1); rospy.Subscriber('/scan',LaserScan,self.scan_cb,queue_size=1)
  self.srv=rospy.Service('/warehouse_patrol/control',PatrolControl,self.control_cb); self.server=actionlib.SimpleActionServer('/warehouse_patrol/patrol',PatrolAction,execute_cb=self.execute_cb,auto_start=False); self.server.start(); rospy.Timer(rospy.Duration(.1),self.loop); rospy.on_shutdown(self.stop)
  self.publish_global_path()
  rospy.loginfo('Ready: /warehouse_patrol/status, /warehouse_patrol/control, /warehouse_patrol/patrol')
 def publish_global_path(self):
  msg=Path()
  msg.header.stamp=rospy.Time.now()
  msg.header.frame_id='odom'
  for x,y in self.points:
   pose=PoseStamped()
   pose.header=msg.header
   pose.pose.position.x=x
   pose.pose.position.y=y
   pose.pose.orientation.w=1.0
   msg.poses.append(pose)
  self.path_pub.publish(msg)
 def odom_cb(self,m):
  q=m.pose.pose.orientation; p=m.pose.pose.position
  with self.lock: self.pose=(p.x,p.y); self.yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); self.last_odom=rospy.Time.now()
 def scan_cb(self,m):
  with self.lock: self.scan=m; self.last_scan=rospy.Time.now()
 def err(self,a): return math.atan2(math.sin(a),math.cos(a))
 def front(self):
  if self.scan is None:return 0.
  a=[]
  for i,d in enumerate(self.scan.ranges):
   ang=self.scan.angle_min+i*self.scan.angle_increment
   if abs(self.err(ang))<math.radians(18) and not math.isnan(d) and not math.isinf(d) and self.scan.range_min<d<self.scan.range_max:a.append(d)
  return min(a) if a else self.scan.range_max
 def stop(self): self.pub.publish(Twist())
 def status(self,dist=0):
  m=PatrolStatus(); m.header.stamp=rospy.Time.now(); m.state=self.state; m.waypoint_index=min(self.index+1,len(self.points)); m.total_waypoints=len(self.points); m.distance_to_waypoint=float(dist); m.message=self.message; self.status_pub.publish(m); self.state_pub.publish(String(data=self.state))
 def control_cb(self,r):
  c=r.command.strip().lower()
  with self.lock:
   if c in ('start','resume','continue'):
    if self.state in ('IDLE','STOPPED','SUCCEEDED','ABORTED'):self.index=0
    self.state='RUNNING'; self.message='巡逻运行中'; self.cancel=False
   elif c=='pause':self.state='PAUSED'; self.message='已暂停'
   elif c=='stop':self.state='STOPPED'; self.message='已停止'; self.cancel=True
   elif c=='reset':self.index=0; self.state='IDLE'; self.message='已重置'; self.cancel=True
   else:return PatrolControlResponse(False,'命令应为 start/pause/resume/stop/reset')
  self.stop(); self.status(); return PatrolControlResponse(True,self.message)
 def feedback(self,d):
  f=PatrolFeedback(); f.waypoint_index=min(self.index+1,len(self.points)); f.total_waypoints=len(self.points); f.distance_to_waypoint=float(d); f.state=self.state; return f
 def execute_cb(self,g):
  with self.lock:self.index=0; self.repeat=g.repeat; self.cancel=False; self.action_active=True; self.state='RUNNING'; self.message='Action任务执行中'
  result=PatrolResult(); rate=rospy.Rate(5)
  while not rospy.is_shutdown():
   if self.server.is_preempt_requested() or self.cancel or self.state=='STOPPED':
    self.stop(); result.success=False; result.message='任务已取消'; self.server.set_preempted(result); self.action_active=False; return
   if self.state=='PAUSED':self.stop(); self.server.publish_feedback(self.feedback(0)); rate.sleep(); continue
   if self.index>=len(self.points):
    if self.repeat:self.index=0; continue
    self.stop(); result.success=True; result.message='巡逻完成'; self.state='SUCCEEDED'; self.message=result.message; self.action_active=False; self.status(); self.server.set_succeeded(result); return
   d=self.distance()
   self.server.publish_feedback(self.feedback(d)); rate.sleep()
  self.stop(); result.success=False; result.message='ROS关闭'; self.server.set_aborted(result); self.action_active=False
 def distance(self):
  if self.pose is None or self.index>=len(self.points):return 0.
  x,y=self.points[self.index]; return math.hypot(x-self.pose[0],y-self.pose[1])
 def loop(self,e):
  dist=self.distance()
  if self.state=='RUNNING':
   if self.pose is None or self.scan is None or (rospy.Time.now()-self.last_odom).to_sec()>1 or (rospy.Time.now()-self.last_scan).to_sec()>1:self.stop(); self.message='等待有效的 /odom 和 /scan'
   elif self.index>=len(self.points):
    self.stop()
    if not self.action_active:self.state='SUCCEEDED'; self.message='巡逻完成'
   else:
    tx,ty=self.points[self.index]; dx,dy=tx-self.pose[0],ty-self.pose[1]; dist=math.hypot(dx,dy); front=self.front()
    if dist<.22:self.index+=1; self.stop()
    elif front<.22:self.stop(); self.message='前方障碍过近，安全停车'
    else:
     er=self.err(math.atan2(dy,dx)-self.yaw); cmd=Twist()
     if abs(er)>.3:cmd.angular.z=max(-.65,min(.65,1.5*er))
     else:cmd.linear.x=min(.18,max(.05,.65*dist)); cmd.angular.z=max(-.22,min(.22,.7*er)); cmd.linear.x=min(cmd.linear.x,.035) if front<.4 else cmd.linear.x
     self.pub.publish(cmd)
  else:self.stop()
  if (rospy.Time.now()-self.last_status).to_sec()>.5:self.status(dist); self.last_status=rospy.Time.now()
if __name__=='__main__':
 try: Manager(); rospy.spin()
 except rospy.ROSInterruptException: pass
