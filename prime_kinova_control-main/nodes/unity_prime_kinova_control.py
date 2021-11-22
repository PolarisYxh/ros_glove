#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import json
import socket
import threading

import rospy
from kinova_msgs.msg import PoseVelocity
from primeii_ros_msgs.msg import FingerFlex, GloveData, GlovesData
import tf.transformations as tft
from geometry_msgs.msg import TwistStamped,Twist,Vector3
import std_msgs
sub = None
velo_unity_pub = None
finger_unity_pub = None
fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]
open=True
twist = Twist()
# 大拇指控制抓取
# 食指、中指、无名指分别控制末端平移
# 手掌控制
def callBack(msg):
  global velo_unity_pub,twist,open

  twist.linear = Vector3(0,0,0)#0, 0, 0
  twist.angular = Vector3(0,0,0)#0, 0, 0
  g = msg.glovesData[0]
  e = tft.euler_from_quaternion([g.wristIMU.x, g.wristIMU.y, g.wristIMU.z,g.wristIMU.w])
  x=e[0]
  y=e[1]
  z=e[2]
  # 旋转 just for left hand, horizon is zero；
  if y>1.2 or y<-0.5:
    if y>1.2:#right
      twist.angular.z = -0.3
    elif y<-0.5:#left
      twist.angular.z = 0.3
  elif z>1 or z<-1:
    if z>1:# up
      twist.angular.y = -0.3
    elif z<-1:
      twist.angular.y = 0.3
  elif x>1 or x<-1:
    if x>1:# ni
      twist.angular.x = -0.3
    elif x<-1:# shun
      twist.angular.x = 0.3
  # 握拳控制夹取
  if g.fingersFlex[0].Joint2Stretch > 0.3:
    open = False
  elif g.fingersFlex[0].Joint2Stretch < 0.3:
    open = True
  # 平移
  if g.fingersFlex[1].Joint1Stretch > 0.3:
    twist.linear.z = -0.1
  if g.fingersFlex[1].Joint1Stretch < -0.3:
    twist.linear.z = 0.1
  if g.fingersFlex[2].Joint1Stretch > 0.3:
    twist.linear.y = -0.1
  if g.fingersFlex[2].Joint1Stretch < -0.3:
    twist.linear.y = 0.1
  if g.fingersFlex[3].Joint1Stretch > 0.3:
    twist.linear.x = -0.1
  if g.fingersFlex[3].Joint1Stretch < -0.3:
    twist.linear.x = 0.1
  # CURRENT_VELOCITY[3] = 1 * dr #x: end effector self rotate,positive is shun
  # CURRENT_VELOCITY[4] = 1 * dp #y: up and down rotate (positive is down)
  # CURRENT_VELOCITY[5] = 1 * dyaw  #z: left and right rotate,positive is left


def main():
  global sub, pub,velo_unity_pub,cur_velo,finger_unity_pub,open
  rospy.init_node("prime_kinova_unity_control")
  #init Subscriber and Publisher
  sub = rospy.Subscriber("GlovesData", GlovesData, callBack)
  velo_unity_pub = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1)#j2n6s300_driver/in/cartesian_velocity
  finger_unity_pub = rospy.Publisher('/j2n6s300/fingers', std_msgs.msg.Float32, queue_size=1)

  # rospy.spin()
  r = rospy.Rate(110)
  while not rospy.is_shutdown():
      cur_velo = TwistStamped()
      cur_velo.twist = twist
      velo_unity_pub.publish(cur_velo)
      x = std_msgs.msg.Float32()
      x.data = open  # 0为close, 1为open
      finger_unity_pub.publish(x)
      r.sleep()
if __name__ == "__main__":
  while 1:
    try:
      main()
    except rospy.ROSInterruptException:
      pass
