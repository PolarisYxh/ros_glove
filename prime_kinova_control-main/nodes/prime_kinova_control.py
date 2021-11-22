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
sub = None
pub = None

fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]

def callBack(msg):
  pose_vel = PoseVelocity()
  e = tft.euler_from_quaternion([msg.glovesData[0].wristIMU.x,msg.glovesData[0].wristIMU.y,msg.glovesData[0].wristIMU.z,msg.glovesData[0].wristIMU.w])
  print(e)
  x=e[0]
  y=e[1]
  z=e[2]
  # 旋转 just for left hand, horizon is zero；
  if y>1.2 or y<-0.5:
    if y>1.2:#right
      pose_vel.twist_angular_z = -0.1
    elif y<-0.5:#left
      pose_vel.twist_angular_z = 0.1
  elif z>1 or z<-1:
    if z>1:# up
      pose_vel.twist_angular_y = -0.1
    elif z<-1:
      pose_vel.twist_angular_y = 0.1
  elif x>1 or x<-1:
    if x>1:# ni
      pose_vel.twist_angular_z = -0.1
    elif x<-1:# shun
      pose_vel.twist_angular_z = 0.1
  # 平移
  if msg.glovesData[0].fingersFlex[1].Joint1Stretch > 0.3:
    pose_vel.twist_linear_z = -0.1
  if msg.glovesData[0].fingersFlex[1].Joint1Stretch < -0.3:
    pose_vel.twist_linear_z = 0.1
  if msg.glovesData[0].fingersFlex[2].Joint1Stretch > 0.3:
    pose_vel.twist_linear_y = -0.1
  if msg.glovesData[0].fingersFlex[2].Joint1Stretch < -0.3:
    pose_vel.twist_linear_y = 0.1
  if msg.glovesData[0].fingersFlex[3].Joint1Stretch > 0.3:
    pose_vel.twist_linear_x = -0.1
  if msg.glovesData[0].fingersFlex[3].Joint1Stretch < -0.3:
    pose_vel.twist_linear_x = 0.1
        # CURRENT_VELOCITY[3] = 1 * dr #x: end effector self rotate,positive is shun
        # CURRENT_VELOCITY[4] = 1 * dp #y: up and down rotate (positive is down)
        # CURRENT_VELOCITY[5] = 1 * dyaw  #z: left and right rotate,positive is left
  pub.publish(pose_vel)

def main():
  global sub, pub
  rospy.init_node("prime_kinova_control")
  #init Subscriber and Publisher
  sub = rospy.Subscriber("GlovesData", GlovesData, callBack)
  pub = rospy.Publisher("/servo_server/delta_twist_cmds", PoseVelocity, queue_size=1)#j2n6s300_driver/in/cartesian_velocity

  rospy.spin()

if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
