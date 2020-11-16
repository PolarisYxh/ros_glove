#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import time
import json
import socket 
import threading

import rospy 
from kinova_msgs.msg import PoseVelocity, JointVelocity
from primeii_ros_msgs.msg import FingerFlex, GloveData, GlovesData

sub = None
pub = None

fingerNames = [ "thumb", "index", "middle", "ring", "pinky" ]

def callBack(msg):
  #pose_vel = PoseVelocity()
  joint_vel = JointVelocity()
  if msg.glovesData[0].fingersFlex[1].Joint1Stretch > 0.3:
    #pose_vel.twist_linear_z = -0.1
    joint_vel.joint1 = -30
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[1].Joint1Stretch < -0.3:
    #pose_vel.twist_linear_z = 0.1
    joint_vel.joint1 = 30
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[2].Joint1Stretch > 0.3:
    #pose_vel.twist_linear_y = -0.1
    joint_vel.joint2 = -30
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[2].Joint1Stretch < -0.3:
    #pose_vel.twist_linear_y = 0.1
    joint_vel.joint2 = 30
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[3].Joint1Stretch > 0.3:
    #pose_vel.twist_linear_x = -0.1
    joint_vel.joint3 = -40
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[3].Joint1Stretch < -0.3:
    #pose_vel.twist_linear_x = 0.1
    joint_vel.joint3 = 40
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[4].Joint1Stretch > 0.3:
    #pose_vel.twist_linear_x = -0.1
    joint_vel.joint4 = -60
    pub.publish(joint_vel)
  if msg.glovesData[0].fingersFlex[4].Joint1Stretch < -0.3:
    #pose_vel.twist_linear_x = 0.1
    joint_vel.joint4 = 60
    pub.publish(joint_vel)


def main():
  global sub, pub
  rospy.init_node("prime_kinova_control")
  #init Subscriber and Publisher
  sub = rospy.Subscriber("GlovesData", GlovesData, callBack)
  pub = rospy.Publisher("/j2n6s300_driver/in/joint_velocity", JointVelocity, queue_size=1)
  
  rospy.spin()
  
if __name__ == "__main__":
  try:
    main()
  except rospy.ROSInterruptException:
    pass
