#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
import rospy
import numpy as np
import math
import matplotlib.pyplot as plt


from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry

import sys, select, os


# Control Constant
HIGH_VEL = 0.015
TH_VEL = 0.001
MAX_STEER = 0.3
POWER_AC = 500
CONTROL_AC = 50

    
def save_key(msg):

    print("callback_key")


    
def save(msg):

    print("callback")
    #while not rospy.is_shutdown():
    rate = rospy.Rate(100) # 100hz
    
    current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    #f.write(current_pos)
    history_x.append(current_pos[0])
    history_y.append(current_pos[1])

    if len(history_x) > 1:
	delta_dist.append(cal_dist(history_x[-2], history_y[-2], history_x[-1], history_y[-1]) )
    print("now point : [ {} , {} ]".format(current_pos[0], current_pos[1]) )
    
    rate.sleep()


if __name__=="__main__":

    rospy.init_node('control_keyboard')
    
    # sub - car info , key
    rospy.Subscriber('key',String, save_key)
    #rospy.Subscriber('ground_truth/state', Odometry, save)
    rospy.spin()
    # pub - car control
    fr_pub = rospy.Publisher('parkingdemo/fr_Steer_position_controller/command', Float64, queue_size=10)
    fl_pub = rospy.Publisher('parkingdemo/fl_Steer_position_controller/command', Float64, queue_size=10)
    rr_pub = rospy.Publisher('parkingdemo/rr_Wheel_effort_controller/command', Float64, queue_size=10)
    rl_pub = rospy.Publisher('parkingdemo/rl_Wheel_effort_controller/command', Float64, queue_size=10)

    # status = 0 ~ stop  ,  status = 1 ~ run
    status = 0
    steer = 0
    wheel_effort = 0

    # publish ros topic
    print("publish")
    fr_pub.publish(steer)
    fl_pub.publish(steer)
    rr_pub.publish(wheel_effort)
    rl_pub.publish(wheel_effort)

         
           
  
