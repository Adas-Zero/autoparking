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

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import sys, select, os
import math
import matplotlib.pyplot as plt

# Control Constant
HIGH_VEL = 0.015
TH_VEL = 0.001
MAX_STEER = 0.3
POWER_AC = 1500
CONTROL_AC = 100
lamda = 15

history_x = []
history_y = []
delta_dist = []


# pub - car control
fr_pub = rospy.Publisher('parkingdemo/fr_Steer_position_controller/command', Float64, queue_size=10)
fl_pub = rospy.Publisher('parkingdemo/fl_Steer_position_controller/command', Float64, queue_size=10)
rr_pub = rospy.Publisher('parkingdemo/rr_Wheel_effort_controller/command', Float64, queue_size=10)
rl_pub = rospy.Publisher('parkingdemo/rl_Wheel_effort_controller/command', Float64, queue_size=10) 


# vel
history_x = []
history_y = []
delta_dist = [0]

# robot direction
dirc = [0,0]
STEER = [0]

vel = [0,0]
key = ['']





# convert Quaternion to Euler
def convert_Quar_to_Euler(x,y,z,w):

    pi = math.atan2( 2*(x*y + z*w) , 1-2*(y*y+z*z) )
    #theta = math.asin( 2*(x*z - w*y) )
    #lamda = math.atan2( 2*(x*w + y*z), (1-2*(z*z+w*w)) )
    
    return pi


# key callback func
def save_key(msg):

    key[0] = msg.data



# odom callback func     
def save_odom(msg):
   
    steer = 0
    wheel_effort = 0.0

    rate = rospy.Rate(100) # 100hz
    
    current_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
    print("now point : [ {} , {} ]".format(current_pos[0], current_pos[1]) )

    PI = convert_Quar_to_Euler(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    
    dirc[0] = math.cos(PI)
    dirc[1] = math.sin(PI)

    history_x.append(current_pos[0])
    history_y.append(current_pos[1])

    if len(history_x) > 1:
        vel[0] = history_x[-1] - history_x[-2]
        vel[1] = history_y[-1] - history_y[-2]
        v = math.sqrt(vel[0]*vel[0] + vel[1]*vel[1])
        delta_dist.append(v)
    
 
    if key[0] == 'w':
        print("1st : {}, 2nd : {} , 3rd : {}".format(HIGH_VEL, HIGH_VEL-v, (HIGH_VEL-v)/HIGH_VEL ))
	wheel_effort = (HIGH_VEL-v)/HIGH_VEL*POWER_AC

    elif key[0] == 's':
	wheel_effort = -(HIGH_VEL-v)/HIGH_VEL*POWER_AC

    elif key[0] == 'a':
        
	steer = MAX_STEER
        fr_pub.publish(steer)
        fl_pub.publish(steer)
        print("Steer", steer)

    elif key[0] == 'd':
	steer = -MAX_STEER
        fr_pub.publish(steer)
        fl_pub.publish(steer)
        print("Steer", steer)

    elif key[0] == '2':
	steer = 0
        fr_pub.publish(steer)
        fl_pub.publish(steer)
        print("Steer", steer)

    elif key[0] == '':
        flag = vel[0]*dirc[0] + vel[1]*dirc[1]
        if flag > 0:
	  wheel_effort = (0-v)*lamda*POWER_AC 
	else:
	  wheel_effort = (v-0)*lamda*POWER_AC 	  
	

    print("wheel_effort : {} , vel : {}".format(wheel_effort, v))
    # pub 
    rr_pub.publish(wheel_effort)
    rl_pub.publish(wheel_effort)

    rate.sleep()



if __name__=="__main__":

    rospy.init_node('control_keyboard')
    


    # sub - car info , key
    rospy.Subscriber('key',String, save_key)
    rospy.Subscriber('ground_truth/state',Odometry, save_odom)


    # status = 0 ~ stop  ,  status = 1 ~ run
    #status = 0
    #steer = 0
    #wheel_effort = 0
    rospy.spin()

    print("finish spin")
    #plt.plot(history_x, history_y)
    plt.plot(delta_dist)
    plt.show()

    plt.plot(history_x, history_y)
    plt.show()


           
  
