#!/usr/bin/env python3

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
import sys, select, os
import gym
import numpy as np
import random

if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

# Control Status
STOP = 0
FORWARD = 1
BACKWARD = 2
RIGHT = 3
LEFT = 4

# Control Constant
MAX_VEL = 0.015
TH_VEL = 0.001
MAX_STEER = 0.3
POWER_AC = 500
CONTROL_AC = 50

msg = """
fuck
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/s : increase/decrease REAR Wheel_effort 
a/d : increase/decrease FRONT Steer 

space key, X : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def getKey():
    if os.name == 'nt':
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class AutoParkEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

        rospy.init_node('parking_demo_teleop')

        self.fr_pub = rospy.Publisher('parkingdemo/fr_Steer_position_controller/command', Float64, queue_size=10)
        self.fl_pub = rospy.Publisher('parkingdemo/fl_Steer_position_controller/command', Float64, queue_size=10)
        self.rr_pub = rospy.Publisher('parkingdemo/rr_Wheel_effort_controller/command', Float64, queue_size=10)
        self.rl_pub = rospy.Publisher('parkingdemo/rl_Wheel_effort_controller/command', Float64, queue_size=10)
        # status = 0 ~ stop  ,  status = 1 ~ run
        self.status = 0
        self.steer = 0
        self.wheel_effort = 0
        self.vel = 0

    def step(self, action):
        if action == STOP:
            pass
        elif action == FORWARD:
            self.wheel_effort += 10
        elif action == BACKWARD:
            self.wheel_effort -= 10
        elif action == RIGHT:
            self.steer -= 0.1
        elif action == LEFT:
            self.steer += 0.1
        # elif action == FORWARD:
        #     if self.vel - MAX_VEL > TH_VEL:
        #         self.wheel_effort = -CONTROL_AC
        #     else:
        #         self.wheel_effort = self.vel / MAX_VEL * POWER_AC
        #
        # elif action == BACKWARD:
        #     if self.vel - MAX_VEL > TH_VEL:
        #         self.wheel_effort = CONTROL_AC
        #     else:
        #         self.wheel_effort = -self.vel / MAX_VEL * POWER_AC
        #
        # elif action == RIGHT:
        #     self.steer = MAX_STEER
        #
        # elif action == LEFT:
        #     self.steer = -MAX_STEER

        self.status += 1
        self.fr_pub.publish(self.steer)
        self.fl_pub.publish(self.steer)
        self.rr_pub.publish(self.wheel_effort)
        self.rl_pub.publish(self.wheel_effort)
        print("Wheel_effort : {} , Steer : {} ".format(self.wheel_effort, self.steer))

        state = np.array([self.wheel_effort, self.steer, self.status])
        reward = self.status * -0.001
        done = 0
        return state, reward, done, {}

    def reset(self):
        self.status = 0
        self.steer = 0
        self.wheel_effort = 0

    def render(self, modes='human', close=False):
        pass


if __name__ == "__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # turtlebot3_model = rospy.get_param("model", "burger")

    env = AutoParkEnv()
    env.reset()
    try:
        print(msg)
        while (1):
            key = getKey()
            if key=='w':
                state, reward, done, info = env.step(FORWARD)
            elif key=='a':
                state, reward, done, info = env.step(LEFT)
            elif key == 's':
                state, reward, done, info = env.step(BACKWARD)
            elif key == 'd':
                state, reward, done, info = env.step(RIGHT)
                if done:
                    break
                # constant time , inform
                if state[2] == 20:
                    print(msg)
                    state = 0

    # publish ros topic

    except:
        print(e)

    finally:
        print("final")

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
