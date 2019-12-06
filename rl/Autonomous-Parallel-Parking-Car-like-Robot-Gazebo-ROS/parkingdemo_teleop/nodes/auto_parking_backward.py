#!/usr/bin/env python


######################################
##       AUTOPARKING SIMULATOR      ##
#########################################___made by: Park Sung Hun / bbuglebbugle@naver.com
import roslib
import rospy
import sys
import cv2
import math
import message_filters
import numpy as np
import time
import itertools

import select
import tty
import termios


from itertools import combinations
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

from adasone_msgs.msg import ObdParam
from adasone_msgs.msg import VehicleCmd

class NonBlockingConsole(object):
    def __enter__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    def get_data(self):
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return False




# initialize ROS node, named "AutoParking_project"
rospy.init_node('AutoParking_project', anonymous=False)
rospy.loginfo("To stop, press CTRL + C")

# global variable

FINISH_FLAG = 0
cv_window_name_back = 'Backward CAM01'
cv_window_name_back_ori = 'Backward CAM02'
cv_window_name_avm = 'AVM CAM01'
avm_frame = 0
frame = 0

left_distance = 0
left_ref_x = 0
right_distance = 0
right_ref_x = 0
FR = 0
FL = 0
FRW = 0 
FLW = 0
BRW = 0 
BLW = 0
BR = 0 
BL = 0

slope = 0
base_slope = 90

# vehicle control value
MAX_STEERING_WHEEL = 540.0
MID_STEERING_WHEEL = 270.0
MAX_SPEED = 0.5
MID_SPEED = 0.25

GEAR_POSITION = 0

GOAL_STEERING_WHEEL = 0
PRESENT_STEERING_WHEEL = 0

# sensor configuration
FRONT_DETECT_RANGE = 50
SIDE_DETECT_RANGE = 50
BACK_DETECT_RANGE = 50

### avm image coordination

# vehicle coordination
FRONT_L_X = 86
FRONT_L_Y = 12
FRONT_R_X = 168
FRONT_R_Y = 12
BEHIND_L_X = 84
BEHIND_L_Y = 150
BEHIND_R_X = 168
BEHIND_R_Y = 150
MID_X = FRONT_L_X + (FRONT_R_X - FRONT_L_X)/2
MID_Y = FRONT_L_Y + (BEHIND_L_Y - FRONT_L_Y)/2

# parking slot coordination
SLOT_X = 0
SLOT_Y = 0

def houghTransform(image, edges, mode):
    global FINISH_FLAG
    global slope
    global base_slope
    rho = 2
    theta = np.pi/180
    threshold = 30
    min_line_length = 30
    max_line_gap = 50

    min_distance = 9999

    line_image = np.copy(image)*0 #creating a blank to draw lines on

    # Run Hough on edge detected image
    if mode == 0:
        avm_lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
    elif mode == 1:
        back_lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
    
   

    if mode == 0 and avm_lines is not None:
        # Iterate over the output "lines" and draw lines on the blank
        for line in avm_lines:
            for x1,y1,x2,y2 in line:
                if (x1 >= FRONT_L_X and x1 <= FRONT_R_X and y1 >= FRONT_L_Y and y1 <= BEHIND_R_Y) or (x2 >= FRONT_L_X and x2 <= FRONT_R_X and y2 >= FRONT_L_Y and y2 <= BEHIND_R_Y):
                    continue
                else:
                    distance = math.sqrt((y2-y1)**2+(x2-x1)**2)
                    if distance >= 80:
                        continue
                    line_slope = math.atan2(y1-y2,x2-x1)*(180/math.pi)
                    if line_slope < 0.0:
                        line_slope += 180.0


                    #cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),1)
                    if line_slope >= 60 and line_slope <= 130:
                        cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),2)
                        cv2.circle(line_image,(x1,y1),2,(0,0,255),1)
                        cv2.circle(line_image,(x2,y2),2,(0,0,255),1)
                        slope = line_slope

                        #print slope, y2,'-',y1,'/',x2,'-',x1



        # Create a "color" binary image to combine with line image
        color_edges = np.dstack((edges, edges, edges))

        # Draw the lines on the edge image
        combo = cv2.addWeighted(color_edges, 0.8, line_image, 1, 0)

        cv2.line(combo, (MID_X,MID_Y),(MID_X,MID_Y - 30),(255,120,30),1)
        return combo

    if mode == 1 and back_lines is not None:
        remain_distance = 0
        finish_line_slope = 0
        parking_line_x1 = 0     
        parking_line_x2 = 0
        parking_line_y1 = 0
        parking_line_y2 = 0
        
        for line in back_lines:
            for x1,y1,x2,y2 in line:
                if y1 >= 225 or y2 >= 225:
                    continue
                else:
                    finish_line_slope = math.atan2(y1-y2,x2-x1)*(180/math.pi)
                    if finish_line_slope < 0.0:
                        finish_line_slope += 180.0
                    if (finish_line_slope >= 0 and finish_line_slope <= 10) or (finish_line_slope >= 170 and finish_line_slope <= 180):
                        if (y1+y2)/2 <= min_distance:
                            min_distance = (y1+y2)/2
                            remain_distance = min_distance
                            parking_line_x1 = x1
                            parking_line_x2 = x2
                            parking_line_y1 = y1
                            parking_line_y2 = y2
                



        cv2.line(line_image,(parking_line_x1,parking_line_y1),(parking_line_x2,parking_line_y2),(255,0,255),2)
        cv2.circle(line_image,(parking_line_x1,parking_line_y1),2,(0,255,0),1)
        cv2.circle(line_image,(parking_line_x2,parking_line_y2),2,(0,255,0),1)

        finish_slope = finish_line_slope
        # Create a "color" binary image to combine with line image
        color_edges = np.dstack((edges, edges, edges))

        # Draw the lines on the edge image
        combo = cv2.addWeighted(color_edges, 0.8, line_image, 1, 0)

        if remain_distance >= 185:
            FINISH_FLAG = 1

        else:
            FINISH_FLAG = 0


        return combo

    return edges 


def callback_sonar1(msg):
    global FRW
    global SIDE_DETECT_RANGE

    sonar1_data = msg.ranges
    if sonar1_data <= SIDE_DETECT_RANGE:
        FRW = 1
    else:
        FRW = 0

def callback_sonar2(msg):
    global FLW
    global SIDE_DETECT_RANGE

    sonar2_data = msg.ranges
    if sonar2_data <= SIDE_DETECT_RANGE:
        FLW = 1
    else:
        FLW = 0

def callback_sonar3(msg):
    global BR
    global BACK_DETECT_RANGE

    sonar3_data = msg.ranges
    if sonar3_data <= BACK_DETECT_RANGE:
        BR = 1
    else:
        BR = 0

def callback_sonar4(msg):
    global BL
    global BACK_DETECT_RANGE

    sonar4_data = msg.ranges
    if sonar4_data <= BACK_DETECT_RANGE:
        BL = 1
    else:
        BL = 0

def callback_image(back_cam):#, avm_cam):

    global PRESENT_STEERING_WHEEL
    global frame
    #global avm_frame
    global left_distance
    global right_distance
    global left_ref_x 
    global right_ref_x 

    cv2.namedWindow(cv_window_name_back, cv2.WINDOW_NORMAL)
    #cv2.namedWindow(cv_window_name_avm, cv2.WINDOW_NORMAL)
    cv2.namedWindow(cv_window_name_back_ori, cv2.WINDOW_NORMAL)

    # convert data format(to opencv's mat)
    bridge = CvBridge()
    try:
        frame = bridge.imgmsg_to_cv2(back_cam, "bgr8")
    except CvBridgeError, e:
        print e
    

    left_min_val = 99999
    right_min_val = 99999
    #----------------------------
    # finding width of parking space
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9,9), 0)
    back_edge = cv2.Canny(gray, 50, 150)

    for x in range(0, 319, 1):
        if x >= 1 and x <= 319:
            tmp_left_distance = abs(160 - x)
            tmp_right_distance = abs(x - 160)
            if back_edge[175][x-1] >= 150 and back_edge[175][x] <= 100:
                if tmp_left_distance <= left_min_val and x <= 160:
                    left_min_val = tmp_left_distance
                    left_ref_x = x
                    #cv2.circle(frame,(left_ref_x,100),2,(0,255,0),-1)
                    left_distance = tmp_left_distance

            elif back_edge[175][x] <= 100 and back_edge[175][x+1] >= 150:
                if tmp_right_distance <= right_min_val and x >= 160:
                    right_min_val = tmp_right_distance
                    right_ref_x = x
                    #cv2.circle(frame,(right_ref_x,100),2,(0,0,255),-1)
                    right_distance = tmp_right_distance
    #---------------------------
    back_result = houghTransform(frame, back_edge, 1)
    cv2.circle(frame,(left_ref_x,175),2,(0,255,0),-1)
    cv2.circle(frame,(right_ref_x,175),2,(0,0,255),-1)



    cv2.line(frame, (160,150),(160,180),(255,0,0),3)

    frame = np.array(frame, dtype=np.uint8)    
    #avm_frame = np.array(avm_frame, dtype=np.uint8)
    cv2.imshow(cv_window_name_back, back_result)
    #cv2.imshow(cv_window_name_avm, avm_result)
    cv2.imshow(cv_window_name_back_ori, frame)
    cv2.waitKey(1)
    '''
    bridge2 = CvBridge()
    try:
        avm_frame = bridge2.imgmsg_to_cv2(avm_cam, "bgr8")
    except CvBridgeError, e:
        print e

    # finding vertical parking line
    gray2 = cv2.GaussianBlur(avm_frame,(9,9),0)
 
    edge = cv2.Canny(gray2, 200, 220)
    
    avm_result = houghTransform(avm_frame, edge, 0)

    cv2.circle(avm_result,(MID_X,MID_Y),3,(255,120,30),-1)
    avm_result = cv2.putText(avm_result,'steering wheel :',(32,250),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),1,cv2.LINE_AA)
    temp = "%0.1f"%GOAL_STEERING_WHEEL
    avm_result = cv2.putText(avm_result,temp,(200,250),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,255),1,cv2.LINE_AA)
    '''




def callback_obd(msg):
    global PRESENT_STEERING_WHEEL
    global GEAR_POSITION
    PRESENT_STEERING_WHEEL = msg.steering_angle # 0.7 * 771 = 540(degrees)
    GEAR_POSITION = msg.gear_pos
    # 0(P) / 1(R) / 2(N) / 3(D)



''' Definition for Publisher '''
pub_cmd_vel = rospy.Publisher('vehicle_cmd',VehicleCmd,queue_size=10)

''' Definition for Subscriber '''
##ULTRASONIC
#sub_sensor = rospy.Subscriber('scan',LaserScan,callback_sensor)
sub_sonar1 = rospy.Subscriber('sonar1',LaserScan,callback_sonar1)
sub_sonar2 = rospy.Subscriber('sonar2',LaserScan,callback_sonar2)
sub_sonar3 = rospy.Subscriber('sonar3',LaserScan,callback_sonar3)
sub_sonar4 = rospy.Subscriber('sonar4',LaserScan,callback_sonar4)


##CAMERA
#sub_image = message_filters.Subscriber('front_image1',Image)
sub_image = rospy.Subscriber('front_image1',Image, callback_image)
#sub_avm_image = message_filters.Subscriber('avm_cam',Image)
#ts = message_filters.TimeSynchronizer([sub_image, sub_avm_image], 10)
#ts.registerCallback(callback_image)


##HARDWARE
sub_obd = rospy.Subscriber('obd_param',ObdParam, callback_obd)





# module for detecting parking slot is required, at here, substituted as key input
nothing = input()

print 'put GEAR stick at D'
while 1:
    if GEAR_POSITION == 3: #DRIVING_POS
        break
    if input() is not None:
        break
max_time_end = time.time() + 5
while 1:
    move_cmd = VehicleCmd()
    move_cmd.steering = 0.0
    move_cmd.velocity = MAX_SPEED
    GOAL_STEERING_WHEEL = move_cmd.steering


    pub_cmd_vel.publish(move_cmd)
    if time.time() > max_time_end:
        print 'Auto-parking process ON'
        break



print 'put GEAR stick at R'
while 1:
    if GEAR_POSITION == 1: #REVERSE_POS
        break
    if input() is not None:
        break

max_time_end = time.time() + 6


with NonBlockingConsole() as nbc:

    while 1: # first step : go backward until detecting the obstacles at behind
        move_cmd = VehicleCmd()
        move_cmd.steering = -MAX_STEERING_WHEEL
        move_cmd.velocity = -MAX_SPEED

        if time.time() > max_time_end:
	    print 'Comparing Angle check ON'
	    if abs(90 - slope) <= 30:
	        print 'Comparing Angle check OFF'
	        break


        if BR == 1 or BL == 1:
	    print '[WARN, behind] object detected!!! [BR BL] = ',BR,BL
	    break
        else:
	    GOAL_STEERING_WHEEL = move_cmd.steering
	    '''
	    move_cmd.velocity = 0.0
	    pub_cmd_vel.publish(move_cmd)
	    while abs(PRESENT_STEERING_WHEEL - move_cmd.steering) >= qq:
	    move_cmd.velocity = -MAX_SPEED
	    '''

	    pub_cmd_vel.publish(move_cmd)
        if nbc.get_data() == '\x1b': #ESC
            break
    

print 'put GEAR stick at D'
while 1:
    if GEAR_POSITION == 3: #DRIVING_POS
        break
    if input() is not None:
        break
max_time_end = time.time() + 3.5

with NonBlockingConsole() as nbc:

    while 1: # second step : go forward until detecting the obstacles at front
        move_cmd = VehicleCmd()
        move_cmd.steering = MAX_STEERING_WHEEL
        move_cmd.velocity = MID_SPEED

        if time.time() > max_time_end:
            print 'Comparing Angle check ON'
            if abs(90 - slope) <= 5:
                print 'Comparing Angle check OFF'
                print 'slope is ',slope
                break
        if FR == 1 or FL == 1:
            print '[WARN, front] object detected!!! [FR FL] = ',FR,FL
            break
        else:
            GOAL_STEERING_WHEEL = move_cmd.steering        
            '''
            move_cmd.velocity = 0.0
            pub_cmd_vel.publish(move_cmd)
            while abs(PRESENT_STEERING_WHEEL - move_cmd.steering) >= qq:
            move_cmd.velocity = MID_SPEED
            '''
            pub_cmd_vel.publish(move_cmd)
        if nbc.get_data() == '\x1b': #ESC
	    break

print 'put GEAR stick at R'
while 1:
    if GEAR_POSITION == 1: #REVERSE_POS
        break
    if input() is not None:
        break
while 1: # and go backward with correcting angles(using camera, compare L/R balance) 
    if left_distance < right_distance:
        move_cmd = VehicleCmd()
        move_cmd.steering = MAX_STEERING_WHEEL
        move_cmd.velocity = -MID_SPEED
    else:
        move_cmd = VehicleCmd()
        move_cmd.steering = -MAX_STEERING_WHEEL
        move_cmd.velocity = -MID_SPEED

    # this right/left_distance is calculated in camera view
    if abs(right_distance-left_distance) <= 2:
        print 'right_distance ',right_distance
        print 'left_distance ',left_distance
        print 'theta is arranged properly'
        break
    else:
        GOAL_STEERING_WHEEL = move_cmd.steering
        pub_cmd_vel.publish(move_cmd)

toggle = False
while 1: 
    if toggle == True:
        print 'put GEAR stick at R'
        toggle = False
        while 1:
            if GEAR_POSITION == 1:
                break
            if input() is not None:
                break
    move_cmd = VehicleCmd()
    move_cmd.velocity = -MAX_SPEED
    move_cmd.steering = 0.0
###
    #if BR == 1 or BL == 1:
    #    print 'Line 312, [BR BL] = ',BR,BL
    #    max_time_end = time.time() + 1.5
    #    if BL == 1:
    #        print 'BL == 1'
    #        while 1:
    #            if time.time() > max_time_end:
    #                print 'escape finished, retry...'
    #                while abs(90 - slope) >= 5:
    #                    if slope > 90:
    #                        move_cmd = VehicleCmd()
    #                        move_cmd.steering = MAX_STEERING_WHEEL
    #                        move_cmd.velocity = MID_SPEED
    #                        pub_cmd_vel.publish(move_cmd)
    #                    else:
    #                        move_cmd = VehicleCmd()
    #                        move_cmd.steering = -MAX_STEERING_WHEEL
    #                        move_cmd.velocity = MID_SPEED
    #                        pub_cmd_vel.publish(move_cmd)
    #                break
    #            move_cmd = VehicleCmd()
    #            move_cmd.velocity = MID_SPEED
    #            move_cmd.steering = -MAX_STEERING_WHEEL
    #            pub_cmd_vel.publish(move_cmd)
    #    elif BR == 1:
    #        print 'BR == 1'
    #        while 1:
    #            if time.time() > max_time_end:
    #                print 'escape finished, retry...'
    #                while abs(90 - slope) >= 5:
    #                    if slope > 90:
    #                        move_cmd = VehicleCmd()
    #                        move_cmd.steering = MAX_STEERING_WHEEL
    #                        move_cmd.velocity = MID_SPEED
    #                        pub_cmd_vel.publish(move_cmd)
    #                    else:
    #                        move_cmd = VehicleCmd()
    #                        move_cmd.steering = -MAX_STEERING_WHEEL
    #                        move_cmd.velocity = MID_SPEED
    #                        pub_cmd_vel.publish(move_cmd)
    #                break
    #            move_cmd = VehicleCmd()
    #            move_cmd.velocity = MID_SPEED
    #            move_cmd.steering = MAX_STEERING_WHEEL
    #            pub_cmd_vel.publish(move_cmd)
###
    #while abs(right_distance - left_distance) >= 6:
    #    if abs(90-slope) >= 2:    
    #        if left_distance < right_distance:
    #            move_cmd = VehicleCmd()
    #            move_cmd.steering = -MID_STEERING_WHEEL
    #            move_cmd.velocity = MID_SPEED
    #        else:
    #            move_cmd = VehicleCmd()
    #            move_cmd.steering = MID_STEERING_WHEEL
    #            move_cmd.velocity = MID_SPEED
    #            GOAL_STEERING_WHEEL = move_cmd.steering
    #        pub_cmd_vel.publish(move_cmd)
    #    else:
    #        break
    #print 'put GEAR stick at R'
    #while 1:
    #    if GEAR_POSITION == 1: #REVERSE_POS
    #        break
    #    if input() is not None:
    #        break   
    while abs(90 - slope) >= 2:
        #print '90 vs ', slope
        if slope > 90:
            move_cmd = VehicleCmd()
            move_cmd.steering = -MID_STEERING_WHEEL
            move_cmd.velocity = -MID_SPEED
        else:
            move_cmd = VehicleCmd()
            move_cmd.steering = MID_STEERING_WHEEL
            move_cmd.velocity = -MID_SPEED

        if BR^BL == 1:
            toggle = True
	    print 'put GEAR stick at D'
	    while 1:
                if GEAR_POSITION == 3: # DRIVING_POS
                    break
                if input() is not None:
                    break
            print 'Line 354, [BR BL] = ',BR,BL
            max_time_end = time.time() + 1.5
            if BL == 1:
                print 'BL == 1'
                while 1:
                    if time.time() > max_time_end:
                        print 'escape finished, retry...'
                        break
                    move_cmd = VehicleCmd()
                    move_cmd.velocity = MAX_SPEED
                    move_cmd.steering = -MID_STEERING_WHEEL


                    pub_cmd_vel.publish(move_cmd)
            elif BR == 1:
                print 'BR == 1'
                while 1:
                    if time.time() > max_time_end:
                        print 'escape finished, retry...'
                        break
                    move_cmd = VehicleCmd()
                    move_cmd.velocity = MAX_SPEED
                    move_cmd.steering = MID_STEERING_WHEEL

                    pub_cmd_vel.publish(move_cmd)
        GOAL_STEERING_WHEEL = move_cmd.steering
        pub_cmd_vel.publish(move_cmd)

    if FINISH_FLAG == 1 or (BR == 1 and BL == 1):
        break

    #----------------------------
    # at here i have to make a solution for the case(obstacle at behind, only one side)
    #----------------------------

    GOAL_STEERING_WHEEL = move_cmd.steering
    pub_cmd_vel.publish(move_cmd)






#while 1: # park finished, but when L/R is not balanced,
#    if abs(scan_value[85] - scan_value[265]) >= 0.25:
#        if scan_value[265] > scan_value[85]: # close to right side
#            while 1: # move left-forward until L/R is balanced
#                if abs(scan_value[85] - scan_value[265]) <= 0.15:
#                    break
#                else:
#                    move_cmd = VehicleCmd()
#                    move_cmd.velocity = MAX_SPEED
#                    move_cmd.steering = MID_STEERING_WHEEL
#                    PRESENT_STEERING_WHEEL = move_cmd.steering
#                    pub_cmd_vel.publish(move_cmd)
#            while 1: # move forward until there is enough space at left side
#                if scan_value[265] >= 2.0:
#                    break
#                else:
#                    move_cmd = VehicleCmd()
#                    move_cmd.velocity = MAX_SPEED
#                    move_cmd.steering = 0.0
#                    PRESENT_STEERING_WHEEL = move_cmd.steering
#                    pub_cmd_vel.publish(move_cmd)
#            while 1: # correcting angle
#                if abs(90 - slope) <= 3:
#                    print 'theta re-arranging success'
#                    while 1:
#                        move_cmd = VehicleCmd()
#                        move_cmd.velocity = -MAX_SPEED
#                        move_cmd.steering = 0.0
#
#                        if FINISH_FLAG == 1 or (BR == 1 and BL == 1):
#                            break
#                        PRESENT_STEERING_WHEEL = move_cmd.steering
#                        pub_cmd_vel.publish(move_cmd)
#                    break
#                else:
#                    move_cmd = VehicleCmd()
#                    move_cmd.velocity = MAX_SPEED
#                    move_cmd.steering = -MID_STEERING_WHEEL
#                    PRESENT_STEERING_WHEEL = move_cmd.steering
#                    pub_cmd_vel.publish(move_cmd)
#        else: # close to left side
#            while 1: # move right-forward until L/R is balanced
#                if abs(scan_value[85] - scan_value[265]) <= 0.15:
#                    break
#                else:
#                    move_cmd = VehicleCmd()
#                    move_cmd.velocity = MAX_SPEED
#                    move_cmd.steering = -MID_STEERING_WHEEL
#                    PRESENT_STEERING_WHEEL = move_cmd.steering
#                    pub_cmd_vel.publish(move_cmd)
#            while 1: # move forward until there is enough space at right side
#                if scan_value[85] >= 2.0:
#                    break
#                else:
#                    move_cmd = VehicleCmd()
#                    move_cmd.velocity = MAX_SPEED
#                    move_cmd.steering = 0.0
#                    PRESENT_STEERING_WHEEL = move_cmd.steering
#                    pub_cmd_vel.publish(move_cmd)
#            while 1: # correcting angle
#                if abs(90-slope) <= 3:
#                    print 'theta re-arranging success'
#                    while 1:
#                        move_cmd = VehicleCmd()
#                        move_cmd.velocity = -MAX_SPEED
#                        move_cmd.steering = 0.0
#
#                        if FINISH_FLAG == 1 or (BR == 1 and BL == 1):
#                            print 'there is an obstacle behind'
#                            break
#                        PRESENT_STEERING_WHEEL = move_cmd.steering
#                        pub_cmd_vel.publish(move_cmd)
#                    break
#                else:
#                    move_cmd = VehicleCmd()
#                    move_cmd.velocity = MAX_SPEED
#                    move_cmd.steering = MID_STEERING_WHEEL
#                    PRESENT_STEERING_WHEEL = move_cmd.steering
#                    pub_cmd_vel.publish(move_cmd)  
#    else:
#        print 'Auto parking process is finished'
#        break
###
