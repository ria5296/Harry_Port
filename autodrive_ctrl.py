#! /usr/bin/env python
# -*- coding: utf-8 -*-

##################################################################################
# This code control delivery robot to move along with delivery server.           #
# If you want room points' coordinates, modify under lines.                      #
# home_point, room1_poinnt, room2_point, room3_point, room4_point                #
# If you want to add other rooms, add lines for them, and modify point_array.    #
#                                                                                #
# to launch this code  : ~$ roslaunch delivery delivery_ctrl.launch              #
# to test start room   : ~$ rostopic pub -1 /start_room std_msgs/Int8 1          #
# to test end room     : ~$ rostopic pub -1 /end_room std_msgs/Int8 3            #
# to test start_check  : ~$ rostopic pub -1 /start_check std_msgs/Bool True      #
# to test basket_check : ~$ rostopic pub -1 /basket_check std_msgs/Bool True     #
##################################################################################

import rospy
import numpy as np
import actionlib
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, Pose
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8, Bool

x_meter, y_meter = (0, 0)   # initiate start point
start_room = 0              # start room's number(or ID)
end_room = 0                # end room's number(or ID)
start_check = False         # start order check, True=can go to next place
basket_check = False        # basket open check, True=can open basket
is_reached = 0              # if reached at goal, this changes to 1, else 0

# home & room points(x_meter, y_meter)   -> 헤딩각 추가 가능
home_point = (-2.00, 0.00)
room1_point = (2.00, 1.00)
room2_point = (2.00, -1.00)
room3_point = (-0.75, -1.75)
room4_point = (-0.75, 1.75)
point_array = [home_point, room1_point, room2_point, room3_point, room4_point]

# Subscriber's callback function for get amcl_pose
def getAmclPose(data):
    global x_meter, y_meter
    x_meter = data.pose.pose.position.x
    y_meter = data.pose.pose.position.y

# Subscriber's callback function for get start room
def getStartRoom(data):
    global start_room
    start_room = data.data

# Subscriber's callback function for get end room
def getEndRoom(data):
    global end_room
    end_room = data.data

# Subscriber's callback function for get navigation status
def getNavStatus(data):
    global is_reached
    if data.status.status == 3:
        is_reached = 1

# Subscriber's callback functions for get start check
def getStartCheck(data):
    global start_check
    if data.data == True:
        start_check = True

# function for publishing navigation goal
def pubNavGoal(pose):
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    client.wait_for_server()
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = Pose(Point(pose[0], pose[1], 0), Quaternion(0,0,0,1))
    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available or goal could not be reached")
        rospy.signal_shutdown("Action server not available or goal could not be reached")  

# function for waiting is_start
def waitStarting():
    global start_check
    rospy.loginfo("waiting for starting order")
    while not start_check: pass
    start_check = False
    rospy.loginfo("success\n")
    is_reached = 0

# initiate node
rospy.init_node("delivery_ctrl_node")
r = rospy.Rate(10)

#subscriber and publisher
rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, getAmclPose, queue_size=1)
rospy.Subscriber("/start_room", Int8, getStartRoom, queue_size=1)
rospy.Subscriber("/end_room", Int8, getEndRoom, queue_size=1)
rospy.Subscriber("/move_base/result", MoveBaseActionResult, getNavStatus, queue_size=1)
rospy.Subscriber("/start_check", Bool, getStartCheck, queue_size=1)
rospy.Subscriber("/basket_check", Bool, getBasketCheck, queue_size=1)

# main loop
while True:

    # check delivery rooms
    if start_room != 0 and end_room != 0:
        print("\n"+"="*28+"start delivery"+"="*28)
        # start/end room number check
        rospy.loginfo("start room is '%d'" % (start_room))
        rospy.loginfo("  end room is '%d'\n" % (end_room))
    
        # go to start room
        rospy.loginfo("now go to start room, room%d" % start_room)
        pubNavGoal(point_array[start_room])
        while True:
            if is_reached == 1:
                rospy.loginfo("now reached at start room")
                break
        waitStarting()
        
        # go to end room
        rospy.loginfo("now go to end room, room%d" % end_room)
        pubNavGoal(point_array[end_room])
        while True:
            if is_reached == 1:
                rospy.loginfo("now reached at end room")
                break
        waitStarting()
        
        # go to home
        rospy.loginfo("now go to home")
        pubNavGoal(point_array[0])
        while True:
            if is_reached == 1:
                rospy.loginfo("now reached at home")
                break
        is_reached = 0
        start_room = 0
        end_room = 0
        
        print("="*70+"\n")
        rospy.loginfo("waiting delivery call")
        
    r.sleep()
