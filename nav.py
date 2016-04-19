#!/usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Twist
from p2os_msgs.msg import MotorState
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray
from apriltags_ros.msg import AprilTagDetectionArray

goal_status = 0
found_ids = {}

def moveBaseActionResultHandler(data):
    global goal_status
    goal_status = data.status.goal_id
    #print "status: ", status
    #print data.result
    #http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html
    pass

def tagDetectionsHandler(data):
    global found_ids
    detections = data.detections

    for detection in detections:
        id = detection.id
        if id in found_ids
        size = detection.size
        pose = detection.pose

def main():
    global point
    global status
    print 'what... are your ending coordinates?'
    rospy.init_node('homework4_navigator')
    rate = rospy.Rate(10)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, moveBaseActionResultHandler)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tagDetectionsHandler)

    while (point == None):
       rate.sleep()

    #TODO::
    x = 0
    y = 0


    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    print "make sure you start the motors!"

    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    #Waits until the action server has started up and started listening for goals
    client.wait_for_server()

    #TODO
    time = 0
    three_minutes = 3
    # MAIN LOOP
    while time < three_minutes:
        #NAVIGATE (USING FRONTIER EXPLORATION?)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
