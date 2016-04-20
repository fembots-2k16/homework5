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
from nav_msgs.msg import Odometry
from frontier_exploration.msg import ExploreTaskAction, ExploreTaskGoal, ExploreTaskActionResult
from p2os_msgs.msg import MotorState

rate = None
goal_client = None
exploration_client = None
explore_status = None
goal_status = 0
interrupt_exploration = False
found_ids = {}
robot_pose = None

def moveBaseActionResultHandler(data):
    global goal_status
    goal_status = data.status.status
    #print "status: ", status
    #print data.result
    #http://docs.ros.org/fuerte/api/actionlib_msgs/html/msg/GoalStatus.html

def exploreTaskActionResultHandler(data):
    global explore_status
    explore_status = data.status.status

def tagDetectionsHandler(data):
    global found_ids, robot_pose, goal_status, rate, interrupt_exploration, exploration_client

    detections = data.detections

    for detection in detections:
        id = detection.id
        if id not in found_ids:
            print "we found an april tag!"
            size = detection.size
            #tag detections pose
            april_pose = detection.pose

            interrupt_exploration = True
            print "interrupt exploration from april tags handler"
            exploration_client.cancel_all_goals()

            #MATH for finding global point from april tag (shout out to ryan)
            #relative tag posiiton
            april_x = april_pose.position.x
            april_z = april_pose.position.z

            angle = math.asin(april_x / april_z)
            x_diff = april_z * math.cos(angle)
            y_diff = april_z * math.sin(angle)
            #global tag position
            april_x = robot_pose.position.x + x_diff
            april_y = robot_pose.position.y + y_diff

            goal_x = april_x + 0.5*math.cos(-angle)
            goal_y = april_y + 0.5*math.sin(-angle)
            goal_z_theta = -angle

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.get_rostime()

            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y
            goal.target_pose.pose.orientation.z = goal_z_theta

            goal_client.send_goal(goal)
            print "sending goal and waiting"

            #wait for the goal to finish!!!
            goal_client.wait_for_result()

            interrupt_exploration = False
            startExploration()

def odometryHandler(data):
    global robot_pose
    robot_pose = data.pose.pose

def startExploration():
    global goal_status, exploration_client, rate, interrupt_exploration
    if interrupt_exploration:
        return
    exploration_goal = ExploreTaskGoal()
    exploration_goal.explore_boundary.header.seq = 1
    exploration_goal.explore_boundary.header.frame_id = "map"
    exploration_goal.explore_center.point.x = 1
    exploration_goal.explore_center.point.y = 0
    exploration_goal.explore_center.point.z = 0

    exploration_client.send_goal(exploration_goal)
    print "sent the exploration goal... waiting..."
    print "interrupt_exploration", interrupt_exploration
    print "explore_status", explore_status
    while (explore_status == 0 or explore_status == 1) and not interrupt_exploration:
        rate.sleep()
    print "exploration goal 'complete'"
    exploration_client.cancel_all_goals()


def main():
    global goal_client, exploration_client, rate, status
    rospy.init_node('homework5_navigator')
    rate = rospy.Rate(10)

    rospy.Subscriber("/odomOdometry", Odometry, odometryHandler)
    rospy.Subscriber("/move_base/result", MoveBaseActionResult, moveBaseActionResultHandler)
    rospy.Subscriber("/explore_server/result", ExploreTaskActionResult, exploreTaskActionResultHandler)
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, tagDetectionsHandler)


    velPub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    motPub = rospy.Publisher("/cmd_motor_state", MotorState, queue_size=10)

    motor = MotorState()
    motor.state = 1

    print "starting motors..."
    for i in xrange(20):
        motPub.publish(motor)
        rate.sleep()

    #goal_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    exploration_client = actionlib.SimpleActionClient("explore_server", ExploreTaskAction)

    #Waits until the action server has started up and started listening for goals
    #goal_client.wait_for_server()
    print "waiting for the exploration server..."
    exploration_client.wait_for_server()

    #move it forward first to appease the lord almighty DWA planner
    # print "move forward."
    # twist = Twist()
    # twist.linear.x = 0.3
    #
    # for i in xrange(20):
    #     velPub.publish(twist)
    #     rate.sleep()
    # #now stop!
    # print "stop."
    # for i in xrange(20):
    #     velPub.publish(Twist())
    #     rate.sleep()


    #TODO
    time = 0
    three_minutes = 3
    # MAIN LOOP
    print "exploration!"
    while time < three_minutes:
        startExploration()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
