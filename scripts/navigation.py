#!/usr/bin/env python

"""

    RoboCup@Home Education | oc@robocupathomeedu.org
    navi.py - enable turtlebot to navigate to predefined waypoint location

"""

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

original = 0
start = 1

class NavToPoint:
    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        
	# Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")

        # A variable to hold the initial pose of the robot to be set by the user in RViz
        initial_pose = PoseWithCovarianceStamped()
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)

	# Get the initial pose from the user
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        
        # Make sure we have the initial pose
        while initial_pose.header.stamp == "":
        	rospy.sleep(1)
            
        rospy.loginfo("Ready to go")
	rospy.sleep(1)

	locations = dict()

	# Location A
	A_x = 0.95
	A_y = 5.62
	A_theta = 0.00247
	B_x = -0.577
	B_y = 5.59
	B_theta = -0.00253
	C_x = 1.03
	C_y = 8.32
	C_theta = -0.00143
	D_x = -0.547
	D_y = 8.37
	D_theta = -0.00143

	quaternion = quaternion_from_euler(0.0, 0.0, A_theta)
	locations['A'] = Pose(Point(A_x, A_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
	quaternion = quaternion_from_euler(0.0, 0.0, B_theta)
	locations['B'] = Pose(Point(B_x, B_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
	quaternion = quaternion_from_euler(0.0, 0.0, C_theta)
	locations['C'] = Pose(Point(C_x, C_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
	quaternion = quaternion_from_euler(0.0, 0.0, D_theta)
	locations['D'] = Pose(Point(D_x, D_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))

	self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")
	table = input("Enter table to go: ")


	while not rospy.is_shutdown():
	  self.goal.target_pose.header.frame_id = 'map'
	  self.goal.target_pose.header.stamp = rospy.Time.now()

	  # Robot will go to point A
	  if start == 1:
		if table == 1:
			rospy.loginfo("Going to point A")
			rospy.sleep(2)
			self.goal.target_pose.pose = locations['A']
		  	self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
			    rospy.loginfo("Reached point A")
			    rospy.sleep(2)
			    rospy.loginfo("Ready to go back")
			    rospy.sleep(2)
			    global start
			    start = 0
		elif table == 2:
			rospy.loginfo("Going to point B")
			rospy.sleep(2)
			self.goal.target_pose.pose = locations['B']
		  	self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
			    rospy.loginfo("Reached point B")
			    rospy.sleep(2)
			    rospy.loginfo("Ready to go back")
			    rospy.sleep(2)
			    global start
			    start = 0
		elif table == 3:
			rospy.loginfo("Going to point C")
			rospy.sleep(2)
			self.goal.target_pose.pose = locations['C']
		  	self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
			    rospy.loginfo("Reached point C")
			    rospy.sleep(2)
			    rospy.loginfo("Ready to go back")
			    rospy.sleep(2)
			    global start
			    start = 0
		else:
			rospy.loginfo("Going to point D")
			rospy.sleep(2)
			self.goal.target_pose.pose = locations['D']
		  	self.move_base.send_goal(self.goal)
			waiting = self.move_base.wait_for_result(rospy.Duration(300))
			if waiting == 1:
			    rospy.loginfo("Reached point D")
			    rospy.sleep(2)
			    rospy.loginfo("Ready to go back")
			    rospy.sleep(2)
			    global start
			    start = 0

	  # After reached point A, robot will go back to initial position
	  elif start == 0:
		rospy.loginfo("Going back home")
		rospy.sleep(2)
		self.goal.target_pose.pose = self.origin
		self.move_base.send_goal(self.goal)
		waiting = self.move_base.wait_for_result(rospy.Duration(300))
		if waiting == 1:
		    rospy.loginfo("Reached home")
		    rospy.sleep(2)
		    global start
		    start = 2

	  rospy.Rate(5).sleep()

    def update_initial_pose(self, initial_pose):
        self.initial_pose = initial_pose
	if original == 0:
		self.origin = self.initial_pose.pose.pose
		global original
		original = 1

    def cleanup(self):
        rospy.loginfo("Shutting down navigation	....")
	self.move_base.cancel_goal()

if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass

