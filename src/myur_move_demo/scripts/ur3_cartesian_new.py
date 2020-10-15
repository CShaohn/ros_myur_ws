#!/usr/bin/env python
#coding:utf-8
# use moveit_commander (the Python MoveIt user interfaces )
# -*- coding: UTF-8 -*-
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi

class MoveGroupInteface(object):
	def __init__(self):
		super(MoveGroupInteface, self).__init__()
		######################### setup ############################
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node('ur_move_test_node', anonymous=True)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()  # Not used in this tutorial
		group_name = "manipulator"  # group_name can be find in ur5_moveit_config/config/ur5.srdf
		self.move_group_commander = moveit_commander.MoveGroupCommander(group_name)
		self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)
		
		################ Getting Basic Information ######################
		self.planning_frame = self.move_group_commander.get_planning_frame()
		print "============ Planning frame: %s" % self.planning_frame
		self.eef_link = self.move_group_commander.get_end_effector_link()
		print "============ End effector link: %s" % self.eef_link
		self.group_names = self.robot.get_group_names()
		print "============ Available Planning Groups:", self.robot.get_group_names()
		print "============ Printing robot state:"
		print self.robot.get_current_state()  # get
		print ""

	def plan_cartesian_path(self, scale=1):
		waypoints = []
		wpose = self.move_group_commander.get_current_pose().pose
		wpose.position.z -= scale * 0.1  # First move up (z)
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
		waypoints.append(copy.deepcopy(wpose))
		wpose.position.y += scale * 0.1  # Third move sideways (y)
		waypoints.append(copy.deepcopy(wpose))    

		# We want the Cartesian path to be interpolated at a resolution of 1 cm
		# which is why we will specify 0.01 as the eef_step in Cartesian
		# translation.  We will disable the jump threshold by setting it to 0.0,
		# ignoring the check for infeasible jumps in joint space, which is sufficient
		# for this tutorial.
		(plan, fraction) = self.move_group_commander.compute_cartesian_path(
										waypoints,   # waypoints to follow
										0.01,      # eef_step  
										0.0)         # jump_threshold  

		# Note: We are just planning, not asking move_group to actually move the robot yet:
	#	print "=========== Planning completed，Cartesian path is saved============="
		return plan, fraction

	def execute_plan(self, plan):
		## Use execute if you would like the robot to follow
		## the plan that has already been computed:
		self.move_group_commander.execute(plan, wait=True)

print "----------------------------------------------------------"
print "Welcome to the MoveIt MoveGroup Python Interface Tutorial"
print "----------------------------------------------------------"
print "Press Ctrl-D to exit at any time"
print ""
print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
raw_input()
tutorial = MoveGroupInteface()
print "============ Press `Enter` to plan and display a Cartesian path ..."
raw_input()
cartesian_plan, fraction = tutorial.plan_cartesian_path()
print "============ Press `Enter` to execute a saved path  ..."
raw_input()
tutorial.execute_plan(cartesian_plan)
print "============ Press `Enter` to go back ..."
raw_input()
cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
tutorial.execute_plan(cartesian_plan)
