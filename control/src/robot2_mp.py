#! /usr/bin/env python

#Code taken from Robot Ignite Academy. If perfroms multiple movements in the joint task space. The arm
# will move from the home position to a position above a 40x40mm cube, the pick the cube up, return home, 
# then put the cube bacl where it got it from.

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

names1 = 'position1'
values1 = [-0.5,-0.36,-0.19,0.7]
names2 = 'position2'
values2 = [0.5,0.4,-0.25,1.35]

###### Functions ########
def open_gripper():
	print "Opening Gripper..."
	gripper_group_variable_values[0] = 00.009
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def close_gripper():
	print "Closing Gripper..."
	gripper_group_variable_values[0] = -00.0006
	gripper_group.set_joint_value_target(gripper_group_variable_values)
	plan2 = gripper_group.go()
	gripper_group.stop()
	gripper_group.clear_pose_targets()
	rospy.sleep(1)

def move_home():
	arm_group.set_named_target("home")
	print "Executing Move: Home"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_zero():
	arm_group.set_named_target("zero")
	print "Executing Move: Zero"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position1():
	arm_group.set_named_target("position1")
	print "Executing Move: Position1"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position2():
	arm_group.set_named_target("position2")
	print "Executing Move: Position2"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

def move_position3():

	pose_target = geometry_msgs.msg.Pose()

	
	pose_target.orientation.x = -0.0978499189566
	pose_target.orientation.y = 0.714374698452
	pose_target.orientation.z = 0.0940288997676
	pose_target.orientation.w = 0.68647851356

	pose_target.position.x = 0.191577665297
	pose_target.position.y = 0.0501351324449
	pose_target.position.z =0.106057326459

	arm_group.set_pose_target(pose_target)

	print "Executing Move: Position3"
	plan1 = arm_group.plan()
	arm_group.execute(plan1, wait=True)
	arm_group.stop()
	arm_group.clear_pose_targets()
	variable = arm_group.get_current_pose()
	print (variable.pose)
	rospy.sleep(1)

###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory_robot2', anonymous=True)

robot = moveit_commander.RobotCommander("robot2/robot_description", ns = "robot2")
scene = moveit_commander.PlanningSceneInterface(ns = "robot2")
arm_group = moveit_commander.MoveGroupCommander("arm","robot2/robot_description", ns = "robot2")
gripper_group = moveit_commander.MoveGroupCommander("gripper","robot2/robot_description", ns = "robot2")
display_trajectory_publisher = rospy.Publisher('robot2/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#Had probelms with planner failing, Using this planner now. I believe default is OMPL
#arm_group.set_planner_id("RRTConnectkConfigDefault")
#Increased available planning time from 5 to 10 seconds
arm_group.set_planning_time(10);

arm_group.remember_joint_values(names1, values1)
arm_group.remember_joint_values(names2, values2)

gripper_group_variable_values = gripper_group.get_current_joint_values()

###### Main ########
move_home()
open_gripper()
move_position1()
close_gripper()
#move_home()
move_position2()
open_gripper()
move_home()

moveit_commander.roscpp_shutdown()
