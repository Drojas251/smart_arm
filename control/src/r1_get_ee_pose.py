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




###### Setup ########
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_execute_trajectory_robot1', anonymous=True)

robot = moveit_commander.RobotCommander("robot1/robot_description", ns = "robot1")
scene = moveit_commander.PlanningSceneInterface(ns = "robot1")
arm_group = moveit_commander.MoveGroupCommander("arm","robot1/robot_description", ns = "robot1")
gripper_group = moveit_commander.MoveGroupCommander("gripper","robot1/robot_description", ns = "robot1")
display_trajectory_publisher = rospy.Publisher('robot1/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

#Had probelms with planner failing, Using this planner now. I believe default is OMPL
#arm_group.set_planner_id("RRTConnectkConfigDefault")
#Increased available planning time from 5 to 10 seconds
arm_group.set_planning_time(10);


gripper_group_variable_values = gripper_group.get_current_joint_values()

###### Main ########
variable = arm_group.get_current_pose()
print (variable.pose)
rospy.sleep(1)


moveit_commander.roscpp_shutdown()
