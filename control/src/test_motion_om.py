#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg 
import copy
from std_msgs.msg import String


import rospkg, genpy
import yaml



def move_arm_xyz(x_cord, y_cord, z_cord):

	current_pose = arm_group.get_current_pose().pose
	x=current_pose.orientation.x
	y = current_pose.orientation.y
	z = current_pose.orientation.z
	w =current_pose.orientation.w

	pose_target = geometry_msgs.msg.Pose()

	pose_target.orientation.w = w
	pose_target.orientation.x = x
	pose_target.orientation.y = y
	pose_target.orientation.z = z

	pose_target.position.x = x_cord
	pose_target.position.y = y_cord
	pose_target.position.z =z_cord

	arm_group.set_pose_target(pose_target)
	plan1 = arm_group.go()

	arm_group.set_start_state_to_current_state()
	arm_group.stop()



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial', anonymous =True)
scene = moveit_commander.PlanningSceneInterface()
robot = moveit_commander.RobotCommander()

arm_group = moveit_commander.MoveGroupCommander("arm")

display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

end_effector_link = arm_group.get_end_effector_link()



arm_group.set_named_target("zero")
plan1 = arm_group.go()

arm_group.set_named_target("home")
plan1 = arm_group.go()

move_arm_xyz(0.25,0.05,0.18)

rospy.sleep(3)
moveit_commander.roscpp_shutdown()


