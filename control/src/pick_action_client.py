#! /usr/bin/env python

import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import rospy
import actionlib
from std_msgs.msg import Float64

from moveit_msgs.msg import PickupAction, PickupGoal

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def move(angles):
    goal = PickupGoal()

    goal.target_name = "box"

    goal.group_name = "arm"

    goal.end_effector = ""





    goal = FollowJointTrajectoryGoal()
    goal.trajectory.joint_names = ['joint1', 'joint2','joint2' ,'joint4']
    point = JointTrajectoryPoint()
    point.positions = angles
    point.time_from_start = rospy.Duration(3)
    goal.trajectory.points.append(point)
    client.send_goal_and_wait(goal)

if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    client = actionlib.SimpleActionClient('robot2/pickup', PickupAction)
    client.wait_for_server()
    move_joint([-0.5,-0.36,-0.19,0.7])
