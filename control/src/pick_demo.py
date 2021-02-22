#!/usr/bin/env python




import sys
import rospy
import moveit_commander
from moveit_msgs.msg import Grasp
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped, Vector3Stamped

if __name__=='__main__':

    GRIPPER_OPEN = [00.009]
    GRIPPER_CLOSED = [-00.0006]

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm_group = moveit_commander.MoveGroupCommander("arm")
    end_effector_link = arm_group.get_end_effector_link()

    # clean the scene
    #scene.remove_world_object("pole")
    #scene.remove_world_object("table")
    scene.remove_world_object("part")

    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    #p.pose.position.x = 0.22
    #p.pose.position.y = 0.2
    #p.pose.position.z = 0.1
    #p.pose.orientation.w = 1.0
    #scene.add_box("pole", p, (0.1, 0.025, 0.1))

    #p.pose.position.x = 0.35
    #p.pose.position.y = 0.0
    #p.pose.position.z = 0.025
    #scene.add_box("table", p, (0.5, 1.5, 0.05))

    p.pose.position.x = 0.289
    p.pose.position.y = 0.0
    p.pose.position.z = 0.0125
    scene.add_box("part", p, (0.025, 0.025, 0.025))

    rospy.sleep(1)

    grasp = Grasp()
    grasp.grasp_pose.header.frame_id = "world"
    grasp.grasp_pose.pose.orientation.x = 3.57719987617e-08
    grasp.grasp_pose.pose.orientation.y = 7.52404672178e-05
    grasp.grasp_pose.pose.orientation.z = -0.000475435572642
    grasp.grasp_pose.pose.orientation.w = 0.999996581245

    grasp.grasp_pose.pose.position.x = 0.289
    grasp.grasp_pose.pose.position.y = 0.0
    grasp.grasp_pose.pose.position.z = 0.01

    grasp.pre_grasp_approach.direction.header.frame_id = "end_effector_link"
    grasp.pre_grasp_approach.direction.vector.x = 1.0
    grasp.pre_grasp_approach.min_distance = 0.095
    grasp.pre_grasp_approach.desired_distance = 0.115

    grasp.post_grasp_retreat.direction.header.frame_id = "end_effector_link"
    grasp.post_grasp_retreat.direction.vector.z = 1.0
    grasp.post_grasp_retreat.min_distance = 0.1
    grasp.post_grasp_retreat.desired_distance = 0.25
    

    # pick an object
    robot.arm.pick("part",grasp=grasp)

    rospy.spin()
    roscpp_shutdown()
