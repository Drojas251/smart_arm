#!/usr/bin/env python
import rospy, sys
import numpy as np
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from moveit_msgs.msg import Grasp, GripperTranslation, MoveItErrorCodes

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from copy import deepcopy

GROUP_NAME_ARM = 'arm'
GROUP_NAME_GRIPPER = 'gripper'

GRIPPER_FRAME = 'end_effector_link'

GRIPPER_OPEN = [00.008,0.008]
GRIPPER_CLOSED = [-00.0006, -00.0006]
GRIPPER_NEUTRAL = [0.008, 0.008]
GRIPPER_GRASP = [-00.0006,-00.0006]

GRIPPER_JOINT_NAMES = ['gripper','gripper_sub']

GRIPPER_EFFORT = [1.0]

REFERENCE_FRAME = 'world'

class MoveItDemo:
    def __init__(self):
        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)
        
        rospy.init_node('moveit_demo')
        
        # Use the planning scene object to add or remove objects
        scene = PlanningSceneInterface()
        
        # Create a scene publisher to push changes to the scene
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        
        # Create a publisher for displaying gripper poses
        self.gripper_pose_pub = rospy.Publisher('gripper_pose', PoseStamped, queue_size=5)
        
        # Create a dictionary to hold object colors
        self.colors = dict()
                        
        # Initialize the move group for the right arm
        right_arm = MoveGroupCommander(GROUP_NAME_ARM)
        
        # Initialize the move group for the right gripper
        right_gripper = MoveGroupCommander(GROUP_NAME_GRIPPER)
        
        # Get the name of the end-effector link
        end_effector_link = right_arm.get_end_effector_link()
 
        # Allow some leeway in position (meters) and orientation (radians)
        #right_arm.set_goal_position_tolerance(0.05)
        #right_arm.set_goal_orientation_tolerance(0.1)

        # Allow replanning to increase the odds of a solution
        right_arm.allow_replanning(True)
        
        # Set the right arm reference frame
        #right_arm.set_pose_reference_frame(REFERENCE_FRAME)
        
        # Allow 5 seconds per planning attempt
        right_arm.set_planning_time(3)

        #right_arm.set_goal_joint_tolerance(0.5)
        right_arm.set_goal_orientation_tolerance(0.2)
        
        # Set a limit on the number of pick attempts before bailing
        max_pick_attempts = 10
        
        # Set a limit on the number of place attempts
        max_place_attempts = 5
                
        # Give the scene a chance to catch up
        rospy.sleep(2)



   
        # clean the scene
        scene.remove_world_object("table")
        scene.remove_world_object("part")
        
        # Remove any attached objects from a previous session
        scene.remove_attached_object(GRIPPER_FRAME, "part")
        
        # Give the scene a chance to catch up    
        rospy.sleep(1)
        
        # Start the arm in the "grasp" pose stored in the SRDF file
        right_arm.set_named_target('home')
        right_arm.go()
        rospy.sleep(2)


        #gripper_group_variable_values = right_gripper.get_current_joint_values()
        #gripper_group_variable_values[0] = 00.008
        #right_gripper.set_joint_value_target(gripper_group_variable_values)
        
        # Open the gripper to the neutral position
        right_gripper.set_joint_value_target(GRIPPER_OPEN)
        right_gripper.go()
       
        rospy.sleep(1)


        # _______________________________GOOD UP TO HERE ____________________________

        place_pose = PoseStamped()
        place_pose.header.frame_id = REFERENCE_FRAME
        place_pose.pose.position.x = 0.21
        place_pose.pose.position.y = 0.0
        place_pose.pose.position.z = 0.2
        scene.add_box("part", place_pose, (0.04, 0.02, 0.05))
        
        # Specify a pose to place the target after being picked up
        target_pose = PoseStamped()
        target_pose.header.frame_id = REFERENCE_FRAME
        # start the gripper in a neutral pose part way to the target     

        #target_pose.pose.position.x = 0.133923763037
        #target_pose.pose.position.y = 0.00
        #target_pose.pose.position.z = 0.2097191751

        current_pose = right_arm.get_current_pose().pose
        x = current_pose.orientation.x
        y = current_pose.orientation.y
        z = current_pose.orientation.z
        w = current_pose.orientation.w
        E = euler_from_quaternion([x,y,z,w] )
        rospy.loginfo("ANGLES " + str(E) )


        target_pose.pose.orientation.x = x
        target_pose.pose.orientation.y = y
        target_pose.pose.orientation.z = z
        target_pose.pose.orientation.w = w

        grasp_pose = target_pose
        #right_arm.set_pose_target(target_pose)
        #right_arm.go()

        #rospy.sleep(2)

        target_pose.pose.position.x = 0.175
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.2

        # Initialize the grasp pose to the target pose

        #target_pose.pose.position.x = 0.17
        #target_pose.pose.position.y = 0.00
        #target_pose.pose.position.z = 0.2   
        #right_arm.set_pose_target(target_pose)
        #right_arm.go()   
        #rospy.sleep(2)



        # Shift the grasp pose by half the width of the target to center it
        #grasp_pose.pose.position.y -= target_size[1] / 2.0
        #grasp_pose.pose.position.x = 0.12792118579
        #grasp_pose.pose.position.y = -0.285290879999
        #grasp_pose.pose.position.z = 0.120301181892
                
        # Generate a list of grasps
        grasps = self.make_grasps(grasp_pose, 'part')

        # Publish the grasp poses so they can be viewed in RViz
        print "Publishing grasps"
        for grasp in grasps:
            self.gripper_pose_pub.publish(grasp.grasp_pose)
            rospy.sleep(0.2)

        # Track success/failure and number of attempts for pick operation
        result = None
        n_attempts = 0
        
        # Repeat until we succeed or run out of attempts
        while result != MoveItErrorCodes.SUCCESS and n_attempts < max_pick_attempts:
            n_attempts += 1
            rospy.loginfo("Pick attempt: " +  str(n_attempts))
            result = right_arm.pick('part', grasps)
            rospy.sleep(0.2)
        
        # If the pick was successful, attempt the place operation   
        if result == MoveItErrorCodes.SUCCESS:
            result = None
            n_attempts = 0
            rospy.loginfo("Pick operation SUCCESS " )
            

        else:
            rospy.loginfo("Pick operation failed after " + str(n_attempts) + " attempts.")

        rospy.sleep(1)

        # Shut down MoveIt cleanly
        moveit_commander.roscpp_shutdown()
        
        # Exit the script
        moveit_commander.os._exit(0)
        
    # Get the gripper posture as a JointTrajectory
    def make_gripper_posture(self, joint_positions):
        # Initialize the joint trajectory for the gripper joints
        t = JointTrajectory()
        
        # Set the joint names to the gripper joint names
        t.joint_names = GRIPPER_JOINT_NAMES
        
        # Initialize a joint trajectory point to represent the goal
        tp = JointTrajectoryPoint()
        
        # Assign the trajectory joint positions to the input positions
        tp.positions = joint_positions
        
        # Set the gripper effort
        tp.effort = GRIPPER_EFFORT
        
        tp.time_from_start = rospy.Duration(1.0)
        
        # Append the goal point to the trajectory points
        t.points.append(tp)
        rospy.loginfo("Made gripper posture " )
        
        # Return the joint trajectory
        return t
    
    # Generate a gripper translation in the direction given by vector
    def make_gripper_translation(self, min_dist, desired, vector):
        # Initialize the gripper translation object
        g = GripperTranslation()
        
        # Set the direction vector components to the input
        g.direction.vector.x = vector[0]
        g.direction.vector.y = vector[1]
        g.direction.vector.z = vector[2]
        
        # The vector is relative to the gripper frame
        g.direction.header.frame_id = GRIPPER_FRAME
        
        # Assign the min and desired distances from the input
        g.min_distance = min_dist
        g.desired_distance = desired
        rospy.loginfo("Made gripper translation " )
        
        return g

    # Generate a list of possible grasps
    def make_grasps(self, initial_pose_stamped, allowed_touch_objects):
        # Initialize the grasp object
        g = Grasp()
        
        # Set the pre-grasp and grasp postures appropriately
        g.pre_grasp_posture = self.make_gripper_posture(GRIPPER_OPEN)
        g.grasp_posture = self.make_gripper_posture(GRIPPER_GRASP)
                
        # Set the approach and retreat parameters as desired
        g.pre_grasp_approach = self.make_gripper_translation(0.01, 0.05, [1.0, 0, 0])
        g.post_grasp_retreat = self.make_gripper_translation(0.1, 0.15, [0, 0, 1.0])

        rospy.loginfo("Successfully made all grasp configs" )
        
        # Set the first grasp pose to the input pose
        g.grasp_pose = initial_pose_stamped
    
        ideal_roll = 0
        ideal_pitch = 0
        ideal_yaw = 0
        
        step_size = 0.1
        idx = 0.5
        idx_roll = ideal_roll 
        idx_pitch = ideal_pitch + idx
        idx_yaw = ideal_yaw 
        roll_vals = []
        pitch_vals = []
        yaw_vals = []
        while idx >= -0.5:
            roll_vals.append(idx_roll)
            pitch_vals.append(idx_pitch)
            yaw_vals.append(idx_yaw)
            idx -= step_size
            idx_roll =0
            idx_pitch -= step_size
            idx_yaw =0

        # A list to hold the grasps
        grasps = []
        
        print "Generating Poses"
        
        # Generate a grasp for each roll pitch and yaw angle
        for p in pitch_vals:
            # Create a quaternion from the Euler angles
            angles = np.array([0,p,0])
            rospy.loginfo("ANGLES " + str(angles) )
            q = quaternion_from_euler(0, p, 0)
                    
            # Set the grasp pose orientation accordingly
            g.grasp_pose.pose.orientation.x = q[0]
            g.grasp_pose.pose.orientation.y = q[1]
            g.grasp_pose.pose.orientation.z = q[2]
            g.grasp_pose.pose.orientation.w = q[3]
                    
            # Set and id for this grasp (simply needs to be unique)
            g.id = str(len(grasps))
                    
            # Set the allowed touch objects to the input list
            g.allowed_touch_objects = allowed_touch_objects
                    
            # Don't restrict contact force
            g.max_contact_force = 0
                    
            # Degrade grasp quality for increasing pitch angles
            g.grasp_quality = 1.0 - abs(p)
                    
            # Append the grasp to the list
            grasps.append(deepcopy(g))
                    
        print "Generated " + g.id + " poses"
        rospy.loginfo("Made all gripper grasps " )
        # Return the list
        return grasps


if __name__ == "__main__":
    MoveItDemo()