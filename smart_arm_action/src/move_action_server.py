#! /usr/bin/env python
# BEGIN ALL
#! /usr/bin/env python
import rospy

# BEGIN PART_1
import time
import actionlib
from smart_arm_action.msg import moveAction, moveGoal, moveResult
# END PART_1

# BEGIN PART_2
def do_move(goal):


    start_time = time.time()
    time.sleep(goal.time_to_wait.to_sec())
    # END PART_2
    # BEGIN PART_3
    result = TimerResult()
    result.time_elapsed = rospy.Duration.from_sec(time.time() - start_time)
    result.updates_sent = 0
    # END PART_3
    # BEGIN PART_4
    server.set_succeeded(result)
    # END PART_4

rospy.init_node('move_action_server')
server = actionlib.SimpleActionServer('move_action', moveAction, do_move, False)
server.start()
rospy.spin()
# END ALL
