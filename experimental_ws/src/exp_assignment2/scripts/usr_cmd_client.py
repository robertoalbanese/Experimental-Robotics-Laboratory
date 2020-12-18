#!/usr/bin/env python

"""!    @file usr_cmd_client.py
        @brief Action client to move the ball.
        
        With this file the human can interact with the ball and move it poiting on a certain location """
# Ros library
import rospy

# Useful library 
import time
from random import randrange

# Actionlib
import actionlib
import actionlib.msg

# Custom message
import exp_assignment2.msg


## Feedback callback of the action client.
# Prints the feedback message received from the action server.
# @param msg Feedback message received from the the action server.


def feedback_cb(msg):
    print 'Feedback received:', msg

## User command action client initialization.
# The action client connects to the action server and sends differt random position goal.


def main():
    rospy.init_node('user_command_client')

    # Action client initialization
    client = actionlib.SimpleActionClient(
        '/ball/reaching_goal', exp_assignment2.msg.PlanningAction)
    client.wait_for_server()

    # Fill in the goal here
    action_msg = exp_assignment2.msg.PlanningActionGoal()
    while 1:
        time.sleep(5)
        action_msg.goal.target_pose.pose.position.x = randrange(-8, 8)
        action_msg.goal.target_pose.pose.position.y = randrange(-8, 8)
        action_msg.goal.target_pose.pose.position.z = 0.5
        if randrange(1, 4) == randrange(1, 4):  # randomizing the case of disappearing ball
            action_msg.goal.target_pose.pose.position.z = -1
            rospy.loginfo('Ball disappeared')

        client.send_goal(action_msg.goal, feedback_cb=feedback_cb)
        client.wait_for_result(rospy.Duration.from_sec(5.0))

    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__ == '__main__':
    main()
