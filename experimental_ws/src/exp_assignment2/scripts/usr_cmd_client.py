#!/usr/bin/env python

"""!    @file usr_cmd_client.py
        @brief Action client to move the ball.
        
        With this file the human can interact with the ball and move it poiting on a certain location """
        
import rospy
import actionlib
import actionlib.msg
import exp_assignment2.msg
import time
from random import randrange

## Feedback callback of the action client.
# Prints the feedback message received from the action server.
# @param msg feedback message received from the the action server.

def feedback_cb(msg):
 print 'Feedback received:', msg

## User command action client initialization.
# The action client connects to the action server and sends differt random position goal.

def main():
    rospy.init_node('user_command_client')
    
    #Action client initialization
    client = actionlib.SimpleActionClient('/reaching_goal', exp_assignment2.msg.PlanningAction)
    client.wait_for_server()

    # Fill in the goal here
    action_msg = exp_assignment2.msg.PlanningActionGoal()
    #~ rospy.loginfo(action_msg);
    #~ goal=action_msg.action_goal
    while 1:
		time.sleep(5)
		action_msg.goal.target_pose.pose.position.x = randrange(-8, 8)
		action_msg.goal.target_pose.pose.position.y = randrange(-8, 8)
		action_msg.goal.target_pose.pose.position.z = 1
		if andrange(1, 4) == randrange(1, 4): #randomizing the case of disappearing ball
			action_msg.goal.target_pose.pose.position.z = -1
			rospy.loginfo('Ball disappeared');

		#~ rospy.loginfo(action_msg);
		
		client.send_goal(action_msg.goal,feedback_cb=feedback_cb)
		client.wait_for_result(rospy.Duration.from_sec(5.0))
    
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
