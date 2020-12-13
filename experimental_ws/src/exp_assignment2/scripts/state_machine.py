#!/usr/bin/env python

"""!    @file state_machine.py
        @brief **FSM** of the architecture.
        
        As it is described in the mainpage, this code is mainly populated from the smach_ros library, which allows me to define and execute a state machine for the application."""

import roslib
import rospy
import smach
import smach_ros
import time
import random
from collections import namedtuple
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from exp_assignment2.srv import *

# User command initialization.
# Stores the command Play of the user.
command = ""

# Subscriber to user command callback.
# This function is called whenever the publisher of the function usr_cmd.cpp publishes a message in the topic "hw1_usr_cmd". The message is stored in command
# @param data stores the message published in the topic "hw1_usr_cmd"


def get_command(data):
    global command
    command = data.data

# Navigate to next position cliend
# This function is called whenever robot need to reach a new position
# @param x stores x coordinate of the new position
# @param y stores y coordinate of the new position
# @return **resp** new current position (resp.x, resp.y)


def go_to_new_position(x, y):
    rospy.wait_for_service('/robot/reach_new_position')
    try:
        new_pos = rospy.ServiceProxy('/robot/reach_new_position', ReachNextPosition)
        resp = new_pos(x, y)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


# Next random position client.
#
# This function is called whenever the **FSM** need to ask a new random position to the server "new_rand_pos_srv"
# @param xmin low boundary of the map for the x coordinate
# @param xmax high boundary of the map for the x coordinate
# @param ymin low boundary of the map for the y coordinate
# @param ymax high boundary of the map for the y coordinate
# @return **resp** new random position (resp.x, resp.y)

def get_position_client(xmin, xmax, ymin, ymax):
    rospy.wait_for_service('new_rand_pos_srv')
    try:
        rand_pos = rospy.ServiceProxy('new_rand_pos_srv', GetNewPos)
        resp = rand_pos(xmin, xmax, ymin, ymax)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

## Definition of state Sleep
#
# In this state the robot will go to sleep and then will return in the Normal state


class Sleep(smach.State):
    ## Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal'],
                             )

    ## Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: SLEEP')

        time.sleep(random.randint(1, 6))
        rospy.loginfo('I just woke up!')

        time.sleep(1)
        return 'gotoNormal'

## Definition of state Normal
#
# In this state the robot will randomly navigate untill either a command is recived from the user or the robot randomly decides to go to Sleep.


class Normal(smach.State):
    ## Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoSleep', 'gotoPlay', 'gotoNormal'],
                             )

    ## Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: NORMAL')

        global command
        global pub

        time.sleep(1)

        #~ # Check if a command is been received
        #~ if (command == "Play"):
            #~ command = ""
            #~ return 'gotoPlay'

        #~ # Randomly going to sleep
        #~ if (random.randint(1, 5) == 1):
            #~ return 'gotoSleep'

        # Move to a new random position
        new_pos = get_position_client(x.min, x.max, y.min, y.max)
        print x
        print y
        reached_pos = go_to_new_position(new_pos.x, new_pos.y)

        rospy.loginfo('Dog: I\'m in      [%d,%d]',
                      reached_pos.x, reached_pos.y)
        time.sleep(1)
        return 'gotoNormal'

## Definition of state Play
#
# In this state the robot will reach the user, it will go in the pointed position and it will finally come back to the user.


class Play(smach.State):
    ## Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal', 'gotoPlay'],
                             )

    ## Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: PLAY')

        # Play behaviour
        reached_pos = go_to_new_position(x.user, y.user)
        time.sleep(2)
        rospy.loginfo('Dog: I\'m in      [%d,%d]',
                      reached_pos.x, reached_pos.y)
        new_pos = get_position_client(x.min, x.max, y.min, y.max)
        time.sleep(1)
        rospy.loginfo('User: Go to      [%d,%d]', new_pos.x, new_pos.y)
        reached_pos = go_to_new_position(new_pos.x, new_pos.y)
        time.sleep(2)
        rospy.loginfo('Dog: Here I am   [%d,%d]', reached_pos.x, reached_pos.y)
        reached_pos = go_to_new_position(x.user, y.user)
        time.sleep(2)
        rospy.loginfo(
            'Dog: Back to my owner    [%d,%d]', reached_pos.x, reached_pos.y)
        time.sleep(1)

        # Randomly reuturn to Normal state
        if (random.randint(1, 3) == 1):
            return 'gotoNormal'

        time.sleep(1)
        return 'gotoPlay'

# Main body of the program.
#
# This function initialize some of the nodes executed by the program:<br>
# - A subscriber to the topic "hw1_usr_cmd", to read all the incoming commands of the user.<br>
# - A smach state machine.<br>
#
# It also define structures *x* and *y* in which informations, as the boundaries of the map and the position of the user, are stored.


def main():
    rospy.init_node('Assignment_2_FSM')

    # Subcriber to hw1_usr_cmd topic
    rospy.Subscriber("hw1_usr_cmd", String, get_command)

    # define environment structure builder
    env_struct = namedtuple("env_struct", "min max user")

    # Build environment
    xmin = -4
    xmax = 4
    ymin = -4
    ymax = 4

    # Global structs composed of three fields
    global x
    global y

    # x stores the minimum and maximum boundary of the map for the x coordinate along with the x position of the user
    x = env_struct(xmin, xmax, random.randint(xmin, xmax))
    # y stores the minimum and maximum boundary of the map for the y coordinate along with the x position of the user
    y = env_struct(ymin, ymax, random.randint(ymin, ymax))

    global next_pos
    next_pos = Pose2D()
    next_pos.x = xmin
    next_pos.y = ymin

    rospy.loginfo(
        'Map is setted as: x = {%d:%d}, y = {%d:%d}.', x.min, x.max, y.min, y.max)
    rospy.loginfo('The user is located in [%d,%d]', x.user, y.user)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),
                               transitions={'gotoSleep': 'SLEEP',
                                            'gotoPlay': 'PLAY',
                                            'gotoNormal': 'NORMAL'})
        smach.StateMachine.add('SLEEP', Sleep(),
                               transitions={'gotoNormal': 'NORMAL'})
        smach.StateMachine.add('PLAY', Play(),
                               transitions={'gotoNormal': 'NORMAL',
                                            'gotoPlay': 'PLAY'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
