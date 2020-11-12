#!/usr/bin/env python

"""!
 @mainpage
 @author Roberto Albanese
 @version 1.0
 @date 10-28-2020
 @section Introduction
 This code implements a basic software architecture accordingly to assignment n°1 of the **Experimental Robotics** course.<br>
 If you want to learn more about code development please read the README file in the git repository
"""

"""!    @file state_machine.py
        @brief Example Python program with Doxygen style comments."""

import roslib
import rospy
import smach
import smach_ros
import time
import random
from collections import namedtuple
from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from assignment_1.srv import *

# INSTALLATION
# - create ROS package in your workspace:
#          $ catkin_create_pkg smach_tutorial std_msgs rospy
# - move this file to the 'smach_tutorial/scr' folder and give running permissions to it with
#          $ chmod +x state_machine.py
# - run the 'roscore' and then you can run the state machine with
#          $ rosrun smach_tutorial state_machine.py
# - install the visualiser using
#          $ sudo apt-get install ros-kinetic-smach-viewer
# - run the visualiser with
#          $ sudo apt-get install ros-kinetic-smach-viewer

#Subscriber to user command callback
def get_command(data):
    global command
    command = data.data

#Client to new position callback
def go_to_new_position(x,y):
    rospy.wait_for_service('reach_new_position')
    try:
        new_pos = rospy.ServiceProxy('reach_new_position', Pose2D)
        #new_pos = rospy.ServiceProxy('reach_new_position', reach_next_pos)
        resp = new_pos(x,y)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


#Next position client
def get_position_client(xmin, xmax, ymin, ymax):
    rospy.wait_for_service('random_position')
    try:
        rand_pos = rospy.ServiceProxy('random_position', get_pos)
        resp = rand_pos(xmin, xmax, ymin, ymax)
        return resp
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

#define state Sleep
class Sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal'],
                             )

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: SLEEP')

        time.sleep(random.randint(1, 6))
        rospy.loginfo('I just woke up!')

        time.sleep(1)
        return 'gotoNormal'

# define state Normal
class Normal(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoSleep', 'gotoPlay', 'gotoNormal'],
                             )

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: NORMAL')

        global command
        global pub

        time.sleep(1)

        #Check if a command is been received
        if (command == "Play"):
            command = ""
            return 'gotoPlay'

        #Randomly going to sleep
        if (random.randint(1, 5) == 1):
            return 'gotoSleep'

        #Move to a new random position
        new_pos = get_position_client(x.min,x.max,y.min,y.max)
        reached_pos = go_to_new_position(new_pos.x,new_pos.y)

        rospy.loginfo('Dog: I\'m in      [%d,%d]', reached_pos.x, reached_pos.y)
        time.sleep(1)
        return 'gotoNormal'

# define state Sleep
class Play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal','gotoPlay'],
                             )

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: PLAY')

        #Play behaviour
        reached_pos = go_to_new_position(x.user,y.user)
        time.sleep(2)
        rospy.loginfo('Dog: I\'m in      [%d,%d]',reached_pos.x,reached_pos.y)
        new_pos = get_position_client(x.min,x.max,y.min,y.max)
        time.sleep(1)
        rospy.loginfo('User: Go to      [%d,%d]',new_pos.x,new_pos.y)
        reached_pos = go_to_new_position(new_pos.x,new_pos.y)
        time.sleep(2)
        rospy.loginfo('Dog: Here I am   [%d,%d]',reached_pos.x,reached_pos.y)
        reached_pos = go_to_new_position(x.user,y.user)
        time.sleep(2)
        rospy.loginfo('Dog: Back to my owner    [%d,%d]',reached_pos.x,reached_pos.y)
        time.sleep(1)

        #Randomly reuturn to Normal state
        if (random.randint(1, 3) == 1):
            return 'gotoNormal'

        time.sleep(1)
        return 'gotoPlay'


def main():
    rospy.init_node('Assignment_1_state_machine')

    #Subcriber to hw1_usr_cmd topic
    rospy.Subscriber("hw1_usr_cmd", String, get_command)

    #Plubisher to hw1_position
    global pub
    pub = rospy.Publisher('hw1_position', Pose2D, queue_size=1)
    rospy.Subscriber("hw1_position", Pose2D, go_to_new_position)

    #Command initialization
    global command
    command = ""

    # define environment structure builder
    env_struct = namedtuple("env_struct", "min max user")

    # Build environment
    xmin = random.randint(0, 5)
    xmax = random.randint(10, 15)
    ymin = random.randint(0, 5)
    ymax = random.randint(10, 15)

    global x
    global y
    x = env_struct(xmin, xmax, random.randint(xmin, xmax))
    y = env_struct(ymin, ymax, random.randint(ymin, ymax))

    global next_pos
    next_pos = Pose2D()
    next_pos.x = xmin
    next_pos.y = ymin

    rospy.loginfo('Map is setted as: x = {%d:%d}, y = {%d:%d}.',x.min, x.max, y.min, y.max)
    rospy.loginfo('The user is located in [%d,%d]',x.user,y.user)

    #Create a SMACH state machine
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
