#!/usr/bin/env python

import roslib
import rospy
import smach
import smach_ros
import time
import random
from collections import namedtuple
from std_msgs.msg import String
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

def get_position_client(xmin, xmax, ymin, ymax):
    rospy.wait_for_service('random_position')
    try:
        new_position = rospy.ServiceProxy('random_position', get_pos)
        resp1 = new_position(xmin, xmax, ymin, ymax)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

# define environment structure builder
env_struct = namedtuple("env_struct", "min max user")

# define state Sleep
class Sleep(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal'],
                             )

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        rospy.loginfo('Executing state Sleep')
        time.sleep(random.randint(1, 6))
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
        rospy.loginfo('Executing state Normal')
        time.sleep(1)

        if (random.randint(1, 4) == random.randint(1, 4)):
            return 'gotoSleep'

        new_pos = get_position_client(1, 20, 2, 15)
        rospy.loginfo('I am in x = %d, y = %d', new_pos.x, new_pos.y)
        return 'gotoNormal'

# define state Sleep
class Play(smach.State):
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal'],
                             )

    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(5)
        rospy.loginfo('Executing state Play')
        # userdata.unlocked_counter_out = userdata.unlocked_counter_in + 1
        time.sleep(random.randint(1, 5))
        return 'gotoNormal'


def main():
    rospy.init_node('Assignment_1_state_machine')

    xmin = random.randint(0, 5)
    xmax = random.randint(10, 15)
    ymin = random.randint(0, 5)
    ymax = random.randint(10, 15)

    # Build environment
    x = env_struct(xmin, xmax, random.randint(xmin, xmax))
    y = env_struct(ymin, ymax, random.randint(ymin, ymax))

    rospy.loginfo('Map is setted as: x = {%d:%d}, y = {%d:%d}.',x.min, x.max, y.min, y.max)
    rospy.loginfo('The user is located in [%d,%d]',x.user,y.user)

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
                               transitions={'gotoNormal': 'NORMAL'})

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
