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
import actionlib
import actionlib.msg
import exp_assignment2.msg
from exp_assignment2.msg import BallState
from collections import namedtuple
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Pose2D, Twist
from exp_assignment2.srv import *

# User command initialization.
# Stores the command Play of the user.
command = ""

ball_state = BallState()

# Publisher which will publish to the topic '/joint_head_controller/command' the position of the head.
head_pos_publisher = rospy.Publisher(
    '/robot/joint_head_controller/command', Float64, queue_size=1)

# Publisher vel_pub to move the whole robot
vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

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


def feedback_cb(msg):
    print 'Feedback received:', msg


def go_to_new_position(x, y):
    # rospy.wait_for_service('/robot/reaching_goal')
    try:
        print x
        print y
        new_pos = actionlib.SimpleActionClient(
            '/robot/reaching_goal', exp_assignment2.msg.PlanningAction)
        new_pos.wait_for_server(rospy.Duration(10.0))
        msg = exp_assignment2.msg.PlanningActionGoal()
        msg.goal.target_pose.pose.position.x = x
        msg.goal.target_pose.pose.position.y = y
        print "befor sending goal"
        time.sleep(2)
        new_pos.send_goal(msg.goal, feedback_cb=feedback_cb)
        time.sleep(2)
        print "after"
        new_pos.wait_for_result(rospy.Duration.from_sec(5.0))
        print "result"
        """new_pos.cancel_all_goals() """
        return
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def update_state(msg):
    global ball_state
    ball_state = msg

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

# Definition of state Sleep
#
# In this state the robot will go to sleep and then will return in the Normal state


class Sleep(smach.State):
    # Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal'],
                             )

    # Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(1)
        rospy.loginfo('State: SLEEP')

        reached_pos = go_to_new_position(-5, 7)

        time.sleep(random.randint(1, 6))
        rospy.loginfo('I just woke up!')

        time.sleep(1)
        return 'gotoNormal'

# Definition of state Normal
#
# In this state the robot will randomly navigate untill either a command is recived from the user or the robot randomly decides to go to Sleep.


class Normal(smach.State):
    # Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoSleep', 'gotoPlay', 'gotoNormal'],
                             )

    # Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: NORMAL')

        # global command
        # global pub

        time.sleep(1)

        # ~ # Check if a command is been received
        # ~ if (command == "Play"):
        # ~ command = ""
        # ~ return 'gotoPlay'

        # Randomly going to sleep
        """ if (random.randint(1, 5) == 1):
            return 'gotoSleep' """
        if (random.randint(1, 2) == 1):
            return 'gotoSleep'

        # Move to a new random position
        #new_pos = get_position_client(x.min, x.max, y.min, y.max)
        # request for the service to move in X and Y position
        new_goal_x = random.randrange(-5, 5)
        new_goal_y = random.randrange(-5, 5)
        reached_pos = go_to_new_position(new_goal_x, new_goal_y)
        # go_to_new_position(-1, -1)
        if ball_state.state == True:
            return 'gotoPlay'

        """    rospy.loginfo('Dog: I\'m in      [%d,%d]',
                      reached_pos.x, reached_pos.y)
        time.sleep(1) """
        return 'gotoNormal'
        # Wait for ctrl-c to stop the application
        rospy.spin()

# Definition of state Play
#
# In this state the robot will reach the user, it will go in the pointed position and it will finally come back to the user.


class Play(smach.State):
    # Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal', 'gotoPlay'],
                             )
        # Use time to see how much seconds pass after the ball is no more detected
        self.now = rospy.Time()
        self.then = rospy.Time()
        self.prev_ball_detected = False

    # Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: PLAY')
        global ball_state
        while 1:
            self.now = rospy.get_rostime()
            vel = Twist()
            # 400 is the center of the image
            vel.angular.z = -0.005*(ball_state.center[0]-400)
            # 150 is the radius that we want see in the image, which represent the desired disatance from the object
            vel.linear.x = -0.01*(ball_state.radius-150)
            # The head mantein a fixed position
            head_pos_publisher.publish(0)
            vel_pub.publish(vel)
            if vel.linear.x < 0.01 and ball_state.radius > 130:
                head_pos_publisher.publish(0.785398)
                time.sleep(5)
                head_pos_publisher.publish(-0.785398)
                time.sleep(5)
                head_pos_publisher.publish(0)
                time.sleep(5)
                rospy.loginfo("Finito di muovere la testa ")

            while self.prev_ball_detected == True and ball_state.state == False:
                self.then = rospy.get_rostime()
                if self.then.secs - self.now.secs > 3:
                    return 'gotoNormal'

            self.then = self.now
            self.prev_ball_detected = ball_state.state
            time.sleep(1)
        return 'gotoPlay'

        # Wait for ctrl-c to stop the application
        rospy.spin()

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

    # A subscriber to the topic '/ball/state'. self.update_state is called
    # when a message of type BallState is received.
    msg_ballstate_subscriber = rospy.Subscriber(
        '/ball/state', BallState, update_state)

    # Build environment
    xmin = -1
    xmax = 1
    ymin = -1
    ymax = 1

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
