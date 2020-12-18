#!/usr/bin/env python

"""!    @file state_machine.py
        @brief **FSM** of the architecture.
        
        As it is described in the mainpage, this code is mainly populated from the smach_ros library, which allows me to define and execute a state machine for the application."""

# Ros library
import roslib
import rospy

# Finite state machine
import smach
import smach_ros

# Useful libraries
import time
import random

# Actionlib
import actionlib
import actionlib.msg

# Messages
import exp_assignment2.msg
from exp_assignment2.msg import BallState
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

## Ball state.
# Describes the state of the ball w.r.t. the robot
ball_state = BallState()

## Feedback callback for the action client to '/robot/reaching_goal'.
# This function is called whenever is received a feedback from the action server
# @param msg feedback message


def feedback_cb(msg):
    print 'Feedback received:', msg


## Action client to the server '/robot/reaching_goal'.
# The client send a goal position whenever the robot need to go in a new position
# @param x x coordinate of the new position
# @param y y coordinate of the new position


def go_to_new_position(x, y):
    # rospy.wait_for_service('/robot/reaching_goal')
    try:
        new_pos = actionlib.SimpleActionClient(
            '/robot/reaching_goal', exp_assignment2.msg.PlanningAction)
        new_pos.wait_for_server(rospy.Duration(10.0))

        msg = exp_assignment2.msg.PlanningActionGoal()
        msg.goal.target_pose.pose.position.x = x
        msg.goal.target_pose.pose.position.y = y

        new_pos.send_goal(msg.goal, feedback_cb=feedback_cb)
        # new_pos.send_goal(msg.goal)
        new_pos.wait_for_result(rospy.Duration.from_sec(30.0))
        new_pos.cancel_all_goals()  # cancel all possible pending goals
        time.sleep(1)
        return
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

## Callback function of msg_ballstate_subscriber.
# This function is called whenever a new ball state is published in the topic '/ball/state'
# @param msg ball_state message


def update_state(msg):
    global ball_state
    ball_state = msg

## Definition of state Sleep
#
# In this state the robot will go to sleep in a given position [-5,7] and then it will return in the Normal state.


class Sleep(smach.State):
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
        # function called when exiting from the node, it can be blocking
        time.sleep(1)
        rospy.loginfo('State: SLEEP')

        reached_pos = go_to_new_position(-5, 7)

        # If the ball has been seen go to PLAY
        if ball_state.state == True:
            return 'gotoPlay'

        rospy.loginfo('Gonna take a nap.')
        time.sleep(random.randint(4, 6))
        rospy.loginfo('I just woke up!')

        time.sleep(1)
        return 'gotoNormal'

## Definition of state Normal
#
# In this state the robot will randomly navigate untill either the ball is seen or the robot randomly decides to go to SLEEP.


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

        time.sleep(1)

        # Randomly going to sleep
        if (random.randint(1, 5) == 1):
            return 'gotoSleep'

        # Move to a new random position
        new_goal_x = random.randrange(-5, 5)
        new_goal_y = random.randrange(-5, 5)
        rospy.loginfo("I'll go to [%d,%d]", new_goal_x, new_goal_y)
        time.sleep(2)
        # Calling the action server
        reached_pos = go_to_new_position(new_goal_x, new_goal_y)

        # If a ball has been seen
        if ball_state.state == True:
            return 'gotoPlay'
        # Otherwise
        return 'gotoNormal'

        # Wait for ctrl-c to stop the application
        rospy.spin()

## Definition of state Play
#
# In this state the robot follows the ball untill it cannot see it anymore for a certain amount of time (3 s).


class Play(smach.State):
    ## Initialization of the state.
    #
    # The outcomes, the '/robot/joint_head_controller/command' publisher and the '/robot/cmd_vel' publisher are defined.
    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal', 'gotoPlay'],
                             )
        # Use time to see how much seconds pass after the ball is no more detected
        self.now = rospy.Time()
        self.then = rospy.Time()
        self.prev_ball_detected = False

        # Publisher which will publish to the topic '/joint_head_controller/command' the position of the head.
        self.head_pos_publisher = rospy.Publisher(
            '/robot/joint_head_controller/command', Float64, queue_size=1)

        # Publisher vel_pub to move the whole robot
        self.vel_pub = rospy.Publisher("/robot/cmd_vel", Twist, queue_size=1)

    ## Body of the class
    def execute(self, userdata):
        # function called when exiting from the node, it can be blacking
        time.sleep(1)
        rospy.loginfo('State: PLAY')
        global ball_state
        # Play until the robot can't see the ball
        while 1:
            self.now = rospy.get_rostime()
            vel = Twist()
            # 400 is the center of the image
            vel.angular.z = -0.005*(ball_state.center[0]-400)
            # 150 is the radius that we want see in the image, which represent the desired disatance from the object
            vel.linear.x = min(-0.01*(ball_state.radius-150), 1)
            # The head mantein a fixed position
            self.head_pos_publisher.publish(0)

            # if the ball stops and we reach it
            if vel.linear.x < 0.01 and ball_state.radius > 130 and abs(ball_state.center[0]-400) < 10:
                # stop
                vel.linear.x = 0
                vel.angular.z = 0
                self.vel_pub.publish(vel)
                # move the head
                self.head_pos_publisher.publish(0.785398)
                time.sleep(5)
                self.head_pos_publisher.publish(-0.785398)
                time.sleep(5)
                self.head_pos_publisher.publish(0)
                time.sleep(5)
            else:
                self.vel_pub.publish(vel)
            # if the robot can't see the ball for 3 secs go back to NORMAL
            while self.prev_ball_detected == True and ball_state.state == False:
                # Stop the robot
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                self.vel_pub.publish(twist_msg)

                self.then = rospy.get_rostime()
                if self.then.secs - self.now.secs > 3:
                    return 'gotoNormal'

            self.then = self.now
            self.prev_ball_detected = ball_state.state
            time.sleep(1)

        return 'gotoPlay'
        # Wait for ctrl-c to stop the application
        rospy.spin()

## Main body of the program.
# This function initialize some of the nodes executed by the program:<br>
# - A subscriber to the topic "/ball/state" to read the current ball state w.r.t. the robot.<br>
# - A smach state machine.<br>


def main():
    rospy.init_node('Assignment_2_FSM')

    # A subscriber to the topic '/ball/state'. update_state is called
    # when a message of type BallState is received.
    msg_ballstate_subscriber = rospy.Subscriber(
        '/ball/state', BallState, update_state)

    rospy.loginfo(
        'Map is setted as: x = {-8:8}, y = {-8:8}.')
    rospy.loginfo('The user is located in [%-7,-7].')

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
                               transitions={'gotoNormal': 'NORMAL',
                                            'gotoPlay': 'PLAY'})
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
