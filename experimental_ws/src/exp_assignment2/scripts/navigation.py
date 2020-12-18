#! /usr/bin/env python

"""!    @file navigation.py
        @brief Navigation action server.
        
        This file defines the navigation field of the architecture."""

# Ros library
import rospy

# Useful libraries
import numpy as np
import time
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion

# Actionlib
import actionlib
import actionlib.msg

# Messages
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

# Custom messages
import exp_assignment2.msg
from exp_assignment2.msg import BallState

## Definition of the class Navigation.
#
# This is an action server which takes a desired goal position and brings the robot to it by publishing velocities in the topic 'robot/cmd_vel'.
class Navigation:
    ## Initialization of the class.
    #
    # This constructor initialize some of the publisher and subscriber executed by the program:<br>
    # - A publisher to the topic "/robot/cmd_vel" to send command velocities to the robot.<br>
    # - A subscriber to the topic "/robot/odom" to read the current position of the robot.<br>
    # - A subscriber to the topic "/ball/state" to read the current state of the ball w.r.t. the robot.<br>
    # - A publisher to the topic "/joint_head_controller/command" to control the position of the head of the robot.<br>
    # - An action server to the server '/robot/reaching_goal' to drive the robot in a new position.

    # - Some useful variables.
    def __init__(self):
        # Creates a node with name 'navigation'.
        rospy.init_node('navigation')
        ## Current position.
        self.msg_pose = Pose()

        self.rate = rospy.Rate(20)
        ## Distance tolerance for navigation.
        self.distance_tolerance = 0.1
        ## Action server state.
        self.state = 0
        ## Ball reached flag.
        self.ball_reached = False
        ## Action server success.
        self.success = False
        ## Linear velocity upper bound.
        self.ub = 1.5
        ## Linear velocity lower bound.
        self.lb = 0.5

        ## Publisher which will publish to the topic '/robot/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            '/robot/cmd_vel', Twist, queue_size=10)

        ## A subscriber to the topic '/robot/pose'. self.update_pose is called when a message of type Pose is received.
        self.msg_pose_subscriber = rospy.Subscriber(
            '/robot/odom', Odometry, self.update_pose)

        ## A subscriber to the topic '/ball/state'. self.update_state is called when a message of type BallState is received.
        self.msg_ballstate_subscriber = rospy.Subscriber(
            '/ball/state', BallState, self.update_state)

        ## Publisher which will publish to the topic '/joint_head_controller/command' the position of the head.
        self.head_pos_publisher = rospy.Publisher(
            '/joint_head_controller/command', Float64, queue_size=1)

        ## An action service for the server /robot/reach_new_position'. planning() is called whenever a cliend request is received.
        self.act_s = actionlib.SimpleActionServer(
            '/robot/reaching_goal', exp_assignment2.msg.PlanningAction, self.planning, auto_start=False)
        self.act_s.start()

    ## Callback function of the subscriber to the topic '/ball/state'.
    #
    # It updates the state of the ball w.r.t. the robot
    # @param msg BallState message
    def update_state(self, msg):
        if msg.state == True:
            self.state = 2
        else:
            self.state = 0
        self.ball_reached = msg.ball_reached

    ## Action state controller
    #
    # Changes the state of the action server
    # @param new_state New action server state
    def change_state(self, new_state):
        self.state = new_state
        print ('State changed to [%s]' % self.state)

    ## Callback function of the subscriber to the topic '/robot/odom'.
    #
    # It updates the current position of the robot
    # @param data New current position of the robot
    def update_pose(self, data):
        # Get yaw from quaternions
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.msg_pose.x = round(data.pose.pose.position.x, 4)
        self.msg_pose.y = round(data.pose.pose.position.y, 4)
        self.msg_pose.theta = round(yaw, 4)

    ## Euclidean distance calculator.
    #
    # Euclidean distance between current pose and the goal.
    # @param goal_pose Goal position
    def euclidean_distance(self, goal_pose):
        
        return sqrt(pow((goal_pose.x - self.msg_pose.x), 2) +
                    pow((goal_pose.y - self.msg_pose.y), 2))

    ## Linear velocity calculator.
    # @param goal_pose Goal position
    def linear_vel(self, goal_pose, constant=0.2):
        return constant * (0.1 + self.euclidean_distance(goal_pose))

    ## Steering angle calculator.
    # @param goal_pose Goal position
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.msg_pose.y, goal_pose.x - self.msg_pose.x)

    ## Angular velocity calculator.
    # @param goal_pose Goal position
    def angular_vel(self, goal_pose, constant=2):
        return constant * (self.steering_angle(goal_pose) - self.msg_pose.theta)

    ## Robot velocity controller
    #
    # It computes and publishes the next velocities for the robot
    # @param data Goal position
    def move2goal(self, data):
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = data.target_pose.pose.position.x
        goal_pose.y = data.target_pose.pose.position.y

        err_pos = self.euclidean_distance(goal_pose)
        vel_msg = Twist()

        # If we havent reached the goal yet
        if err_pos >= self.distance_tolerance:
            # The head mantein a fixed position
            self.head_pos_publisher.publish(0)

            # Porportional controller.

            # Linear velocity in the x-axis.
            temp_vel_x = self.linear_vel(goal_pose)
            # Upper and lower bounding
            vel_msg.linear.x = np.sign(
                temp_vel_x)*min(abs(max(abs(temp_vel_x), abs(self.lb))), abs(self.ub))

            # Angular velocity in the z-axis.
            vel_msg.angular.z = self.angular_vel(goal_pose)
            print vel_msg
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        else:
            print ('Position error: [%s]' % err_pos)
            self.change_state(1)

    ## Stops the robot
    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        self.velocity_publisher.publish(twist_msg)

    ## Core of the action server
    #
    # It manages different behaviors w.r.t. to the state of the action server.
    # @param goal Goal position
    def planning(self, goal):

        self.state = 0
        feedback = exp_assignment2.msg.PlanningFeedback()
        result = exp_assignment2.msg.PlanningResult()

        while not rospy.is_shutdown():
            """ and self.success == False: """
            if self.act_s.is_preempt_requested():
                rospy.loginfo('Goal was preempted')
                self.act_s.set_preempted()
                self.success = False
                break
            elif self.state == 0:
                feedback.stat = "Reaching the goal"
                feedback.position.position.x = self.msg_pose.x
                feedback.position.position.y = self.msg_pose.y
                self.act_s.publish_feedback(feedback)
                self.move2goal(goal)
            elif self.state == 1:
                feedback.stat = "Target reached!"
                feedback.position.position.x = self.msg_pose.x
                feedback.position.position.y = self.msg_pose.y
                self.act_s.publish_feedback(feedback)
                self.done()
                self.act_s.set_succeeded(result)
                break
            elif self.state == 2:
                rospy.loginfo('Goal: Green!')
                feedback.stat = "Green ball seen!"
                feedback.position.position.x = self.msg_pose.x
                feedback.position.position.y = self.msg_pose.y
                self.act_s.publish_feedback(feedback)
                self.done()
                self.act_s.set_succeeded(result)
                break
            else:
                rospy.logerr('Unknown state!')

## Main function
#
# It defines an object of the type Navigation
if __name__ == '__main__':
    try:
        x = Navigation()

        # If we press control + C, the node will stop.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
