#! /usr/bin/env python

"""!    @file go_to_point_ball.py
        @brief Ball navigation action server.
        
        This file defines the ball navigation field of the architecture."""

# Ros library
import rospy

# Useful library 
import math

#Actionlib
import actionlib
import actionlib.msg
# Custom message
import exp_assignment2.msg

# Messages
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState

## robot position variable
position_ = Point()
## robot pose variable
pose_ = Pose()
## robot yaw variable
yaw_ = 0
## Action server state
state_ = 0
## Goal
desired_position_ = Point()
# parameters
yaw_precision_ = math.pi / 9  # +/- 20 degree allowed
yaw_precision_2_ = math.pi / 90  # +/- 2 degree allowed
dist_precision_ = 0.1
kp_d = 0.5
ub_d = 1
z_back = 0.25

## Publisher
pub = None
## Publisher
pubz = None

## Action_server
act_s = None

# callbacks

## Callback function of the subscriber to the topic '/ball/odom'.
#
# It updates the current position of the ball.
# @param msg New current position of the ball
def clbk_odom(msg):
    global position_
    global pose_
    global yaw_

    # position
    position_ = msg.pose.pose.position
    pose_ = msg.pose.pose

## Action state controller
#
# Changes the state of the action server
# @param state New action server state
def change_state(state):
    global state_
    state_ = state
    print ('State changed to [%s]' % state_)

## Ball velocity controller
#
# It computes and publishes the next velocities for the Ball
# @param data Goal position
def go_straight_ahead(des_pos):
    global pub, state_, z_back
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) +
                        pow(des_pos.x - position_.x, 2))

    if des_pos.z != z_back:
        link_state_msg = LinkState()
        link_state_msg.link_name = "ball_link"
        link_state_msg.pose.position.x = position_.x
        link_state_msg.pose.position.y = position_.y
        link_state_msg.pose.position.z = des_pos.z
        z_back = des_pos.z
        pubz.publish(link_state_msg)

    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = kp_d * (des_pos.x-position_.x)
        if twist_msg.linear.x > ub_d:
            twist_msg.linear.x = ub_d
        elif twist_msg.linear.x < -ub_d:
            twist_msg.linear.x = -ub_d

        twist_msg.linear.y = kp_d * (des_pos.y-position_.y)
        if twist_msg.linear.y > ub_d:
            twist_msg.linear.y = ub_d
        elif twist_msg.linear.y < -ub_d:
            twist_msg.linear.y = -ub_d

        pub.publish(twist_msg)

    else:
        print ('Position error: [%s]' % err_pos)
        change_state(1)

## Stops the robot
def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    pub.publish(twist_msg)

## Core of the action server
#
# It manages different behaviors w.r.t. to the state of the action server.
# @param goal Goal position
def planning(goal):

    global state_, desired_position_
    global act_s

    desired_position_.x = goal.target_pose.pose.position.x
    desired_position_.y = goal.target_pose.pose.position.y
    desired_position_.z = goal.target_pose.pose.position.z

    state_ = 0
    rate = rospy.Rate(20)
    success = True

    feedback = exp_assignment2.msg.PlanningFeedback()
    result = exp_assignment2.msg.PlanningResult()

    while not rospy.is_shutdown():
        if act_s.is_preempt_requested():
            rospy.loginfo('Goal was preempted')
            act_s.set_preempted()
            success = False
            break
        elif state_ == 0:
            feedback.stat = "Reaching the goal"
            feedback.position = pose_
            act_s.publish_feedback(feedback)
            go_straight_ahead(desired_position_)
        elif state_ == 1:
            feedback.stat = "Target reached!"
            feedback.position = pose_
            act_s.publish_feedback(feedback)
            done()
            break
        else:
            rospy.logerr('Unknown state!')

        rate.sleep()
    if success:
        rospy.loginfo('Goal: Succeeded!')
        act_s.set_succeeded(result)

## Initialization of the node.
#
# This function initialize some of the publisher and subscriber executed by the program:<br>
# - A publisher to the topic "/ball/cmd_vel" to send command velocities to the ball.<br>
# - A publisher to the topic "/gazebo/set_link_state" to update the current link state of the ball.<br>
# - A subscriber to the topic "/ball/odom" to read the current position of the ball.<br>
# - An action server to the server '/robot/reaching_goal' to drive the ball in a new position.

def main():
    global pub, active_, act_s, pubz
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/ball/cmd_vel', Twist, queue_size=1)
    pubz = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)
    sub_odom = rospy.Subscriber('/ball/odom', Odometry, clbk_odom)
    act_s = actionlib.SimpleActionServer(
        '/ball/reaching_goal', exp_assignment2.msg.PlanningAction, planning, auto_start=False)
    act_s.start()

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    main()
