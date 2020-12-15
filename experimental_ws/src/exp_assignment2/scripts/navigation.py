#! /usr/bin/env python
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import time
import math
import actionlib
import actionlib.msg
import exp_assignment2.msg
from exp_assignment2.srv import ReachNextPosition
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Navigation:

    def __init__(self):
        # Creates a node with name 'navigation'.
        rospy.init_node('navigation')
        # Current position
        self.msg_pose = Pose()

        self.rate = rospy.Rate(20)
        # distance tolerance for navigation
        self.distance_tolerance = 0.1
        # machine state
        self.state = 0

        # Publisher which will publish to the topic '/robot/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            '/robot/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/robot/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.msg_posesubscriber = rospy.Subscriber(
            '/robot/odom', Odometry, self.update_pose)

        # An action service for the server /robot/reach_new_position'. move2goal() is called
        # whenever a cliend request is received.
        self.act_s = actionlib.SimpleActionServer(
            '/robot/reaching_goal', exp_assignment2.msg.PlanningAction, self.planning, auto_start=False)
        self.act_s.start()

    def change_state(self, new_state):
        self.state = new_state
        print ('State changed to [%s]' % self.state)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        # Get yaw from quaternions
        orientation_q = data.pose.pose.orientation
        orientation_list = [orientation_q.x,
                            orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.msg_pose.x = round(data.pose.pose.position.x, 4)
        self.msg_pose.y = round(data.pose.pose.position.y, 4)
        self.msg_pose.theta = round(yaw, 4)
        print self.msg_pose

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.msg_pose.x), 2) +
                    pow((goal_pose.y - self.msg_pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.1):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.msg_pose.y, goal_pose.x - self.msg_pose.x)

    def angular_vel(self, goal_pose, constant=2):
        return constant * (self.steering_angle(goal_pose) - self.msg_pose.theta)

    def move2goal(self, data):
        """Moves the robot to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = data.target_pose.pose.position.x
        goal_pose.y = data.target_pose.pose.position.y

        err_pos = self.euclidean_distance(goal_pose)
        vel_msg = Twist()

        if err_pos >= self.distance_tolerance:

            # Porportional controller.

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        else:
            print ('Position error: [%s]' % err_pos)
            self.change_state(1)

    def done(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0
        pub.publish(twist_msg)

    def planning(self, goal):

        self.state = 0
        success = True
        print goal
        feedback = exp_assignment2.msg.PlanningFeedback()
        result = exp_assignment2.msg.PlanningResult()

        while not rospy.is_shutdown():
            if self.act_s.is_preempt_requested():
                rospy.loginfo('Goal was preempted')
                self.act_s.set_preempted()
                success = False
                break
            elif self.state == 0:
                feedback.stat = "Reaching the goal"
                feedback.position = self.msg_pose
                self.act_s.publish_feedback(feedback)
                self.move2goal(goal)
            elif self.state == 1:
                feedback.stat = "Target reached!"
                feedback.position = self.msg_pose
                self.act_s.publish_feedback(feedback)
                self.done()
                break
            else:
                rospy.logerr('Unknown state!')

        # If we press control + C, the node will stop.
        rospy.spin()
        if success:
            rospy.loginfo('Goal: Succeeded!')
            self.act_s.set_succeeded(result)


if __name__ == '__main__':
    try:
        x = Navigation()

        # If we press control + C, the node will stop.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
