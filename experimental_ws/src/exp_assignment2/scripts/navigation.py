#! /usr/bin/env python
# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool, Float64
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from tf import transformations
import time
import math
import actionlib
import actionlib.msg
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Custom messages
import exp_assignment2.msg
from exp_assignment2.msg import BallState
from exp_assignment2.srv import ReachNextPosition

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
        # ball reached flag
        self.ball_reached = False
        # action server success
        self.success = False
        # linear velocity upper and lower bound
        self.ub = 1.5
        self.lb = 0.8    

        # Publisher which will publish to the topic '/robot/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            '/robot/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/robot/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.msg_pose_subscriber = rospy.Subscriber(
            '/robot/odom', Odometry, self.update_pose)

        # A subscriber to the topic '/ball/state'. self.update_state is called
        # when a message of type BallState is received.
        self.msg_ballstate_subscriber = rospy.Subscriber(
            '/ball/state', BallState, self.update_state)

        # Publisher which will publish to the topic '/joint_head_controller/command' the position of the head.
        self.head_pos_publisher = rospy.Publisher(
            '/joint_head_controller/command', Float64, queue_size=1)

        # An action service for the server /robot/reach_new_position'. move2goal() is called
        # whenever a cliend request is received.
        self.act_s = actionlib.SimpleActionServer(
            '/robot/reaching_goal', exp_assignment2.msg.PlanningAction, self.planning, auto_start=False)
        self.act_s.start()

    def update_state(self, msg):
        if msg.state == True:
            self.state = 2
        else:
            self.state = 0
        self.ball_reached = msg.ball_reached

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

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.msg_pose.x), 2) +
                    pow((goal_pose.y - self.msg_pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.2):
        return min(max(constant * (0.1 + self.euclidean_distance(goal_pose)),self.lb), self.ub)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.msg_pose.y, goal_pose.x - self.msg_pose.x)

    def angular_vel(self, goal_pose, constant=2):
        return min(constant * (self.steering_angle(goal_pose) - self.msg_pose.theta), self.ub)

    def move2goal(self, data):
        """Moves the robot to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = data.target_pose.pose.position.x
        goal_pose.y = data.target_pose.pose.position.y

        err_pos = self.euclidean_distance(goal_pose)
        vel_msg = Twist()

        if err_pos >= self.distance_tolerance:
            # The head mantein a fixed position
            self.head_pos_publisher.publish(0)

            # Porportional controller.

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)
            print vel_msg
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
        self.velocity_publisher.publish(twist_msg)

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
                # self.success = True
                break
            elif self.state == 2:
                rospy.loginfo('Goal: Green!')
                feedback.stat = "Green ball seen!"
                feedback.position.position.x = self.msg_pose.x
                feedback.position.position.y = self.msg_pose.y
                self.act_s.publish_feedback(feedback)
                self.act_s.set_succeeded(result)
                break
            else:
                rospy.logerr('Unknown state!')

        if self.success:
            #self.success = False
            rospy.loginfo('Goal: Succeeded!')
            self.act_s.set_succeeded(result)


if __name__ == '__main__':
    try:
        x = Navigation()

        # If we press control + C, the node will stop.
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
