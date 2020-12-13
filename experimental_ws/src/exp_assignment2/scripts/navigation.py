#!/usr/bin/env python
import rospy
from exp_assignment2.srv import ReachNextPosition
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt


class Navigazion:

    def __init__(self):
		# Creates a node with name 'navigation'.
		rospy.init_node('navigation')

		# Publisher which will publish to the topic '/turtle1/cmd_vel'.
		self.velocity_publisher = rospy.Publisher('/robot/cmd_vel', Twist, queue_size=10)

		# A subscriber to the topic '/turtle1/pose'. self.update_pose is called
		# when a message of type Pose is received.
		self.pose_subscriber = rospy.Subscriber('/robot/odom', Odometry, self.update_pose)
		
		# A service for the server /robot/reach_new_position'. move2goal() is called
        # whenever a cliend request is received. 
		self.nav_server = rospy.Service('/robot/reach_new_position', ReachNextPosition, self.move2goal)

		self.msg_pose = Pose()
		self.rate = rospy.Rate(10)
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        self.msg_pose.x = round(data.pose.pose.position.x, 4)
        self.msg_pose.y = round(data.pose.pose.position.y, 4)

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

    def move2goal(self,data):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = data.x
        goal_pose.y = data.y
        print goal_pose

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

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

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        

if __name__ == '__main__':
    try:
        x = Navigazion()
        
		# If we press control + C, the node will stop.
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
