#!/usr/bin/env python

"""!    @file ball_perception.py
        @brief Ball detection node.
        
        In this node the the library cv2 is used to detect the precence of the green ball."""

# Python libs
import sys

# numpy and scipy
import numpy as np
import scipy
from scipy.ndimage import filters

# Imutils
import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Custom messages
from exp_assignment2.msg import BallState

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

VERBOSE = False

## Definition of the class image_feature
#
# This node subscribes to the topic '/robot/camera1/image_raw/compressed' and checks the image for the presence of the ball
class image_feature:
    ## Initialization of the class.
    #
    # This constructor initialize some of the publisher and subscriber executed by the program:<br>
    # - A publisher to the topic "/output/image_raw/compressed" to output the compressed image.<br>
    # - A publisher to the topic "/ball/state" to send the current state of the ball w.r.t. the robott.<br>
    # - A subscriber to the topic "/robot/camera1/image_raw/compressed" to store the current image.<br>
    # - A publisher to the topic "/joint_head_controller/command" to control the position of the head of the robot.
    def __init__(self):
        # initialize the node
        rospy.init_node('ball_perception')

        ## Sends the processed and compressed images
        self.image_pub = rospy.Publisher("/output/image_raw/compressed",
                                         CompressedImage, queue_size=1)
        ## Sends to the command manager information reguarding the ball
        self.ball_state_pub = rospy.Publisher(
            "/ball/state", BallState, queue_size=1)

        ## Get the compressed images from the camera
        self.camera_sub = rospy.Subscriber("/robot/camera1/image_raw/compressed",
                                           CompressedImage, self.callback,  queue_size=1)

        ## Publishes the position of the head.
        self.head_pos_publisher = rospy.Publisher(
            '/joint_head_controller/command', Float64, queue_size=1)

    ## Callback function of the subscriber to the topic '/robot/camera1/image_raw/compressed'
    #
    # Here images get converted and features detected
    # @param ros_data Incoming new image
    def callback(self, ros_data):
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)

        # direct conversion to CV2
        # image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        # set colours bound
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
                cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                # Send the current state of the ball w.r.t. the robot
                msg = BallState()
                msg.state = True
                msg.ball_reached = False
                msg.center = center
                msg.radius = radius
                self.ball_state_pub.publish(msg)
        else:
            # Send the current state of the ball w.r.t. the robot
            msg = BallState()
            msg.state = False
            msg.ball_reached = False
            self.ball_state_pub.publish(msg)

        # Show the current image
        small_im = scipy.misc.imresize(image_np, 0.7)
        cv2.imshow('window', small_im)
        cv2.waitKey(2)

## Main function
#
# It defines an object of the type image_feature
def main(args):
    ic = image_feature()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
