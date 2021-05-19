#!/usr/bin/env python

"""!    @file ball_perception.py
        @brief Ball detection node.
        
        In this node the the library cv2 is used to detect the precence of the colored balls."""

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
from exp_assignment3.msg import BallState

# Ros Messages
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

VERBOSE = False

## Definition of the class ImageFeature
#
# This node subscribes to the topic '/camera/color/image_raw/compressed' and inspects the image for the presence of the ball.


class ImageFeature:
    ## Initialization of the class.
    #
    # This constructor initialize some of the publisher and subscriber executed by the program:<br>
    # - A publisher to the topic "/ball/state" to send the current state of the ball w.r.t. the robott.<br>
    # - A subscriber to the topic "/camera/color/image_raw/compressed" to store the current image.<br>
    def __init__(self):
        # initialize the node
        rospy.init_node('ball_perception')

        ## @private Sends to the command manager information reguarding the ball
        self.ball_state_pub = rospy.Publisher(
            "/ball/state", BallState, queue_size=1)

        ## @private Get the compressed images from the camera
        self.camera_sub = rospy.Subscriber("camera/color/image_raw/compressed",
                                           CompressedImage, self.image_processing,  queue_size=1)

        # set colours bound
        greenLower = (50, 50, 20)
        greenUpper = (70, 255, 255)
        blueLower = (105, 50, 20)
        blueUpper = (125, 255, 255)
        redLower = (0, 50, 20)
        redUpper = (10, 255, 255)
        blackLower = (0, 0, 0)
        blackUpper = (180, 255, 30)
        magentaLower = (145, 50, 20)
        magentaUpper = (160, 255, 255)
        yellowLower = (25, 50, 20)
        yellowUpper = (35, 255, 255)
        
        ## @private Ball colors
        self.colors = ['blue', 'red', 'green', 'yellow', 'magenta', 'black']

        ## @private Stores the boundaries of each color mask used to detect colors
        self.boundaries = [[blueLower, blueUpper],
                           [redLower, redUpper],
                           [greenLower, greenUpper],
                           [yellowLower, yellowUpper],
                           [magentaLower, magentaUpper],
                           [blackLower, blackUpper]]

        ## @private Array of flags indicating the presence of each ball
        self.ball_check = [False, False, False, False, False, False]

    ## Callback function of the subscriber to the topic '/camera/color/image_raw/compressed'
    #
    # Here images get converted and features detected
    # @param ros_data Incoming new image
    def image_processing(self, ros_data):
        if VERBOSE:
            print ('received image of type: "%s"' % ros_data.format)
        # direct conversion to CV2
        # image_np is the image decompressed and converted in OpendCv
        np_arr = np.fromstring(ros_data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

        blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # Detection of colors through masks
        for i in range(len(self.colors)):
            mask = cv2.inRange(
                hsv, self.boundaries[i][0], self.boundaries[i][1])
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
                print('')
                print("Found " + self.colors[i] + " ball.")
                c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                print(
                    "Center: (" + str(center[0]) + "," + str(center[1]) + ")")
                print("Radius: " + str(radius))

                # only proceed if the radius meets a minimum size
                if radius > 8:
                    # draw the circle and centroid on the frame,
                    # then update the list of tracked points
                    cv2.circle(image_np, (int(x), int(y)), int(radius),
                               (0, 255, 255), 2)
                    cv2.circle(image_np, center, 5, (0, 0, 255), -1)
                    # Send the current state of the ball w.r.t. the robot
                    msg = BallState()
                    msg.state = True
                    msg.center = center
                    msg.radius = radius
                    msg.color = self.colors[i]

                    self.ball_state_pub.publish(msg)
                    self.ball_check[i] = True
            else:
                # The ball of the i-th color has not been seen
                self.ball_check[i] = False

            """ # send ball state equal to False only if no ball is perceived
            if i == (len(self.colors)-1) and np.nonzero(self.ball_check)[False].size == 0:
                # Send the current state of the ball w.r.t. the robot
                msg = BallState()
                msg.state = False
                self.ball_state_pub.publish(msg) """

        # Show the current image
        small_im = scipy.misc.imresize(image_np, 0.7)
        cv2.imshow('camera', small_im)
        cv2.waitKey(2)

## Main function
#
# It defines an object of the type ImageFeature


def main(args):
    ic = ImageFeature()
    print("Ball Detection Node:")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ("Shutting down ROS Image feature detector module")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
