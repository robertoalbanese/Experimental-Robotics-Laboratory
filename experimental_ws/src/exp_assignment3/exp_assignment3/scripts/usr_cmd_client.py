#!/usr/bin/env python

"""!    @file usr_cmd_client.py
        @brief User interface to play with the robot in the PLAY state.

        With this file the human can interact with the robot and command it to go in a certain location """

# Ros library
import rospy

# Useful library
import time
from random import randrange

# Actionlib
import actionlib
import actionlib.msg

# Custom message
from exp_assignment3.srv import GoToCommand, GoToCommandResponse

# Messages
from std_msgs.msg import String


## Definition of class User.
#
# A simple UI is built up to let the user interact with the robot


class User():
    ## Initialization of the class.
    #
    # This constructor initialize some of the publisher and services executed by the program:<br>
    # - A publisher to the topic "/user/play_command" to publish the "play" coomand to the robot.<br>
    # - A service to the topic "/user/go_to_command" to send the location only when it's requested.<br>
    def __init__(self):
        # initialize the node
        rospy.init_node('user_command_client')

        ## @private send the "play" command to the fsm
        self.cmd_pub = rospy.Publisher(
            "/user/play_command", String, queue_size=1)

        ## @private send the new location to reach to the robot
        self.goto_service = rospy.Service(
            "/user/go_to_command", GoToCommand, self.srv_callback)

        ## @private Rooms of the house
        self.rooms = ['entrance', 'closet', 'living room',
                      'kitchen', 'bathroom', 'bedroom']

        ## @private Flag used to wait for the request of the server
        self.flag = False

        ## @private Flag setted up whenever the robot exits from the PLAY state
        self.end_flag = False

    ## Core function of the class.
    # The user can decide to go in the Play state sending a "play" command
    # and then point where the robot has to go.
    def run(self):
        print("""          
Hello user! You will be free to choose the location that the robot has to reach.
    Here a little reminder of where the balls are located:

                        Blue    ----->   Entrance
                        Red     ----->   Closet
                        Green   ----->   Living room
                        Yellow  ----->   Kitchen
                        Magenta ----->   Bathroom
                        Black   ----->   Bedroom
            
You can guide the robot by sending a "play" command whenever you want and then by choosing a room! """)
        while 1:
            # python 3.0
            # cmd = input('Command: ')

            # python 2.7
            cmd = raw_input('Command: ')

            if cmd == "play":
                self.cmd_pub.publish(cmd)

                # Play until the robot changes state
                while self.end_flag == False:
                    while self.flag == False:  # wait for the client request
                        pass
                    self.flag = False
                self.end_flag = False
            else:
                print("""Unexpected string! To send a play command write "play".""")

    ## Callback function of the server to the topic '/user/go_to_command'
    #
    # After receiving a request, the user can choose the next location.
    # @param req Incoming request from the client

    def srv_callback(self, req):
        # if the request is "end" set up the end_flag
        if req.request == "end":
            self.flag = True
            self.end_flag = True
            return GoToCommandResponse()
        # else wait for a right location
        while 1:
            loc = raw_input("Go to: ")
            if loc.lower() in self.rooms:
                self.flag = True
                return GoToCommandResponse(loc)
            else:
                print("Unknown location.")


## User command action client initialization.
# Declaration of the User class

def main():
    u = User()
    u.run()
    # Wait for ctrl-c to stop the application
    rospy.spin()


if __name__ == '__main__':
    main()
