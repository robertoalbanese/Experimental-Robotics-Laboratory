#!/usr/bin/env python

"""!    @file state_machine.py
        @brief **FSM** of the architecture.

        As it is described in the mainpage, this code is mainly populated from the smach_ros library, which allows me to define and execute a state machine for the application."""

# Ros library
import roslib
import rospy
import roslaunch

# Finite state machine
import smach
import smach_ros

# Useful libraries
import time
import random
import os

# Actionlib
import actionlib
import actionlib.msg

# Messages
from exp_assignment3.msg import BallState
from std_msgs.msg import Float64, String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from exp_assignment3.srv import GoToCommand, GoToCommandResponse

## Ball state.
# Describes the state of the ball w.r.t. the robot.<br><br>
# Ball State msg:
# <blockquote>bool state<br>
# int32[2] center<br>
# float64 radius<br>
# string color<br>

ball_state = BallState()

## Odometry.
# Describes the odometry information of the robot.
odometry = Odometry()

## Feedback callback for the action client to '/move_base'.
# This function is called whenever is received a feedback from the action server
# @param msg feedback message


def feedback_cb(msg):
    print 'Feedback received:', msg

## Action client to the server '/move_base'.
# The client send a goal position whenever the robot need to go in a new position
# @param x coordinate of the new position
# @param y coordinate of the new position


def movebase_client(x, y):
    try:
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0

        # client.send_goal(goal, feedback_cb=feedback_cb)
        client.send_goal(goal)
        return

    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

## Callback function of 'msg_ballstate_sub'.
# This function is called whenever a new ball state is published in the topic '/ball/state'
# @param msg ball_state message


def update_state(msg):
    global ball_state
    ball_state = msg

## Callback function of 'odom_sub'.
# This function is called whenever is received an Odometry message from the topic '/odom'
# @param odom odometry info


def odom_callback(odom):
    global odometry
    odometry = odom


## Definition of state Sleep
#
# In this state the robot will go to sleep in the given position [-4, 7] and then it will return in the Normal state.


class Sleep(smach.State):
    ## Initialization of the state.
    #
    # The outcomes are defined.
    def __init__(self):
        # initialisation function
        smach.State.__init__(self,
                             outcomes=['gotoNormal'])

    # Body of the class

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(1)
        print('')
        rospy.loginfo('Gonna take a nap.')

        # The robot reaches the predefined position
        movebase_client(-4, 7)
        client.wait_for_result()
        print('')

        for i in range(random.randint(4, 6)):
            print("Zzz Zzz Zzz")
            time.sleep(1)

        # After some time the robot goes back to the Normal state
        print('')
        rospy.loginfo('I just woke up!')
        time.sleep(1)
        return 'gotoNormal'

## Definition of state Normal
#
# In this state the robot will randomly navigate untill either the ball is seen
# or the robot randomly decides to go to Sleep
# or the user sends a "play" command.


class Normal(smach.State):
    ## Initialization of the class.
    #
    # This constructor initializes the susbscriber to the topic "/user/play_command" to receive the "play" command from the user.<br>

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoSleep', 'gotoPlay',
                                       'gotoNormal', 'gotoTrack'],
                             input_keys=['normal_dictionary_in'],
                             output_keys=['normal_dictionary_out', 'normal_next_out'])

        ## Subscriber to  "/user/play_command". It detects an incoming "play" command
        self.play_sub = rospy.Subscriber(
            "/user/play_command", String, self.play_command)

        ## @private Set high whenever a "play" command is received
        self.play_flag = False

    ## Callback function of the subscriber to the topic "/user/play_command" 
    def play_command(self, data):
        if data.data == "play":
            self.play_flag = True

    ## Body of the class.
    def execute(self, userdata):
        # function called when exiting from the node
        time.sleep(1)
        tmp = userdata.normal_dictionary_in

        new_goal = []

        # Move randomly only in the visited portion of the map
        for id, info in tmp.items():
            if info['seen'] == True:
                new_goal_x = random.uniform(
                    tmp[id]['coord_x'][0], tmp[id]['coord_x'][1])
                new_goal_y = random.uniform(
                    tmp[id]['coord_y'][0], tmp[id]['coord_y'][1])
                new_goal.append([new_goal_x, new_goal_y])

        goal = random.choice(new_goal)
        rospy.loginfo("I'll go to [%f,%f]", goal[0], goal[1])
        time.sleep(2)

        # Calling the action server
        movebase_client(goal[0], goal[1])

        # The goal can be canceled in any moment
        while not (client.get_result()):
            # if play command has been recived --> gotoPlay
            if self.play_flag == True:
                self.play_flag = False
                client.cancel_all_goals()
                userdata.normal_next_out = 'gotoNormal'
                return 'gotoPlay'
            # If a ball has been seen --> gotoTrack
            color = ball_state.color
            if ball_state.state == True and tmp.get(color):
                if tmp[color]['seen'] == False:
                    client.cancel_all_goals()
                    userdata.normal_next_out = 'gotoNormal'
                    return 'gotoTrack'

        # After the robot reaches the goal it randomly goes to Sleep.
        if (random.randint(1, 6) == 1):
            return 'gotoSleep'

        # Otherwise
        return 'gotoNormal'

        # Wait for ctrl-c to stop the application
        rospy.spin()

## Definition of state Play
#
# In this state the robot follows the ball untill it cannot see it anymore for a certain amount of time (3 s).


class Play(smach.State):
    ## Initialization of the class.
    #
    # This constructor initializes the publisher executed by the class:<br>
    # - A publisher to the topic "/cmd_vel" to send the control velocities to the robot.<br>

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoNormal', 'gotoPlay', 'gotoFind'],
                             input_keys=['play_dictionary_in'],
                             output_keys=['play_dictionary_out', 'play_color_ball_out'])

        ## @private Publisher vel_pub to move the robot
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        ## @private Iteraction limit. The robot stops playing after certain iteractions
        self.iter = random.randint(2, 5)
        ## @private Increasing counter
        self.n = 0

    ## Client service connecting to "/user/go_to_command". Used to request a new location to the user

    def request_pos_client(self, msg):
        rospy.wait_for_service('/user/go_to_command')
        try:
            req_loc = rospy.ServiceProxy('/user/go_to_command', GoToCommand)
            resp = req_loc(msg)
            return resp
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

    ## Body of the class

    def execute(self, userdata):
        # function called when exiting from the node, it can be blocking
        time.sleep(1)
        tmp = userdata.play_dictionary_in
        self.n = self.n + 1

        # Stop playing if a certain number of iteraction has been executed and return to NORMAL state
        if (self.n == self.iter):
            self.n = 0
            self.iter = random.randint(2, 5)
            print('')
            print("Robot: I'm fed up, I don't want to play anymore.")
            print('')
            end = self.request_pos_client('end')
            return 'gotoNormal'

        # Go to the user
        movebase_client(-5.0, 7.0)
        client.wait_for_result()

        print('')
        print("Robot: I'm arrived, where do I go now?")

        while 1:
            new_loc = self.request_pos_client('')

            for id, info in tmp.items():
                if (tmp[id].get('room') == new_loc.location.lower()):
                    ball_color = id

            print('')

            if not ball_color:
                print(
                    "The received room is unknown or it doesn't exist! Wait for another location.")
            else:
                if tmp[ball_color]['seen'] == True:
                    print("Robot: I'll go the the " + ball_color + " ball")
                    movebase_client(tmp[ball_color]['x'], tmp[ball_color]['y'])
                    client.wait_for_result()
                    print("Robot: I'm arrived!")
                    print('')
                    time.sleep(3)
                    userdata.play_dictionary_out = tmp
                    return 'gotoPlay'
                else:
                    print('The ' + new_loc.location.lower() +
                          ' is not been discovered yet, as well as the ' + ball_color + ' ball.')
                    print("Robot: I'll go to find it!")
                    print('')
                    userdata.play_color_ball_out = ball_color
                    userdata.play_dictionary_out = tmp
                    return 'gotoFind'
        # Wait for ctrl-c to stop the application
        rospy.spin()

## Definition of state Find
#
# In this state the robot explores the unknow locations of the map.


class Find(smach.State):
    ## Initialization of the class.
    # The path of the explore-lite launch file is loaded

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoPlay', 'gotoFind', 'gotoTrack'],
                             input_keys=['find_dictionary_in',
                                         'find_color_ball_in'],
                             output_keys=['find_dictionary_out', 'find_next_out'])

        ## @private Explore-lite launch file path
        self.path = rospy.get_param("explore-lite path")

        ## @private First iteration check
        self.first = True

    ## Body of the class.
    # In this state the robot uses the information of the map to explore the unknown rooms.<br>
    # The explore-lite node is launched within this function.
    def execute(self, userdata):
        # function called when exiting from the node
        time.sleep(1)
        tmp = userdata.find_dictionary_in
        ball_color = ''

        # Launch the eplore-lite package
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [self.path])
        launch.start()
        time.sleep(5)

        # Timeout condition initialization
        now = rospy.get_rostime()
        if self.first == True:
            self.first = False
            ## @private End time of time-out condition
            self.then = rospy.get_rostime()

        # If no ball has been seen after 60 secs shutdown explore-lite pkg
        while (now.secs - self.then.secs) < 60:

            for id, info in tmp.items():
                if (id == ball_state.color):
                    ball_color = id

            if ball_state.state == True and ball_color:
                if tmp[ball_color]['seen'] == False:
                    ball_color = ''
                    # if this is the ball i was looking for --> 'gotoTrack' and then --> 'gotoPlay'
                    if ball_state.color == userdata.find_color_ball_in:
                        launch.shutdown()
                        client.cancel_all_goals()
                        self.first = True
                        userdata.find_dictionary_out = tmp
                        userdata.find_next_out = 'gotoPlay'
                        return 'gotoTrack'
                    # else 'gotoTrack' and then --> 'gotoFind'
                    else:
                        launch.shutdown()
                        client.cancel_all_goals()
                        userdata.find_next_out = 'gotoFind'
                        userdata.find_dictionary_out = tmp
                        return 'gotoTrack'
                ball_color = ''

            # Update timeout condition
            now = rospy.get_rostime()

        # If the robot was not able to find the ball it was looking for
        self.first = True
        client.cancel_all_goals()
        launch.shutdown()

        print("The robot was not able to find the ball within 60 secs. It will go back to the user!")
        userdata.find_dictionary_out = tmp
        return 'gotoPlay'

        # Wait for ctrl-c to stop the application
        rospy.spin()

## Definition of state Track.
#
# In this state the robot reaches the ball and saves its position.


class Track(smach.State):
    ## Initialization of the state.
    #
    # This constructor initializes the publisher executed by the class:<br>
    # - A publisher to the topic "/cmd_vel" to send the control velocities to the robot.<br>

    def __init__(self):
        # initialisation function, it should not wait
        smach.State.__init__(self,
                             outcomes=['gotoFind', 'gotoNormal',
                                       'gotoTrack', 'gotoPlay'],
                             input_keys=['track_dictionary_in',
                                         'track_previous_in'],
                             output_keys=['track_dictionary_out'])
        ## @private Publisher vel_pub to move the whole robot
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    ## Body of the class
    def execute(self, userdata):
        # function called when exiting from the node

        global ball_state
        vel = Twist()
        tmp = userdata.track_dictionary_in

        # Track until the robot can't see the ball
        while ball_state.state == True:
            # 600 is the center of the image
            vel.angular.z = -0.005*(ball_state.center[0]-300)
            # 65 is the radius that we want see in the image, which represent the desired disatance from the object
            vel.linear.x = min(-0.01*(ball_state.radius-65), 0.6)

            # if we reach the ball
            if ball_state.radius > 30 and abs(ball_state.center[0]-300) < 10:
                # stop
                vel.linear.x = 0
                vel.angular.z = 0
                self.vel_pub.publish(vel)

                # Store the position of the ball in the dictionary
                tmp[ball_state.color]['seen'] = True
                tmp[ball_state.color]['x'] = odometry.pose.pose.position.x
                tmp[ball_state.color]['y'] = odometry.pose.pose.position.y

                # Unlock the second portion of the house
                if ball_state.color == 'blue':
                    # Corridor_1 and entrance represent the same room. They are mutually exclusive
                    tmp['corridor_1']['seen'] = False
                    tmp['corridor_2']['seen'] = True
                    tmp['corridor_3']['seen'] = True

                userdata.track_dictionary_out = tmp
                return userdata.track_previous_in
            else:
                self.vel_pub.publish(vel)

        # If the robot has lost the ball
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        rospy.loginfo(
            "The robot has lost the ball! It will go back to the previous state ")
        userdata.track_dictionary_out = tmp
        return userdata.track_previous_in

        # Wait for ctrl-c to stop the application
        rospy.spin()

## Main body of the program.
#
# This function initialize some of the structures executed by the program:<br>
# - A subscriber to the topic "/ball/state" to read the current ball state w.r.t. the robot.<br>
# - A global move_base action server to control the robot
# - A subscriber to the "/odom" topic
# - A smach state machine.<br>


def main():
    rospy.init_node('Assignment_3_FSM')

    # A subscriber to the topic '/ball/state'. update_state is called
    # when a message of type BallState is received.
    msg_ballstate_sub = rospy.Subscriber(
        '/ball/state', BallState, update_state)

    global client
    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server(rospy.Duration(10.0))

    odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])

    # Userdata initialization
    
    ## Room dictionary.
    # Dictionary containing all the information about rooms and balls. Each ball is connected with the right room and marked as seen or unseen.
    sm.userdata.room_dictionary = {'blue':          {'room': 'entrance',    'seen': False,  'x': '',    'y': '',    'coord_x': [-5.0, -1.0],    'coord_y': [4.5, 7.5]},
                                   'red':           {'room': 'closet',      'seen': False,  'x': '',    'y': '',    'coord_x': [-5.0, -2.5],    'coord_y': [1.5, 2.0]},
                                   'green':         {'room': 'living room', 'seen': False,  'x': '',    'y': '',    'coord_x': [-5.0, 0],       'coord_y': [-4.5, -1.0]},
                                   'yellow':        {'room': 'kitchen',     'seen': False,  'x': '',    'y': '',    'coord_x': [0.5, 5.0],      'coord_y': [-7.5, -7.0]},
                                   'magenta':       {'room': 'bathroom',    'seen': False,  'x': '',    'y': '',    'coord_x': [3.5, 5.0],      'coord_y': [-4.5, -3.0]},
                                   'black':         {'room': 'bedroom',     'seen': False,  'x': '',    'y': '',    'coord_x': [3.5, 5.0],      'coord_y': [-0.5, 2.0]},
                                   'corridor_1':    {'seen': True,                                                  'coord_x': [-5.0, -1.0],    'coord_y': [4.5, 7.5]},
                                   'corridor_2':    {'seen': False,                                                 'coord_x': [-1.5, -0.5],    'coord_y': [0.5, 2.5]},
                                   'corridor_3':    {'seen': False,                                                 'coord_x': [0, 1.5],        'coord_y': [-4.5, -0.5]}}

    sm.userdata.previous_state = 'gotoNormal'  # The fist state of sm will be NORMAL
    sm.userdata.requested_ball = ''

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('NORMAL', Normal(),
                               transitions={'gotoSleep': 'SLEEP',
                                            'gotoPlay': 'PLAY',
                                            'gotoNormal': 'NORMAL',
                                            'gotoTrack': 'TRACK'},
                               remapping={'normal_dictionary_in': 'room_dictionary',
                                          'normal_dictionary_out': 'room_dictionary',
                                          'normal_next_out': 'previous_state'})
        smach.StateMachine.add('FIND', Find(),
                               transitions={'gotoPlay': 'PLAY',
                                            'gotoFind': 'FIND',
                                            'gotoTrack': 'TRACK'},
                               remapping={'find_dictionary_in': 'room_dictionary',
                                          'find_dictionary_out': 'room_dictionary',
                                          'find_next_out': 'previous_state',
                                          'find_color_ball_in': 'requested_ball'})
        smach.StateMachine.add('PLAY', Play(),
                               transitions={'gotoNormal': 'NORMAL',
                                            'gotoPlay': 'PLAY',
                                            'gotoFind': 'FIND'},
                               remapping={'play_dictionary_in': 'room_dictionary',
                                          'play_dictionary_out': 'room_dictionary',
                                          'play_color_ball_out': 'requested_ball'})
        smach.StateMachine.add('SLEEP', Sleep(),
                               transitions={'gotoNormal': 'NORMAL'})
        smach.StateMachine.add('TRACK', Track(),
                               transitions={'gotoNormal': 'NORMAL',
                                            'gotoFind': 'FIND',
                                            'gotoTrack': 'TRACK',
                                            'gotoPlay': 'PLAY'},
                               remapping={'track_dictionary_in': 'room_dictionary',
                                          'track_dictionary_out': 'room_dictionary',
                                          'track_previous_in': 'previous_state'})

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
