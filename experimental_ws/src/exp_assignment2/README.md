# Robot dog Architecture - Assignment 2

   - __Author:__ Roberto Albanese
   - __Version:__ 1.0
   - __Date:__ 12-10-2020

## Introduction
This is the second assignment of the course *Experimental Robotics*. I am asked to build a ROS architecture to implement a dog alike robot’s behavior.
In this project folder the reader will encounter the following folders:
- [action](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/action) Action server message;
- [config](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/config) Configuration file for head transmission;
- [doc](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/doc/html) Doxygen documentation;
- [launch](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/launch) Launch files;
- [msg](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/msg) Message filse
- [scripts](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/scripts) Source files;
- [urdf](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/urdf) Urdf models;
- [world](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/world) World model.

## Software Architecture and System's Features
The scenario is represented by a robot, simulating a pet, that interacts with a human and moves in a discrete 2D environment. <br>
The robot has three possible behaviors: it can Sleep, Play or stay in a Normal state.<br><br>
<p align="center">
  <img src="https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/blob/master/experimental_ws/src/exp_assignment2/FSM.png" />
</p>

In the Normal state the robot has to move randomly.<br><br>
In the Sleep state the robot goes back to the human and sleeps there for a random amount of time. Then it goes back in the Normal state. The robot can pass to the Spleep state only from the Normal state in a random time instant.<br><br>
In the Play state the robot follows the green ball. If the robot reaches the ball and the ball stops, the robot has to look around with his head and then continue following the ball, if possible. The robot can reach the Play state whenever it sees the ball. The robot remains in the Play state untill it cannot see the ball for a certain amount of time (3s).<br><br>
The software architecture is composed by five elements:
<p align="center">
  <img src="https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/blob/master/experimental_ws/src/exp_assignment2/Architecture.png" />
</p>

* __User Command__: it represents the human command to move the ball;
* __Command Manager__: it is the main part of the architecture in which the **FSM** is;
* __Robot Navigation__: it manages the motion of the robot and brings it to the new position goal;
* __Ball Navigation__: it manages the motion of the robot and brings it to the new position goal;
* __Perception__: Detects the presence of the green ball.

Robot Navigation and Ball Navigazion are action servers because they need to execute only when a request is send and they need to be able to stop in any time. Feedback messages are also useful to track the execution of the servers.
User Command is meant to be an action server because it has to send requests to Ball Navigation.
Perception is a node because it has to live in background and always execute.
Command Manager, as well, needs to execute asyncronously ant it is intended to be a node.
### File list
In the  [source folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/scripts) it is possible to find five files which compose the whole architecture:

* __usr_cmd_client.py__: in here I have defined an action client which randomly sends reuests to the action server '*/ball/reaching_goal*';
* __navigation.py__: it waits for a request message of the type *Planning.action* in which it is present the new position where the robot needs to go, then brings the robot to the goal position by publishing velocities in the topic '*/robot/cmd_vel*';
* __go_to_point_ball.py__: same exact behavior of *navigation.py*. The only difference is that the nodes controls the motion of the ball by publishing velocities in the topic '*/ball/cmd_vel*';
* __state_machine.py__ is the core node that manages information from *Perception* and *Robot navigation*. It initializes and executes a state machine, using the library *smach_ros*, in which all the three states and their behaviours are defined.<br>

In the [action folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/action) is it possible to find the file used by the action servers *Robot navigation* and **Ball Navigation:
* __Planning.action__: the goal *target_pose* represents the position we want to reach. Result in empty. In the feedback are stored the state of the action server and the current position of the object.
```
geometry_msgs/PoseStamped target_pose
---
---
string stat
geometry_msgs/Pose position

```

In the [message folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/msg) is it possible to find the file used by the node *Perception*:
* __BallState.msg__: it represents the state of the ball w.r.t. the robot. Here we can find some informations, as the position of the center of the ball w.r.t. the image and its radius. *state* and *ball_reached* are used as flags.
```
bool state
bool ball_reached
int32[2] center
float64 radius

```

In the [launch folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment2/launch) it is possible to find the launch file used to execute all the nodes.

## Installation
All the following installation are needed by using the docker image given in the beginning of the course. It is possible to install it by following the instraction in: https://hub.docker.com/r/carms84/rpr .<br>
There are few steps to follow before to being able to execute the code. Firs of all we need to intall the _smach_ library to let ROS to work with finite state machines:
```
$ sudo apt-get install ros-kinetic-smach-viewer
```
Then it is also necessary to install the package imutils (install pip: https://pip.pypa.io/en/stable/installing/):

```
$ pip install imutils
```
Then is it necessary to give running permission to the python source files:
* Open the terminal
* Move into the folder /../experimental_ws/src/exp_assignment2/scripts
* Launch the command:
```
$ chmod +x name_of_the_file.py
```

It's also required to make a _catkin_make_ in your _ROS_ workspace. Check [ROS tutorials](http://wiki.ros.org/catkin/Tutorials) to see how to do it.    

## Usage
First of all it is required to source your workspace setup.bash file. To do it open a terminal, move into the workspace directory and the launch the command:
```
source devel/setup.bash
```

If wou want run the whole project just type:
```
$ roslaunch exp_assignment2 gazebo_world.launch 
```
The launch file is done in a way that it makes possible to see the state of the **FSM** in the *smach_viewer* node automatically.<br>
A window will pop up showing the image taken from the camera.
In the terminal in which the roslaunch command is executed it is possible to see the current positions of the robot.<br><br>
In order to manually execute the nodes/services to see in each terminal the data generated from each node/service, it is required to *rosrun* each node/service listed in *exp_assignment2.launch* in the exact same order they are listed in.
## Software Limitation and Possible technical Improvements
### Limitations
- The routine used to block the robot when the ball stops is not optimal; sometimes the robot think that the ball stopped becuase of the *if* condition too simplified (the FSM uses the velocity of the robot and the radius of the ball to understand if it has stopped: when the robot is going slow and the ball passes in front of it, the condition is verified (false positive)).
- The robot is not able to avoid obstacles.
### Possible Improvements
- Improvement of the condition to recognise the stopping ball
- Obstacle avoidance routine (SLAM and trajectory planning)
## Contacts
Roberto Albanese ralbanese18@gmail.com
<br>
<br>
<br>

**Robotics Engineering Master - 2nd year**  <br>
**University of Genoa**
