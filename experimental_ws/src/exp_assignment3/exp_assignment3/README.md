# Robot dog Architecture - Assignment 3

   - __Author:__ Roberto Albanese
   - __Version:__ 1.0
   - __Date:__ -05-2021

## Introduction
This is the last assignment of the course *Experimental Robotics Laboratory* of the Master degree of Robotics Engineering of the University of Genova. The objective of the project is to build a ROS architecture capable to implement different behaviors. This package represents the last step of the learning process of ROS development.
In this project folder the reader will encounter the following folders:
- [action](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/action) Action server message;
- [config](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/config) Configuration file for Rviz layout;
- [doc](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/doc/html) Doxygen documentation;
- [img](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/img) Report images;
- [launch](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/launch) Launch files;
- [msg](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/msg) Message files;
- [param](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/param) Move base parameters;
- [scripts](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/scripts) Python source files;
- [src](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/src) Cpp source code;
- [srv](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/srv) Service messages;
- [urdf](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/urdf) Urdf models;
- [world](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/world) World model.

The aim of this assignment is to cover essential concepts, i.e. SLAM algorithms, autonomous navigation and features recognition, for the development of a mobile robot capable of moving autonomously.

## Software Architecture and System's Features
A two-driving-wheeled robot was chosen for the project, equipped with an RGB camera used for feature recognition and a hokuyo laser used for SLAM. The robot has five possible behaviors: it can **Sleep**, **Play**, stay in a **Normal** state, explore the map to **Find** the balls and **Track** each ball. Below it will be explained in detail each state of the robot with a focus on design decisions.
<p align="center">
  <img src="https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/blob/master/experimental_ws/src/exp_assignment3/img/FSM.png" />
</p>

- ### Normal
    This is the initial state of the *FSM* and it is one of the more similar to the previous assignment. In this state the robot can move randomly in the house. The Navigation is managed by the package *move_base*, which receives a goal position and controls the robot by planning both a global and local path to bring it there. Since a SLAM alorithms is implemented in this project, the house is assumed to be unknown, as well as the location and dimension of each room. It follows that, in the **Normal** state, the robot cannot move freely in the house, but it can visit only known locations of it. Due to this design choice, the house has been divided in 9 sectors, which can be visited only when the room becomes known, i.e. the corresponding ball has been detected. Below it is shown how the sectors are defined:
    ```
    Aggiungi immagine settori
    ```

- ### Sleep
    The robot can go in the **Sleep** state only from the **Normal** state. In the **Sleep** state the robot reaches the predefined location [ -4 , 7 ] and waits there for a bunch of seconds. After that, it returns back in the **Normal** state.
- ### Play
    The robot reaches the **Play** state only from the **Normal** state. The transition happens whenever the robot receives a *"play"* command from the user. Once the command is received, the robot goes to the predefined location [ -5 , 7 ] (user position) and waits for a location to reach. If the robot knows the location (i.e the corresponding ball has already been seen) the robot navigates until it reaches it and stops there for a bunch of seconds and then returns in the **Play** state; if the location is unknown (i.e the corresponding ball has not been seen yet) the robot transitions to the **Find** state to look for it. After some time in the **Play** state the robot goes back to the **Normal** state.
- ### Find
    In this state the robot explores the unknow locations of the house to find the missing balls. For this purpose, it was chosen to implement the *explore-lite* package, which cooperate with the *move_base* and the SLAM algorithm to control the robot. Whenever an unknown ball is seen, the robot goes to the **Track** state. Then, if the ball is the one corresponding to the requested location of the **Play** state, the robot goes back to the **Play** state, whereas if it does not correspond to it, the robot goes back to the **Find** state.
- ### Track 
    In this state the robot is controlled to move close to an unknow detected ball. As soon as it reaches it, it saves its position as the location of the corresponding room. Then it goes back to the previous state.

    
The software architecture is composed by six elements, as shown in the figure:
<p align="center">
  <img src="https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/blob/master/experimental_ws/src/exp_assignment3/exp_assignment3/img/Architecture.png" />
</p>

* __User Command__: Human command to start playing with the robot;
* __Command Manager__: Logic core of the architecture in which the **FSM** is;
* __Navigation__: Motion of the robot;
* __Perception__: Detection of the balls;
* __Exploration__: Exploration of the unknow locations;
* __SLAM__: SLAM algorithm

*User Command* block represents the behaviour of the human. When the robot is in the **Normal** state the human can *manually* send a *"play"* command to start to play with the robot. It has been decided to send the command by writing a string message in the topic */user/play_command*. Then an action server is settled up so that the human can wait for the robot to reach the predefined position and then send the goal location. In this way two diffents nodes can work in parallel and send the needed messages only when the are both ready.

*Command Manager* block represents the FSM controlling the program. It is composed of the five states and uses the following userdata to keep inside the machine some informations:
* *room_dictionary*: Holds all the Knowledge information of the program, i.e. room/ball correlation, room dimensions, ball positions, ball detection flag;
* *previous_state*: Contains the state from which the **Track** state was reached;
* *requested_ball*: Contains the ball color requested from the **Play** state. It is used in the **Find** state to check if the ball seen was the requested one.

*Navigation* block is composed of the *move_base* action server which includes the global and the local path planning and the obstacles avoidance provided by the package ROS Navighation Stack.

*Perception* block detects the presence of a ball using an *open_cv* algorithm which percive the presence of one of the six-ball color inside the image received from the camera. It was necessary to edit the color of the chair model of the human because the brown shade of the wood was detected as *red* from the algorithm.

*Exploration* block is exectuted by the *explore_lite* package. For a better performance it was decided to decrease the hokuyo laser range in order to be consistent with the range of the camera. A too large difference in range resulted in the balls not being detected in the detection phase of the **Find** state: this is due to the fact that some location where considered already visited from the *explore_lite* algorithm (because present in the map) but never seen by the camera.

*SLAM* block exploits the *gmapping* package which provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping, creating a 2-D occupancy grid map from laser and pose data collected by the robot.

### File list
In the [source folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/scripts) it is possible to find five files which compose the whole architecture:

* __usr_cmd_client.py__: the action server */user/go_to_command* is set up. The user *manually* sends the *"play"* command in the */user/play_command* topic and the decides where the robot ha to go. 
* __ball_perception.py__: it uses six different masks to detect the presence of one the six ball color and send a message in the topic *ball/state* to the FSM with all the informations of the detected ball.
* __state_machine.py__ is the core node that manages information from *Perception*. It initializes and executes a state machine, using the library *smach_ros*, in which all the five states and their behaviours are defined.<br>

In the [message folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/msg) is it possible to find the file used by the node *Perception*:
* __BallState.msg__: it represents the state of the ball w.r.t. the robot. Here we can find some informations, as the position of the center of the ball w.r.t. the image and its radius. *state* used as flags and color of the ball.
```
bool state
int32[2] center
float64 radius
string color
```
In the [server folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/srv) is it possible to find the file used by the node *User Command*:
* __GoToCommand.srv__: it represents the message of the server with the request sent in the **Play** state and the response of the *User*
```
string request
---
string location
```

In the [launch folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/exp_assignment3/exp_assignment3/launch) it is possible to find the launch file used to execute all the nodes:
* **exp_assignment3.launch**: launch file of the final project; it launches the Gazebo and RVIZ environments, gmapping algorithm, move_base server and the FSM;
* **fsm.launch**: it launches the FSM node and the *smach_viewer* node;
* **gmapping.launch**: it loads all the parameters for the gmapping and launches the gmapping node
* **move_base.launch**: it loads all the parameters for the move_base and enables the move_base action server
* **simulation.launch**: it sets up the Gazebo and RVIZ environmets with che corresponding models. 

## Installation
The project was developed using the docker image given in the beginning of the course as a starting point. It is possible to install it by following the instractions in this link: https://hub.docker.com/r/carms84/rpr .<br>
There are few steps to follow before being able to execute the code. Firs of all we need to intall the _smach_ library to let ROS to work with finite state machines:
```
$ sudo apt-get install ros-kinetic-smach-viewer
```
Then it is also necessary to install the package imutils (install pip: https://pip.pypa.io/en/stable/installing/):

```
$ pip install imutils
```
Then is it necessary to give running permission to the python source files:
* Open the terminal
* Move into the folder /../experimental_ws/src/exp_assignment3/scripts
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
$ roslaunch exp_assignment3 gazebo_world.launch 
```
The launch file is done in a way that it makes possible to see the state of the **FSM** in the *smach_viewer* node automatically.<br>
A window will pop up showing the image taken from the camera.
In the terminal in which the roslaunch command is executed it is possible to see the current positions of the robot.<br><br>
In order to manually execute the nodes/services to see in each terminal the data generated from each node/service, it is required to *rosrun* each node/service listed in *exp_assignment3.launch* in the exact same order they are listed in.
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
