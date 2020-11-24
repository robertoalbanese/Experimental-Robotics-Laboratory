# Behavioral Architecture - Assignment 1

   - __Author:__ Roberto Albanese
   - __Version:__ 1.0
   - __Date:__ 10-28-2020

## Introduction
This is the first assignment of the course *Experimental Robotics*. I am asked to build a ROS architecture to implement a dog alike robot’s behavior.
In this project folder the reader will encounter the following folders:
- [doc](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/doc/html) Doxygen documentation;
- [launch](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/launch) Launch files;
- [src](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/src) Source files;
- [srv](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/srv) Service files.

## Software Architecture and System's Features
The scenario is represented by a robot, simulating a pet, that interacts with a human and moves in a discrete 2D environment. <br>
The robot has three possible behaviors: it can Sleep, Play or stay in a Normal state.<br><br>
<p align="center">
  <img src="https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/blob/master/experimental_ws/src/assignment_1/FSM.png" />
</p>

In the Normal state the robot has to move randomly.<br><br>
In the Sleep state the robot chooses a random location and sleep there for a random amount of time. Then it goes back in the Normal state. The robot can pass to the Spleep state only from the Normal state in a random time instant.<br><br>
In the Play state the robot reaches the User and waits until it points to a random position. The robot has to reach the position and then come back to the user. The robot can reach the Play state only from the Normal state and only when a "Play" command is recived. The robot can remain in the Play state for multiple iterations.<br><br>
The software architecture is composed by four elements:
<p align="center">
  <img src="https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/blob/master/experimental_ws/src/assignment_1/Architecture.jpg" />
</p>

* __User Command__: it represents the user command "play";
* __Command Manager__: it is the main part of the architecture in which the **FSM** is;
* __Navigation__: it manages the motion of the robot and brings it to the new position;
* __Random Position Generator__: generates random positions.

Navigation and Random Position Generator are services because they are thought to operate in a syncronous mode. They will generate a new position and move the robot to the generated position only when it is specifically requested from the code. On the other hand, User Command is meant to be a node because it has to behave in an asyncronous mode (it must send a command randomly).
### File list
In the  [source folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/src) it is possible to find four files which compose the whole architecture:

* __usr_cmd.cpp__: it is the only node leaving out the state machine. In here I have defined a publisher which randomly publishes a string "Play" in the topic *hw1_usr_cmd*;
* __rand_position.cpp__: it is the fist service of the architecture. It waits for a request message of the type *get_pos.srv* and randomly generates a 2D position;
* __navigation.cpp__: it is the second service of the architecture. It waits for a request message of the type *reach_next_pos.srv* in which it is present the new position where the robot needs to go, waits for a reasonable amount of time to simulate a motion, and gives as response the new current position;
* __state_machine.py__ is the core node that manages information from the two services and the publisher. It initializes and executes a state machine, using the library *smach_ros*, in which all the three states and their behaviours are defined.<br>

In the [service folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/srv) is it possible to find the two files used by the services:
* __get_pos.srv__: the request part of the message represents the boundaries of the map. The service cannot generate a random position outside of the map. The respose part represents the new generated random position.
```
int64 minx
int64 maxx
int64 miny
int64 maxy
---
int64 x
int64 y
```
* __reach_next_pos.srv__:the request part of the message is formed by the new position that the robot has to reach while the response part represents the new current position after the motion.
```
int64 x
int64 y
---
int64 x
int64 y
```
In the [launch folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/launch) it is possible to find the launch file used to execute all the nodes.

## Installation
There are few steps to follow before to being able to execute the code. Firs of all we need to intall the _smach_ library to let ROS to work with finite state machines:
```
$ sudo apt-get install ros-kinetic-smach-viewer
```
Then is it necessary to give running permission to the file *state_machine.py*:
* Open the terminal
* Move into the folder /../experimental_ws/src/assignment_1/src
* Launch the command:
```
$ chmod +x state_machine.py
```

It's also required to make a _catkin_make_ in your _ROS_ workspace. Check [ROS tutorials](http://wiki.ros.org/catkin/Tutorials) to see how to do it.    

## Usage
First of all it is required to source your workspace setup.bash file. To do it open a terminal, move into the workspace directory and the launch the command:
```
source devel/setup.bash
```

If wou want run the whole project just type:
```
$ roslaunch assignment_1 assignment_1.launch
```
The launch file is done in a way that it makes possible to see the state of the **FSM** in the *smach_viewer* node automatically.<br>
In the terminal in which the roslaunch command is executed it is possible to see the current positions of the robot.<br><br>
In order to manually execute the nodes/services to see in each terminal the data generated from each node/service, it is required to *rosrun* each node/service listed in *assignment_1.launch* in the exact same order they are listed in.
## Software Limitation and Possible technical Improvements
### Limitations
- The system generates randomly the boundaries of the map and the position of the user and it's not implemented a routine to accept these information from a configuration file yet;
- It is not implemented a routine to check if the new randomly generated position is inside or outside of the boundaries of the map.
### Possible Improvements
- Recive informations, as the map or the position of the user, from a configuration file;
- Let the user change his/her position during the execution;
- Add a microphone sensor to the robot, so that the user can command the robot by speech;
- Implement a real navigation algorithm;
- Add more states as Eat or Drink.
## Contacts
Roberto Albanese ralbanese18@gmail.com
<br>
<br>
<br>

**Robotics Engineering Master - 2nd year**  <br>
**University of Genoa**
