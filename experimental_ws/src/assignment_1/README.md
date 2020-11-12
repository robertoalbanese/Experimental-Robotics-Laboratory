# Behavioral Architecture - Assignment 1

   - __Author:__ Roberto Albanese
   - __Version:__ 1.0
   - __Date:__ 10-28-2020

## Indrodution
This is the first assignment of the course *Experimental Robotics*. I am asked to build an ROS architecture to implement a dog alike robot’s behavior.
In this project folder the reader will encounter the following folders:
- [doc](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/doc/html) Doxygen documentation
- [launch](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/launch) Launch files
- [src](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/src) Source files
- [srv](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/srv) Service files 

## Software Architecture and System's Features
The scenario is represented by a robot, simulating a pet, that interacts with a human and moves in a discrete 2D environment. <br>
The robot has three possible behaviors: it can Sleep, Play or stay in a Normal state.<br><br>
In the Normal state the robot has to move randomly.<br><br>
In the Sleep state the robot chooses a random location and sleep there for a random amount of time. Then it goes back in the Normal state. The robot can pass to the Spleep state only from the Normal state in a random time instant.<br><br>
In the Play state the robot reaches the User and waits until it points to a random position. The robot has to reach the position and then come back to the user. The robot can reach the Play state only from the Normal state and only when a "Play" command is recived.<br><br>
The software architecture is composed by four elements:
IMAGE
* __User Command__: it represents the user command "play"
* __Command Manager__: it is the main part of the architecture in which the **FSM** is.
* __Navigation__: it manages the motion of the robot and brings it to the new position
* __Random Position Generator__: generates random positions

Navigation and Random Position Generator are services because they are thought to operate in a syncronous mode. They will generate a new position and move the robot to the generated position only when it is specifically requested from the code. On the other hand, User Command is meant to be a node because it has to behave in an asyncronous mode (it must send a command randomly).
### File list
In the  [source folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/src) it is possible to find four files which compose the whole architecture:

* __usr_cmd.cpp__: it is the only node leaving out the state machine. In here I have defined a publisher which randomly publishes a string "Play" in the topic *hw1_usr_cmd*.
* __rand_position.cpp__: it is the fist service of the architecture. It waits for a request message of the type *get_pos.srv* and randomly generates a 2D position
* __navigation.cpp__: it is the second service of the architecture. It waits for a request message of the type *reach_next_pos.srv* in which it is present the new position where the robot needs to go, waits for a reasonable amount of time to simulate a motion, and gives as response the new current position.   
* __state_machine.py__ is the core node that manages information from the two services and the publisher. It initializes and executes a state machine, using the library *smach_ros*, in which all the three states and their behaviours are defined.<br>

In the [service folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/srv) is it possible to find the two files used by the services:
* __get_pos.serv__: the request part of the message represents the boundaries of the map. The service cannot generate a random position outside of the map. The respose part represents the new generated random position.
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
In the [launch folder](https://github.com/robertoalbanese/Experimental-Robotics-Laboratory/tree/master/experimental_ws/src/assignment_1/launch) it is present the launch file used to execute all the nodes.
## Finite State Machine 
The finite state machine, present in the _commandManager_ node, was developed as follows: 
* __NORMAL__ : First of all _NORMAL_ is the initial state. In this state the _FSM_ checks if the _state_ variable (which contains the strings published from the _speakPerception_ node) is equal to the string _play_. In that case the _FSM_ goes in _PLAY_ state. Otherwise it makes a request to the _Navigation_ service to go in a random position provided by the _getPosition_ node. 
Then this routine is iterated for a random number of times (max 3 times) after which the robot goes in _SLEEP_ mode. 
* __PLAY__ : In this state the _commandManager_ moves the robot to the goal position which is just the last position sent by the _getPosition_ node. Then it call again the _Navigation_ service sending as goal position the user position, in order to tell the robot to come back to the user. After then that the _FSM_ come back in _NORMAL_ mode.
* __SLEEP__ : The robot goes to the home which is a known position a priori chosen. Then it sleeps for a random time after which the _FSM_ goes in NORML mode. 

## Installation
In order to use this package it's necessary to install the _smach_ library which allows to implement a _Finite State Machine_. So on a command window digit:
```

$ sudo apt-get install ros-kinetic-smach-viewer
```
Of coure it's also required to make a _catkin_make_ in your _ROS_ workspace
### File list
In this package you can find in the _src_ directory the three nodes file and the service. In the _launch_ directory has been defined several launch files to test the system (_assignment1.launch_ launches every components of the system). _srv_ directory contains the definition of the only service of the architecture. Finally you can find three _bash command file_ usefull to test the system avoiding to digit long terminals commands :
- __run_sys.sh__ allows to launch all componets of the system without the _getPosition_node
- __run_gesPosition.sh__ launches the _getPosition_ node
- __impossiblePosition.sh__  just publishes on the _Position_ topic a position messagge which is out of the map. This is usefull to test the response of the system in case the user points a position which is not reachable from the robot.     

## Usage
First of all remeber to source your wrokspace in every terminal you will use.
If wou want run the whole project just type:
```

$ roslaunch assignment1 assignment1.launch
```
### Test getPosition 
In order to test in two different terminals the messages random generates by the _getPosition_ node and see the their effects, plese go to the _assignment1_ directory and execute from shell:
```

$ cd ~/assignment1
$ ./run_getPosition.sh
```
and then in another terminal execute
```

$ cd ~/assignment1
$ ./run_sys.sh
```
You will see in first terminal window the positions randomly generated by _getPostion_ and the response in real time of the _commandManager_ node.
### Test impossible position
```

$ cd ~/assignment1
$ ./run_sys.sh
```
and in a new terminal execute
```

$ cd ~/assignment1
$ ./impossiblePosition.sh
```
<br>
<br>
<br>

**Robotics Engineering Master - 2nd year**  <br>
**University of Genoa**