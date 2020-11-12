/**
 * @file usr_cmd.cpp
 * @brief Provides stuff for my great program.
 * 
 * Detailed description.
 */

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "user_command");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("hw1_usr_cmd", 1);

    std_msgs::String msg;
    std::stringstream s;

  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "Play";
    ROS_INFO("%s", msg.data.c_str());
    pub.publish(msg);
    sleep(rand() % 60 + 15);
    }    

    ros::spin();
    return 0;
}