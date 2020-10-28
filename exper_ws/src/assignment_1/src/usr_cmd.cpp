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
    msg.data = "Play";

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        sleep(rand() % 10 + 1);
        pub.publish(msg);

        loop_rate.sleep();
    }

    

    ros::spin();
    return 0;
}