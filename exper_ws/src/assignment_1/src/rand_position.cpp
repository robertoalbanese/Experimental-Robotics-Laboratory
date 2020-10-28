#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "assignment_1/get_pos.h"

bool randPos(assignment_1::get_pos::Request &req,
             assignment_1::get_pos::Response &res)
{
    res.x = rand() % (req.maxx - req.minx) + req.minx;
    res.y = rand() % (req.maxy - req.miny) + req.miny;
    ROS_INFO("sending back response: x = [%d] , y = [%d]", int(res.x), int(res.x));
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "random_position_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("random_position",randPos);

    ros::spin();

    return 0;
}