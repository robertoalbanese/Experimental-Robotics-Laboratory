/**
 * @file navigation.cpp
 * @brief Provides stuff for my great program.
 * 
 * Detailed description.
 */

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "assignment_1/reach_next_pos.h"

bool reach_Pos(assignment_1::reach_next_pos::Request &req,
              assignment_1::reach_next_pos::Response &res)
{
    res.x = req.x;
    res.y = req.y;
    ROS_INFO("sending back response: x = [%d] , y = [%d]", int(res.x), int(res.x));
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("reach_new_position",reach_Pos);

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}