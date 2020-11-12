/**
 * @file navigation.cpp
 * @brief In this file the service "reach_new_position" is initialized.
 * 
 * The server recives a requested position and brings the robot to there
 */

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "assignment_1/reach_next_pos.h"

/*!
* \brief Callback function for the service "reach_new_position"
*
* Whenever the service recives a request of the type assignment_1::reach_next_pos::Request, the function will be executed
* \param req stores the request parameter for the service "reach_new_position" (req.x, req.y)
* \param res stores the response parameter for the service "reach_new_position" (res.x, res.y)
*/

bool reachPos(assignment_1::reach_next_pos::Request &req,
              assignment_1::reach_next_pos::Response &res)
{
    res.x = req.x;
    res.y = req.y;
    ROS_INFO("sending back response: x = [%d] , y = [%d]", int(res.x), int(res.x));
    return true;
}

/*! \brief Initialization of the service "reach_new_position" */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("reach_new_position",reachPos);

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}