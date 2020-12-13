/**
 * @file rand_position.cpp
 * @brief In this file the service "new_rand_pos_srv" is initialized.
 * 
 * The server recives a request and it generates a new random position. 
 */

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "exp_assignment2/GetNewPos.h"

/*!
* \brief Callback function for the service new_rand_pos_srv
*
* Whenever the service recives a request of the type assignment_1::GetNewPos::Request, the function will be executed
* \param req stores the request parameter for the service new_rand_pos_srv (req.x, req.y)
* \param res stores the response parameter for the service new_rand_pos_srv (res.x, res.y)
*/

bool randPos(exp_assignment2::GetNewPos::Request &req,
             exp_assignment2::GetNewPos::Response &res)
{
    res.x = rand() % (req.maxx - req.minx) + req.minx;
    res.y = rand() % (req.maxy - req.miny) + req.miny;
    ROS_INFO("sending back response: x = [%d] , y = [%d]", int(res.x), int(res.x));
    return true;
}

/*! \brief Initialization of the service "new_rand_pos_srv" */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "new_rand_pos_srv");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("new_rand_pos_srv",randPos);

    ros::spin();

    return 0;
}
