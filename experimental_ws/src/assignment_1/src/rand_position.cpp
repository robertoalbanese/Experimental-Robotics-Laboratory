/**
 * @file rand_position.cpp
 * @brief Provides stuff for my great program.
 * 
 * Detailed description.
 */

#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "assignment_1/get_pos.h"

/*!
* \brief Callback function for the service random_position
*
* Whenever the service recives a request of the type assignment_1::get_pos::Request, the function will be executed
* \param req stores the request parameter for the service random_position
* \param res stores the response parameter for the service random_position
* @see get_pos.srv
*/

bool randPos(assignment_1::get_pos::Request &req,
             assignment_1::get_pos::Response &res)
{
    ///X parameter of the variable res
    res.x = rand() % (req.maxx - req.minx) + req.minx;
    ///Y parameter of the variable res
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