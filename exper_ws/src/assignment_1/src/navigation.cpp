#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"

ros::Publisher pub;
ros::Subscriber sub;

void move(const geometry_msgs::Pose2D &msg)
{
  ROS_INFO_STREAM(msg);
  sleep(rand() % 10 + 1);
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navigation");
  ros::NodeHandle n;

  pub = n.advertise<geometry_msgs::Pose2D>("hw1_position", 1000);

  sub = n.subscribe("hw1_position", 0, move);

  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}