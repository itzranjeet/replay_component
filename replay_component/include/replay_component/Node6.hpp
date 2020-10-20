#ifndef REPLAY_COMPONENT_INCLUDE_NODE6_HPP
#define REPLAY_COMPONENT_INCLUDE_NODE6_HPP

#include "replay_component/constants.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <iostream>

using namespace std;

class StatusNode
{
public:
  std::string buffer;
  void statusCallback(const std_msgs::String::ConstPtr &msg);
  void RegisterSubscribers();
  void Run();
  void idleStatus_cmd();
  void processingStatus_cmd();
  void dataInterrupted_cmd();
  void dataOver_cmd();

private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
};
#endif // REPLAY_COMPONENT_INCLUDE_NODE6_HPP