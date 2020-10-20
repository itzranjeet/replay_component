#ifndef REPLAY_COMPONENT_INCLUDE_NODE5_HPP
#define REPLAY_COMPONENT_INCLUDE_NODE5_HPP

#include "replay_component/constants.hpp"

#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "std_msgs/String.h"

#include <iostream>

class ControlNode
{
public:
    void ServerFunction();
    void RegisterPublishers();
    void Run();

private:
    ros::NodeHandle nh;
    std_msgs::String msg;
    ros::Publisher trigger_pub;
};

#endif // REPLAY_COMPONENT_INCLUDE_NODE5_HPP