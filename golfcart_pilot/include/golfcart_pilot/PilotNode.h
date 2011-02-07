#ifndef _PILOT_NODE
#define _PILOT_NODE

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_ax1500/channel_forward.h"

class PilotNode {
    ros::NodeHandle n;
    ros::NodeHandle pn;
    ros::ServiceClient ax1500;
    geometry_msgs::Twist state;

    public:
    PilotNode() : pn("~") {};
    bool init();
};

#endif
