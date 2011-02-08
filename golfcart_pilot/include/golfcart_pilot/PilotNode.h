#ifndef _PILOT_NODE
#define _PILOT_NODE

#include <stdint.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_ax1500/channel_forward.h"

struct SteeringParams {
    int chan;
    int encMin;
    int encCen;
    int encMax;
    double radPerTic;
    double ticPerRad;
    double minRad;
    double maxRad;
    double range;
};

class PilotNode {
    ros::NodeHandle n;
    ros::NodeHandle pn;
    ros::ServiceClient ax1500;
    ros::Subscriber absCmdSub;
    ros::Subscriber relCmdSub;
    geometry_msgs::Twist state;
    SteeringParams sParams;


    public:
    PilotNode() : pn("~") {};
    bool init();
    bool connect();
    void absoluteCommandCallback(const geometry_msgs::Twist::ConstPtr &m);
    void relativeCommandCallback(const geometry_msgs::Twist::ConstPtr &m);
};

#endif
