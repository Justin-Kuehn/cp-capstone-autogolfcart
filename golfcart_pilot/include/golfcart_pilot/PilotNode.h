#ifndef _PILOT_NODE
#define _PILOT_NODE

#include <stdint.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_ax1500/channel_forward.h"
#include "pmad/Switch.h"

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

struct SpeedParams {
    int min;
    int max;
    double ticPerPercent;
};

class PilotNode {
    ros::NodeHandle n;
    ros::NodeHandle pn;
    ros::ServiceClient ax1500;
    ros::ServiceClient arduino;
    ros::Subscriber absCmdSub;
    ros::Subscriber relCmdSub;
    geometry_msgs::Twist state;
    SteeringParams stParams;
    SpeedParams spParams;

    public:
    PilotNode() : pn("~") {};
    bool init();
    bool connect();
    void absoluteCommandCallback(const geometry_msgs::Twist::ConstPtr &m);
    void relativeCommandCallback(const geometry_msgs::Twist::ConstPtr &m);
    bool sendSteeringCommand(uint8_t value);
    bool sendSpeedCommand(uint8_t value);
};

#endif
