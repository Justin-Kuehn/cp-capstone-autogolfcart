#ifndef _PILOT_NODE
#define _PILOT_NODE

#include <stdint.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "roboteq_ax1500/channel_forward.h"
#include "pmad/Switch.h"

struct SteeringParams {
    int chan;           // AX1500 channel
    int encMin;         // Linear encoder value when full left
    int encCen;         // Linear encoder value when centered
    int encMax;         // Linear encoder value when full right
    double radPerTic;   // Radians / Encoder Tic
    double ticPerRad;   // Tics / Radian
    double minRad;      // Radian delta from center when steering left
    double maxRad;      // Radian delta from center when steering right
    double range;       // Steering range in radians
};

struct SpeedParams {
    int min;                // Minimum valid speed value for arduino
    int max;                // Maximum valid speed value for arduino
    double ticPerPercent;   // Tics / Percent
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
