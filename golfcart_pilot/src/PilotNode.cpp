#include "golfcart_pilot/PilotNode.h"

void PilotNode::absoluteCommandCallback(const geometry_msgs::Twist::ConstPtr &m) {
    if (!ax1500) {
        ROS_WARN("ax1500 connection lost");
        connect();
    }
}

void PilotNode::relativeCommandCallback(const geometry_msgs::Twist::ConstPtr &m) {
}

bool PilotNode::connect() {
    ros::Rate r(1);
    do {
        ROS_INFO("Connecting to ax1500");
        ax1500 = n.serviceClient<roboteq_ax1500::channel_forward>("channel_forward", true);
        r.sleep();
    } while (!ax1500 && ros::ok());

    if (ax1500) {
        ROS_INFO("Connected to ax1500");
        return true;
    } else {
        ROS_ERROR("Couldn't connect to ax1500");
        return false;
    }
}

bool PilotNode::init() {
    if (!connect())
        return false;

    if (!n.hasParam("/golfcart_pilot/steering_channel")) {
        ROS_ERROR("No steering channel");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_min_enc")) {
        ROS_ERROR("No min steering encoder value");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_center_enc")) {
        ROS_ERROR("No center steering encoder");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_max_enc")) {
        ROS_ERROR("No max steering encoder");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_range")) {
        ROS_ERROR("No steering range");
        return false;
    }

    double encMin, encCen, encMax, range;
    n.getParamCached("/golfcart_pilot/steering_max_enc", encMax);
    n.getParamCached("/golfcart_pilot/steering_center_enc", encCen);
    n.getParamCached("/golfcart_pilot/steering_min_enc", encMin);
    n.getParamCached("/golfcart_pilot/steering_range", range);
    ROS_INFO("Using: max %f tics, center %f tics, min %f tics, range %f radians", encMax, encCen, encMin, range);
    double radPerTic = range / (encMax - encMin);
    ROS_INFO("Radians Per Encoder Tic: %f", radPerTic);
    n.setParam("/golfcart_pilot/rad_per_tic", radPerTic);
    double minRad = (encMin - encCen) * radPerTic;
    double maxRad = (encMax - encCen) * radPerTic;
    ROS_INFO("Max steering: %f radians, Min steering: %f radians", maxRad, minRad);
    n.setParam("/golfcart_pilot/steering_max_rad", maxRad);
    n.setParam("/golfcart_pilot/steering_min_rad", minRad);

    absCmdSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/abs_cmd", 100,
            &PilotNode::absoluteCommandCallback, this);
    relCmdSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/rel_cmd", 100,
            &PilotNode::relativeCommandCallback, this);

    return true;
}
