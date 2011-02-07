#include "golfcart_pilot/PilotNode.h"

void PilotNode::commandCallback(const geometry_msgs::Twist::ConstPtr &m) {
}

bool PilotNode::init() {
    ros::Rate r(1);
    do {
        ROS_INFO("Connecting to ax1500");
        ax1500 = n.serviceClient<roboteq_ax1500::channel_forward>("channel_forward", true);
        r.sleep();
    } while (!ax1500 && ros::ok());

    if (ax1500) {
        ROS_INFO("Connected to ax1500");
    } else {
        ROS_ERROR("Couldn't connect to ax1500");
        return false;
    }

    if (!n.hasParam("/golfcart_pilot/steering_channel")) {
        ROS_ERROR("No steering channel");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_min")) {
        ROS_ERROR("No steering min");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_center")) {
        ROS_ERROR("No steering center");
        return false;
    }
    if (!n.hasParam("/golfcart_pilot/steering_max")) {
        ROS_ERROR("No steering max");
        return false;
    }

    commandSub = n.subscribe<geometry_msgs::Twist>("golfcart_pilot/command", 100,
            &PilotNode::commandCallback, this);

    return true;
}
