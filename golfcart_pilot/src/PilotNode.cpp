#include "golfcart_pilot/PilotNode.h"

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

    return true;
}
