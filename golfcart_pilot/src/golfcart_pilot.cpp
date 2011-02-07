#include <iostream>
#include <ros/ros.h>
#include "golfcart_pilot/PilotNode.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "golfcart_pilot");
    PilotNode n;

    if (!n.init()) {
        ROS_WARN("couldn't initialize");
        return 1;
    }

    ROS_INFO("golfcart_pilot ready");
    ros::spin();

    return 0;
}
