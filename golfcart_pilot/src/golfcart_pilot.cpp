#include <iostream>
#include <ros/ros.h>
#include "golfcart_common/golfcart.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "golfcart_pilot");
    ros::NodeHandle n;

    if (argc != 4) {
        cerr << "Usage: " << argv[0] << " <min> <center> <max>" << endl;
        return 1;
    }

    ROS_INFO("golfcart_pilot ready");
    ros::spin();

    return 0;
}
