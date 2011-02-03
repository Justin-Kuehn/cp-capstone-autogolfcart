#include "ros/ros.h"
#include "roboteq_ax1500/channel_forward.h"

static const std::string RESET("%rrrrrr");
static const std::string CH1_FORWARD("!A");
static const std::string CH1_REVERSE("!a");
static const std::string CH2_FORWARD("!B");
static const std::string CH2_REVERSE("!b");
static const std::string ACCESSORY_C_ON("!C");
static const std::string ACCESSORY_C_OFF("!c");
static const std::string READ_BATTERY_AMPS("?A");
static const std::string READ_POWER_TO_MOTORS("?V");
static const std::string READ_ANALOG_1_2("?P");
static const std::string READ_ANALOG_3_4("?R");
static const std::string READ_HEATSINK_TEMP("?M");
static const std::string READ_BATTERY_INTERNAL_VOLTAGE("?E");
static const std::string READ_DIGITAL_INPUTS("?I");
static const std::string READ_ENCODER("?K");

bool channel_forward(roboteq_ax1500::channel_forward::Request &req,
        roboteq_ax1500::channel_forward::Response &res) {
    ROS_INFO("request: channel=%d, value=%x", req.channel, req.value);

    // TODO: send command and parse result

    bool r = true;
    ROS_INFO("success: %d", r);
    return r;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ax1500_cmd_server");
    ros::NodeHandle n;

    n.advertiseService("channel_forward", channel_forward);
    // TODO: initialize serial
    ROS_INFO("AX1500 Ready");
    // TODO: spin with watchdog
    ros::spin();

    return 0;
}
