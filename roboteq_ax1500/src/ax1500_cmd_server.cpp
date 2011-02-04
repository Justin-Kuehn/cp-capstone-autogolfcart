#include <boost/asio/serial_port.hpp>
#include <boost/scoped_ptr.hpp>
#include "ros/ros.h"
#include "roboteq_ax1500/channel_forward.h"

using namespace std;
using namespace boost;
using namespace boost::asio;

typedef scoped_ptr<io_service> io_service_ptr;
typedef scoped_ptr<serial_port> serial_port_ptr;

static const string RESET("%rrrrrr");
static const string CH1_FORWARD("!A");
static const string CH1_REVERSE("!a");
static const string CH2_FORWARD("!B");
static const string CH2_REVERSE("!b");
static const string ACCESSORY_C_ON("!C");
static const string ACCESSORY_C_OFF("!c");
static const string READ_BATTERY_AMPS("?A");
static const string READ_POWER_TO_MOTORS("?V");
static const string READ_ANALOG_1_2("?P");
static const string READ_ANALOG_3_4("?R");
static const string READ_HEATSINK_TEMP("?M");
static const string READ_BATTERY_INTERNAL_VOLTAGE("?E");
static const string READ_DIGITAL_INPUTS("?I");
static const string READ_ENCODER("?K");

static io_service_ptr io_ptr;
static serial_port_ptr sp_ptr;

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

    if (argc < 2) {
        return 1;
    }

    io_ptr.reset(new io_service());
    ROS_INFO("Using port %s", argv[1]);
    sp_ptr.reset(new serial_port(*io_ptr, argv[1]));
    sp_ptr->set_option(serial_port_base::baud_rate(9600));
    sp_ptr->set_option(serial_port_base::parity(serial_port_base::parity::even));
    sp_ptr->set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
    sp_ptr->set_option(serial_port_base::character_size(7));
    ROS_INFO("AX1500 Ready");

    n.advertiseService("channel_forward", channel_forward);
    // TODO: spin with watchdog
    ros::spin();

    return 0;
}
