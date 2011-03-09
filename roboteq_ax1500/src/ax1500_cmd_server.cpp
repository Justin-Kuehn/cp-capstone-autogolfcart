#include <boost/asio/serial_port.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/scoped_ptr.hpp>
#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "roboteq_ax1500/channel_forward.h"

using namespace std;
using namespace boost;

typedef scoped_ptr<asio::io_service> io_service_ptr;
typedef scoped_ptr<asio::serial_port> serial_port_ptr;

// Commands - See AX1500 reference manual
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
static const char KEEP_ALIVE = 'Z';

static io_service_ptr io_ptr;
static serial_port_ptr sp_ptr;

// Channel Forward handler
bool channel_forward(roboteq_ax1500::channel_forward::Request &req,
        roboteq_ax1500::channel_forward::Response &res) {
    ROS_INFO("channel_forward: channel=%d, value=%x", req.channel, req.value);
    if (!req.channel || req.channel > 2) {
        ROS_ERROR("invalid channel");
        return false;
    }

    char send[6], ret[7];
    const char *cmd = (req.channel == 1) ? CH1_FORWARD.c_str() : CH2_FORWARD.c_str();
    sprintf(send, "%s%02x\r", cmd, req.value);
    ROS_INFO("channel_forward: sending command %s", send);

    asio::write(*sp_ptr, boost::asio::buffer(send, 6));
    asio::read(*sp_ptr, boost::asio::buffer(ret, 7));

    bool r = true;
    char *sptr = send, *rptr = ret;
    // Verify what we sent is echoed back
    for ( ; *sptr; sptr++, rptr++) {
        r &= (*sptr == *rptr);
    }
    // Successful command ends in "+\r"
    if (*rptr != '+') {
        ROS_ERROR("channel_foward: expected %d, got %d", '+', *rptr);
        r = false;
    }
    rptr++;
    if (*rptr != '\r') {
        ROS_ERROR("channel_foward: expected %d, got %d", '+', *rptr);
    }
    ROS_INFO("channel_foward: %s", r ? "success" : "failure");

    return r;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ax1500_cmd_server");
    ros::NodeHandle n;

    if (!ros::param::has("~tty")) {
        cerr << "Usage: " << argv[0] << " _tty:=<tty>" << endl;
        return 1;
    }

    string tty;
    ros::param::get("~tty", tty);

    io_ptr.reset(new asio::io_service());
    ROS_INFO("Using port %s", tty.c_str());
    sp_ptr.reset(new asio::serial_port(*io_ptr, tty.c_str()));
    sp_ptr->set_option(asio::serial_port_base::baud_rate(9600));
    sp_ptr->set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::even));
    sp_ptr->set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));
    sp_ptr->set_option(asio::serial_port_base::character_size(7));
    ROS_INFO("AX1500 Ready");

    ros::ServiceServer channelForward = n.advertiseService("channel_forward", channel_forward);

    /*
     * If the watchdog timer is active, the AX1500 expects input at least once every second otherwise
     * it kills the motors and outputs 'W' characters until connection is restored. So, every 0.25 seconds
     * send KEEP_ALIVE to the AX1500 and check that AX1500 is echoed back.
     */
    while (ros::ok()) {
        char ret;
        asio::write(*sp_ptr, boost::asio::buffer(&KEEP_ALIVE, 1));
        asio::read(*sp_ptr, boost::asio::buffer(&ret, 1));
        if (ret != KEEP_ALIVE)
            ROS_WARN("Keep Alive Failure: expected %d, got %d", KEEP_ALIVE, ret);
        // Execute from the callback queue (service requests, message handling, etc) for 0.25 seconds
        ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.25));
    }

    return 0;
}
