#include <boost/asio/serial_port.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/scoped_ptr.hpp>
#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include "arduino_due/set_speed.h"

using namespace std;
using namespace boost;

typedef scoped_ptr<asio::io_service> io_service_ptr;
typedef scoped_ptr<asio::serial_port> serial_port_ptr;

static io_service_ptr io_ptr;
static serial_port_ptr sp_ptr;

bool set_speed(arduino_due::set_speed::Request &req,
        arduino_due::set_speed::Response &res) {



    return false;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "arduino_cmd_server");
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

    ROS_INFO("ARDUINO_READY");
    ros::ServiceServer setSpeed = n.advertiseService("set_speed", set_speed);
    ros::spin();
    
    return 0;
}
