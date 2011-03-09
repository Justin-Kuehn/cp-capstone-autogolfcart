#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <climits>
#include <boost/thread.hpp>
#include "golfcart_encoder/golfcart_encoder_node.h"
#include "golfcart_encoder/GolfcartEncoder.h"

using namespace ros;
using std::string;

static const int ARRAY_SIZE = 20;

int main(int argc, char **argv) {
    int last_tick = 0, delta_tick = 0, rolling_index = 0;
    double speed[ARRAY_SIZE] = {0};
    double avg_speed = 0;
    EncoderParams eParams;

    ros::Time last_time;
    ros::Duration delta_time;

    SimpleSerial* encoder_port;

    ros::init(argc, argv, "golfcart_encoder");
    ros::NodeHandle nh;

    // Init Params
    if (!nh.hasParam("/golfcart_encoder/wheel_cirum")) {
        ROS_ERROR("No wheel circumference defined");
        return false;
    }
    if (!nh.hasParam("/golfcart_encoder/ticks_per_rev")) {
        ROS_ERROR("No ticks per revolution defined");
        return false;
    }
    if (!nh.hasParam("/golfcart_encoder/max_encoder_ticks")) {
        ROS_ERROR("No max encoder ticks defined");
        return false;
    }
    if (!nh.hasParam("/golfcart_encoder/encoder_port")) {
        ROS_ERROR("No encoder_port defined");
        return false;
    }
    if (!nh.hasParam("/golfcart_encoder/compass_port")) {
        ROS_ERROR("No compass_port defined");
        return false;
    }

    nh.getParam("/golfcart_encoder/wheel_cirum", eParams.wheel_circum);
    nh.getParam("/golfcart_encoder/ticks_per_rev", eParams.ticks_per_rev);
    nh.getParam("/golfcart_encoder/max_encoder_ticks", eParams.max_encoder_ticks);
    nh.getParam("/golfcart_encoder/compass_port", eParams.compass_port);
    nh.getParam("/golfcart_encoder/encoder_port", eParams.encoder_port);
    ROS_INFO("Using: wheel_cirum: %f, ticks_per_rev: %d,  max_encoder_ticks: %d",
            eParams.wheel_circum, eParams.ticks_per_rev, eParams.max_encoder_ticks);
    ROS_INFO("Using: encoder_port: %s, compass_port: %s", 
            eParams.encoder_port.c_str(), eParams.compass_port.c_str());

    // Open Encoder Serial Port
    try {
        encoder_port = new SimpleSerial(eParams.encoder_port, BAUD);
    } catch (...) {
        ROS_ERROR("golfcart_encoder: Error trying to open serial port %s\n", 
                eParams.encoder_port.c_str());
        return -1;
    }


    CompassReader compass(eParams.compass_port);
    // Set Up Compass
    boost::thread compassThread(compass);  

    ros::Publisher pb = 
        nh.advertise<golfcart_encoder::GolfcartEncoder>("encoder", 1);

    ros::Rate loop_rate(100); // 100Hz

    last_tick = atoi(encoder_port->readLine().c_str());
    last_time = ros::Time::now();

    while(ros::ok())
    {
        golfcart_encoder::GolfcartEncoder msg;

        // Read from encoders
        msg.ticks = atoi(encoder_port->readLine().c_str());
        msg.heading = compass.getData();
        msg.header.stamp = ros::Time::now();

        // Calculate Speed
        delta_time = msg.header.stamp - last_time;
        delta_tick = msg.ticks - last_tick;
        if (delta_tick < 0) delta_tick += eParams.max_encoder_ticks;
        speed[rolling_index] = 
            (eParams.wheel_circum / eParams.ticks_per_rev) * (delta_tick / delta_time.toSec());
        rolling_index = (rolling_index + 1) % ARRAY_SIZE;

        last_time = msg.header.stamp;
        last_tick = msg.ticks;

        // Calculate Average Speed
        avg_speed = 0;
        for(int i = 0; i < ARRAY_SIZE; i++) 
        {
            avg_speed += speed[i];
        }
        avg_speed /= ARRAY_SIZE;

        msg.speed = avg_speed;
        pb.publish(msg);

        //ros::spinOnce();
        //loop_rate.sleep();
    }

    compass.die();
    compassThread.join();

    return 0;
}

/**
 * Since the compass sends data at 40Hz compared to the ardiuno's 10Hz,
 * a seperate thread is necessary to aquire compass data in order to prevent
 * a build up of latency.
 */
void CompassReader::operator()() {
    while(!_dead) {
        _compassData = _compassPort->readLine();
    }
}

double CompassReader::getData() {
    if(_compassData.length() < 7)
        return 0;  
    return (double)atoi(_compassData.substr(3,3).c_str());
}
