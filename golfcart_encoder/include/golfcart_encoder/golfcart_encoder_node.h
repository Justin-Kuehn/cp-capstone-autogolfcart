#ifndef GOLFCART_ENCODER_NODE_H
#define GOLFCART_ENCODER_NODE_H

#include <string>
#include "ros/ros.h"
#include "golfcart_encoder/simple_serial.h"

#define BAUD 9600

struct EncoderParams {
    double wheel_circum;  // Meters
    int ticks_per_rev; 
    int max_encoder_ticks; 
    std::string encoder_port;
    std::string compass_port;
};


class CompassReader {
    private:
        bool _dead;
        SimpleSerial *_compassPort;
        std::string _compassData;
    public:
        CompassReader(std::string compassPort) : _dead(false) {
            try {
                _compassPort = new SimpleSerial(compassPort, BAUD);
            } catch (...) {
                ROS_ERROR("golfcart_encoder: Error trying to open serial port %s\n", 
                        compassPort.c_str());
            }
        }
        virtual ~CompassReader() {
            delete _compassPort;
        }

        void die() { _dead = true; }
        void operator()();
        double getData();
};

#endif
