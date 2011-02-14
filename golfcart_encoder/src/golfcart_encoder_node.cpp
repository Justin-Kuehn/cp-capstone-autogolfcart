#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <limits.h>
#include "ros/ros.h"
#include "golfcart_encoder/GolfcartEncoder.h"
#include "simple_serial.h"

using namespace ros;
using std::string;

static const double WHEEL_CIRCUM = 1.1; // Meters
static const double TICKS_PER_REV = 160.0;
static const int ARRAY_SIZE = 20;
static const int MAX_ENCODER_TICKS = 32000;

int main(int argc, char **argv)
{
  int last_tick = 0, delta_tick = 0, rolling_index = 0;
  double speed[ARRAY_SIZE] = {0};
  double avg_speed = 0;
  ros::Time last_time;
  ros::Duration delta_time;
  
  SimpleSerial* port;
  
  
  ros::init(argc, argv, "golfcart_encoder");
  ros::NodeHandle nh;
  
  try {
    port = new SimpleSerial("/dev/ttyUSB0", 9600);
  } catch (...) {
    fprintf(stderr, "golfcart_encoder: Error trying to open serial port\n");
    return -1;
  }
  
  ros::Publisher pb = 
    nh.advertise<golfcart_encoder::GolfcartEncoder>("encoder", 1);
  
  ros::Rate loop_rate(100); // 100Hz
  
  last_tick = atoi(port->readLine().c_str());
  last_time = ros::Time::now();
  
  while(ros::ok())
  {
    golfcart_encoder::GolfcartEncoder msg;
    
    // Read from encoders
    msg.ticks = atoi(port->readLine().c_str());
    msg.header.stamp = ros::Time::now();
    msg.heading = 0; //TODO
    
    // Calculate Speed
    delta_time = msg.header.stamp - last_time;
    delta_tick = msg.ticks - last_tick;
    if (delta_tick < 0) delta_tick += MAX_ENCODER_TICKS;
    speed[rolling_index] = 
      (WHEEL_CIRCUM / TICKS_PER_REV) * (delta_tick / delta_time.toSec());
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
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
