#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include "ros/ros.h"

using namespace ros;
using std::string;

static const double WHEEL_DIAMETER = 1.0;
static const int TICKS_PER_REV = 300;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "segway_rmp");
  ros::NodeHandle nh;
  
  // Set Up Pub/Sub here
  
  while(ros::ok())
  {
    //DO STUFF HERE
  
  }
  
  return 0;
}
