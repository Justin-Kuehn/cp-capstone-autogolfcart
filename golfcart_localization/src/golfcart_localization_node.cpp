#include "ros/ros.h"
#include "golfcart_localization/particle_filter.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "golfcart_encoder");
  ros::NodeHandle nh;
  
  ros::Rate loop_rate(10); // 10Hz
  
  while(ros::ok())
  {
    //DO STUFF HERE
  
    ros::spinOnce();
    loop_rate.sleep();
  }
  
  return 0;
}
