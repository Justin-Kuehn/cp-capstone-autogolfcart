#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <golfcart_encoder/GolfcartEncoder.h>
#include <gps_common/GPSFix.h>
#include "golfcart_localization/particle_filter.h"


bool encoder_init;
bool gps_init;
golfcart_encoder::GolfcartEncoder encoder_data;
gps_common::GPSFix gps_data;

void encoderCallback(const golfcart_encoder::GolfcartEncoder::ConstPtr& msg) {
  encoder_init = true;
  encoder_data = *msg;
}

void gpsCallback(const gps_common::GPSFix::ConstPtr& msg) {
  gps_init = true;
  gps_data = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "golfcart_localization");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10); // 10Hz
  
  ros::Publisher pb = 
    nh.advertise<geometry_msgs::Pose2D>("pose", 1);
  
  ros::Subscriber sub_encoder = nh.subscribe("encoder", 1, encoderCallback);
  ros::Subscriber sub_gps = nh.subscribe("fix", 1, gpsCallback);
  
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
  
  OdomParam odomModel;
  nh.getParam("/golfcart_encoder/wheel_cirum", odomModel.wheelCircum);
  nh.getParam("/golfcart_encoder/ticks_per_rev", odomModel.ticksPerRev);
  nh.getParam("/golfcart_encoder/max_encoder_ticks", odomModel.maxTicks);
  
  ROS_INFO("Waiting for publishers... ");
  gps_init = false;
  encoder_init = false;
  
  while(ros::ok()) {
    if( encoder_init && gps_init ) break;
    //ROS_INFO(" encoder: %d, gps: %d", sub_encoder.getNumPublishers(), sub_gps.getNumPublishers());
    ros::spinOnce();
  }
  
  ROS_INFO("Encoder and GPS publishers aquired. Setting up filter...");
  
  geometry_msgs::Pose2D pose;
  ParticleFilter filter;

  filter.Init(
      gps_data.latitude, 
      gps_data.longitude, 
      encoder_data.heading, 
      encoder_data.ticks, 
      odomModel
  );
  
  ROS_INFO("Done! Running filter.");
  
  while(ros::ok()) {

    filter.RunIteration(
        encoder_data.ticks, 
        gps_data.latitude, 
        gps_data.longitude, 
        encoder_data.heading, 
        0.1
    );
    
    filter.getPose(pose.x, pose.y, pose.theta);
    
    pb.publish(pose);

    loop_rate.sleep();
    ros::spinOnce();
    ros::spinOnce();
  }
  
  return 0;
}
