#include "avt_camera_streaming/avt_virtual.hpp"
#include <ros/ros.h>
#include<iostream>  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "avt_virtual_camera");

  ROS_INFO("Hello Trump");

  std::string sub_topic = "/trigger";
  std::string pub_topic = "/avt_camera_img";

  ros::NodeHandle nh;

  AvtCameraVirtual avtCameraVirtual(sub_topic, pub_topic, nh);

  ros::spin();
}