/*============================================================
    Created by Hui Xiao - University of Connecticuit - 2018
    hui.xiao@uconn.edu
==============================================================*/

#include <iostream>
#include "ros/ros.h"
#include "ros/console.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>

class MessagePublisher
{
public:
    // use member initializer list to initialize ImageTransport
    // Initializing when it is decleared will produce a compile error.
    MessagePublisher() : it(nh)
    {
        img_pub = it.advertise("avt_camera_img",10);
    }

    // convert OpenCV image to ROS message
    // publish message to ROS topic
    void PublishImage(cv::Mat &image);

private:
    // ros::init() is called in main.cpp
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Publisher img_pub;
    sensor_msgs::ImagePtr msg;
};