/*============================================================
    Created by Hui Xiao - University of Connecticuit - 2018
    hui.xiao@uconn.edu
==============================================================*/

#include "avt_camera_streaming/MessagePublisher.h"


void MessagePublisher::PublishImage(cv::Mat &image)
{
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    msg->header.stamp = ros::Time::now();
    img_pub.publish(msg);
}