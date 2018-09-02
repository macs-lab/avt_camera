/*============================================================
    Created by Hui Xiao - University of Connecticuit - 2018
    hui.xiao@uconn.edu
==============================================================*/

#include "avt_camera_streaming/MessagePublisher.h"

MessagePublisher::MessagePublisher()
{
    std::cout << "Starting Message Publisher" << std::endl;
}

void MessagePublisher::PublishImage(cv::Mat &image)
{
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    img_pub.publish(msg);
}