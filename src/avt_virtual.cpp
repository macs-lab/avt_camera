// this node acts as a virtual camera node that publish some images to the camera topic.

#include "avt_camera_streaming/avt_virtual.hpp"
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>

AvtCameraVirtual::AvtCameraVirtual(
    std::string sub_topic,
    std::string pub_topic,
    ros::NodeHandle nodeHandle)
  : mSubTopic(sub_topic)
  , mPubTopic(pub_topic)
  , mNodeHandle(nodeHandle)
{
  // is there a better way to initilize mImageTransport?
  image_transport::ImageTransport mImageTransport(nodeHandle);
  loadVirtualImage();
  mTriggerSub = mNodeHandle.subscribe(mSubTopic, 1, &AvtCameraVirtual::triggerCb, this);
  mImagePub = mImageTransport.advertise(mPubTopic, 1);
}

void AvtCameraVirtual::loadVirtualImage()
{
  try
  {
    std::string package_path = ros::package::getPath("rubix_cube_robot_solver");
    std::string img_path =  package_path + "/test_image" ;
    mImageBuffer["L"] = cv::imread(img_path + "/L.png" , CV_LOAD_IMAGE_COLOR);
    cv::waitKey(1);
    // std::cout << img_path <<std::endl;
    ROS_INFO("Load Successfully");
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not read image L.png.");
  }
}

void AvtCameraVirtual::triggerCb(
    const std_msgs::String::ConstPtr& msg)
{
  // if msg->data.c_str() == "L"
    // std::
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  mImageOut = mImageBuffer["L"];
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mImageOut).toImageMsg();
  mImagePub.publish(img_msg);
}


