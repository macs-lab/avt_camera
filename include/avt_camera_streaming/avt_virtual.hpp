#include <ros/ros.h>
#include "std_msgs/String.h"
#include <unordered_map> 

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>
#include <stdio.h>


class AvtCameraVirtual
{

public:
  /// Constructor for the Virtual AvtCamera.
  /// \param[in] topic,  
  AvtCameraVirtual(
      std::string sub_topic,
      std::string pub_topic,
      ros::NodeHandle nodeHandle);

  /// Destructor for the Virtual AvtCamera.
  virtual ~AvtCameraVirtual() = default;

private:

    std::string mSubTopic;
    std::string mPubTopic;
    ros::NodeHandle mNodeHandle;
    // image_transport::ImageTransport mImageTransport;
    std::unordered_map<std::string, cv::Mat> mImageBuffer;

    cv::Mat mImageOut;
    ros::Subscriber mTriggerSub;
    image_transport::Publisher mImagePub;

    // camera trigger call back function
    void triggerCb(const std_msgs::String::ConstPtr& msg);
    void loadVirtualImage();
};