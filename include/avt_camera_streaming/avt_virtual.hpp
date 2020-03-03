#include <iostream>
#include <stdio.h>
#include <unordered_map> 

#include <ros/ros.h>
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>


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

    ros::NodeHandle mNodeHandle;
    std::string mSubTopic;
    std::string mPubTopic;
    ros::Subscriber mTriggerSub;
    image_transport::Publisher mImagePub;

    cv::Mat mImageOut;
    // std::vector<string> img_name{"L.png", "R.png", "U.png", "D.png", "F.png", "B.png"};
    std::vector<std::string> mImgList{"L", "R", "U", "D", "F", "B"};
    // std::string mImgPath = 
    std::unordered_map<std::string, cv::Mat> mImageBuffer;

    // camera trigger call back function
    void triggerCb(const std_msgs::String::ConstPtr& msg);
    void loadVirtualImage();
};