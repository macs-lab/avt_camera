#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

// this node acts as a virtual camera node that publish some images to the camera topic.

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("avt_camera_img", 1);
  // change file path as needed
  std::string package_path = ros::package::getPath("avt_camera");
  std::string img_path =  package_path + "/images/test_image.jpeg";
  cv::Mat image = cv::imread(img_path, CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
  // change the publish rate here
  ros::Rate loop_rate(1);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}