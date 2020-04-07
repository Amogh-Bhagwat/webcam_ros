#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void webcam_callback(const sensor_msgs::Image::ConstPtr& msg){
  // ROS_INFO("Got frame from webcam");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  imshow("Webcam Display Window", cv_ptr->image);
  waitKey(3);
}

int main(int argc, char **argv){
  // Initialize ros node and define nodehandle
  ros::init(argc, argv, "webcam_sub");
  ros::NodeHandle nh;
  // Open window to show webcam image
  namedWindow("Webcam Display Window", WINDOW_AUTOSIZE);
  // Define subscriber for getting webcam data
  ros::Subscriber sub = nh.subscribe<sensor_msgs::Image>("/webcam", 1, webcam_callback);

  ros::spin();

  return 0;
}
