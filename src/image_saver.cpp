#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>

using namespace cv;

bool save_image;
int count;
boost::format g_format;

bool capture(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
  save_image = true;
  count = count + 1;
  return true;
}

void webcam_callback(const sensor_msgs::ImageConstPtr &image){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  std::string filename_;
  // setting the count and foramt of the image if not set by user
  if (!cv_ptr->image.empty()){
    try{
      filename_ = (g_format).str();
    } catch (...) {g_format.clear();}
    try{
      filename_ = (g_format % count).str();
    } catch (...) {g_format.clear();}
    try{
      filename_ = (g_format % count % "jpg").str();
    } catch (...) {g_format.clear();}
  }
  if (save_image == true){
    imwrite(filename_, cv_ptr->image);
    ROS_INFO("Saved Image %s", filename_.c_str());
    save_image = false;
  }
}

int main (int argc, char **argv){
  ros::init(argc, argv, "image_saver_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  // set subscriber for video and service for saving images
  ros::Subscriber video_sub = nh.subscribe<sensor_msgs::Image>("/webcam", 1, webcam_callback);
  ros::ServiceServer service = nh.advertiseService("image_saver", capture);
  // initialze save_image and filename variable
  save_image = false;
  count = 0;
  std::string filename;
  private_nh.param<std::string>("filename", filename, std::string("left%04i.%s"));
  g_format.parse(filename);
  ROS_INFO("Ready to capture image");

  ros::spin();
  return 0;
}
