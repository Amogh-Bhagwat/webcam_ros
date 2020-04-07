#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

bool save_image;
std::string path, filename;

bool capture(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp){
  save_image = true;
  return true;
}

void webcam_callback(const sensor_msgs::ImageConstPtr &image){
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
  if (save_image == true){
    filename = path + image->header.frame_id + ".jpg";
    imwrite(filename, cv_ptr->image);
    ROS_INFO("Saved Image %s", filename.c_str());
    save_image = false;
  }
}

int main (int argc, char **argv){
  ros::init(argc, argv, "image_saver_server");
  ros::NodeHandle nh;
  // set subscriber for video and service for saving images
  ros::Subscriber video_sub = nh.subscribe<sensor_msgs::Image>("/webcam", 1, webcam_callback);
  ros::ServiceServer service = nh.advertiseService("image_saver", capture);
  // initialze save_image and filename variable
  save_image = false;
  path = "/home/amogh/images/";
  ROS_INFO("Ready to capture image");

  ros::spin();
  return 0;
}
