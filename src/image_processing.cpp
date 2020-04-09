/* Code provides service for image processing and save to system */
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/format.hpp>
#include "webcam_ros/cropImage.h"

// using namespace cv;

boost::format g_format;

class Callbacks {
public:
  Callbacks(): _count(0){
  }
  void webcam_callback(const sensor_msgs::ImageConstPtr &image_ptr){
    // Set the current frame for use by other services
    _image_ptr = image_ptr;
  }
  bool save_image(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    // Simply save the image without any processing
    cv::Mat _image = cv_bridge::toCvCopy(_image_ptr, sensor_msgs::image_encodings::BGR8)->image;
    std::string _filename;
    if(!saveimage(_image, _filename)){
      return true;
    }
    _count ++; // for image numbering during saving
  }
  bool crop_image(webcam_ros::cropImage::Request &req, webcam_ros::cropImage::Response &res){
    cv::Mat _image = cv_bridge::toCvCopy(_image_ptr, sensor_msgs::image_encodings::BGR8)->image;
    // process the image -> performing the crop operation
    int height = _image.rows;
    int width = _image.cols;
    if (req.width < width && req.height < height){
      // perform a central crop operation
      cv::Mat _image_cropped = _image(cv::Rect(int(width - req.width)/2, int(height - req.height)/2, req.width, req.height));
      std::string _filename;
      if(!saveimage(_image_cropped, _filename)){
        return true;
      }
      _count ++;
    } else {
      ROS_WARN("Unable to crop image! Crop dimensions larger than image size");
      return false;
    }
  }
private:
  bool saveimage(const cv::Mat image, std::string &filename){
    if(!image.empty()){
      try{
        filename = (g_format).str();
      } catch (...) {g_format.clear();}
      try{
        filename = (g_format % _count).str();
      } catch (...) {g_format.clear();}
      try{
        filename = (g_format % _count % "jpg").str();
      } catch (...) {g_format.clear();}
      // save the image
      imwrite(filename, image);
      ROS_INFO("Saved Image %s", filename.c_str());
      return true;
    } else {
      ROS_WARN("Couldn't save image! No data");
      return false;
    }
  }
private:
  int _count;
  sensor_msgs::ImageConstPtr _image_ptr;
};

int main (int argc, char **argv){
  ros::init(argc, argv, "image_processing_server");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  // set subscriber for video and service for saving images

  Callbacks callbacks;

  ros::Subscriber video_sub = nh.subscribe<sensor_msgs::Image>("/webcam", 1, &Callbacks::webcam_callback, &callbacks);
  ros::ServiceServer service_save = nh.advertiseService("save_image", &Callbacks::save_image, &callbacks);
  ros::ServiceServer service_crop = nh.advertiseService("crop_image", &Callbacks::crop_image, &callbacks);
  // initialze save_image and filename variable

  std::string filename;
  private_nh.param<std::string>("filename", filename, std::string("left%04i.%s"));
  g_format.parse(filename);
  ROS_INFO("Ready to process and save image");
  ros::spin();
  return 0;
}
