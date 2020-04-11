/* Code provides service for image processing */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include "webcam_ros/cropImage.h"
#include "webcam_ros/saveImage.h"

// using namespace cv;

class Callbacks {
public:
  Callbacks(){
  }
  void webcam_callback(const sensor_msgs::ImageConstPtr &image_ptr){
    // Set the current frame for use by other services
    _image_ptr = image_ptr;
  }
  bool save_image(webcam_ros::saveImage::Request &req, webcam_ros::saveImage::Response &res){
    // Simply save the image without any processing
    cv::Mat _image = cv_bridge::toCvCopy(_image_ptr, sensor_msgs::image_encodings::BGR8)->image;
    if(!_image.empty()){
      cv::imwrite(req.filename, _image);
      ROS_INFO("Saved Image %s", req.filename.c_str());
      res.success = true;
    } else {
      ROS_WARN("Couldn't save image! No data");
      res.success = false;
    }
  }
  bool crop_image(webcam_ros::cropImage::Request &req, webcam_ros::cropImage::Response &res){
    cv::Mat _image = cv_bridge::toCvCopy(_image_ptr, sensor_msgs::image_encodings::BGR8)->image;
    // process the image -> performing the crop operation
    int height = _image.rows;
    int width = _image.cols;
    if (req.width <= width && req.height <= height){
      // perform a central crop operation
      cv::Mat _image_cropped = _image(cv::Rect(int(width - req.width)/2, int(height - req.height)/2, req.width, req.height));
      // convert to sensor_msgs/Image
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _image_cropped).toImageMsg();
      res.image = *msg;
      res.success = true;
    } else {
      ROS_WARN("Unable to crop image! Crop dimensions larger than image size");
      return true;
      res.success = false;
    }
  }
private:
  sensor_msgs::ImageConstPtr _image_ptr;
};

int main (int argc, char **argv){
  ros::init(argc, argv, "image_processing_server");
  ros::NodeHandle nh;
  // set subscriber for video and service for saving images

  Callbacks callbacks;

  ros::Subscriber video_sub = nh.subscribe<sensor_msgs::Image>("/webcam", 1, &Callbacks::webcam_callback, &callbacks);
  ros::ServiceServer service_save = nh.advertiseService("save_image", &Callbacks::save_image, &callbacks);
  ros::ServiceServer service_crop = nh.advertiseService("crop_image", &Callbacks::crop_image, &callbacks);
  // initialze save_image and filename variable

  ROS_INFO("Ready to process and save image");
  ros::spin();
  return 0;
}
