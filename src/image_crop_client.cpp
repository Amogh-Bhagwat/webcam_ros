#include <ros/ros.h>
#include <webcam_ros/cropImage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

int main (int argc, char **argv){
  ros::init(argc, argv, "image_crop_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  // define parameters for corp operation on single image
  std::string filename;
  int width, height;
  private_nh.param("filename", filename, std::string("images/crop_image.jpg"));
  private_nh.param("width", width, 640);
  private_nh.param("height", height, 480);
  // TODO:sanity check for parameters
  if (width > 640 || height > 480) {
    ROS_WARN("Crop operation not possible with given parameters. Check Parameters");
    return 1;
  }
  // define client
  ros::ServiceClient client = nh.serviceClient<webcam_ros::cropImage>("crop_image");
  webcam_ros::cropImage srv;
  srv.request.width = width;
  srv.request.height = height;
  if (client.call(srv)){
    if(srv.response.success){
      sensor_msgs::Image image = srv.response.image;
      // convert image message to cv::Mat and save to file
      cv::Mat _image = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8)->image;
      if(!_image.empty()){
          cv::imwrite(filename, _image);
          ROS_INFO("Saved Cropped image %s", filename.c_str());
      }
    } else {
      ROS_ERROR("Couldn't crop image");
    }
  } else {
    ROS_ERROR("Failed to call service");
  }
  return 0;
}
