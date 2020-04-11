#include <ros/ros.h>
#include "webcam_ros/saveImage.h" // service header
#include <string>
#include <boost/algorithm/string.hpp>
#include <vector>

int main (int argc, char **argv){
  ros::init(argc, argv, "image_save_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  // client to periodically save image from webcam stream
  std::string filename;
  int capture_fps;
  double capture_duration;
  private_nh.param("filename", filename, std::string("images/image.jpg"));
  private_nh.param("capture_fps", capture_fps, 1);
  private_nh.param("capture_duration", capture_duration, 10.0);
  // split filename into name and extenstion
  std::vector<std::string> result;
  std::string file;
  std::string extension;
  boost::split(result, filename, boost::is_any_of("."));
  if (result.size() != 2){
    ROS_ERROR("Filename is invalid");
    return 1;
  } else {
    file = result[0];
    extension = "." + result[1];
  }
  // TODO: check for correctness of filename, capture_fps and capture_duration
  ros::ServiceClient client = nh.serviceClient<webcam_ros::saveImage>("save_image");
  webcam_ros::saveImage srv;
  ros::Rate rate(capture_fps);
  ros::Time start_time = ros::Time::now();
  ros::Duration loop_duration(capture_duration);
  int _count = 0; // count to modify filename
  while(ros::ok() && ros::Time::now() < start_time + loop_duration){
    // TODO: Add count to filename
    std::string filename_ = file + std::to_string(_count) + extension;
    srv.request.filename = filename_;
    if (client.call(srv)){
      if (srv.response.success){
        rate.sleep();
        _count++;
      } else {
        ROS_ERROR("Couldn't save current image");
      }
    } else {
      ROS_ERROR("Failed to call service");
      return 1;
    }
  }
  return 0;
}
