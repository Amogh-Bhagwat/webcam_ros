#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;

int main (int argc, char **argv){
  // Initialize rosnode and define the nodehandle for the node
  ros::init(argc, argv, "webcam_pub");
  ros::NodeHandle nh;
  // Define the publisher for the webcam video stream
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/webcam", 1);
  // Define a message having type sensor_msgs::Image
  sensor_msgs::ImagePtr msg;
  // Define frame to store data from webcam
  Mat frame;

  VideoCapture cap;
  // Set device id and api id
  int device_id = 0;
  int api_id = cv::CAP_ANY;

  cap.open(device_id, api_id);
  if(!cap.isOpened()){
    cerr << "Unable to open Camera\n";
    return -1;
  }

  ros::Rate rate(10);
  while(ros::ok()){
    cap.read(frame);
    if (frame.empty()){
      cerr << "Error! Got blank frame";
    }
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
