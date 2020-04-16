/* Server to blur images from portion of video feed and provide current frame and
detection results as feedback. Server provides total frames proessesed and total
detections as result of the action */

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <webcam_ros/BlurAction.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class BlurAction{
public:
  //Constructor for the class
  BlurAction(std::string name):
    _server(_nh, name, false),
    _name(name)
  {
    // callbacks
    _server.registerGoalCallback(boost::bind(&BlurAction::goalCB, this));
    _server.registerPreemptCallback(boost::bind(&BlurAction::preemptCB, this));
    // subsctiber to webcam data
    _sub = _nh.subscribe("/webcam", 1, &BlurAction::analysisCB, this);
    _server.start();
  }
  // destructor for the class
  ~BlurAction(void){
  }
  // TODO: Define Callbacks
  void goalCB(){
    // accept the goal
    _start_time = ros::Time::now();
    _duration = _server.acceptNewGoal()->duration;
    ros::Duration duration(_duration);
    _end_time = _start_time + duration;
  }

  void preemptCB(){
    ROS_INFO("%s : Preempted", _name.c_str());
    _server.setPreempted();
  }

  void analysisCB(const sensor_msgs::ImageConstPtr &image_msg){
    // check if action is active
    if(!_server.isActive()){
      return;
    }
    if (ros::ok() && ros::Time::now() < _end_time){
      // get cvImage from ros message
      _feedback.frame_number = _total_processed;
      _total_processed++;
      cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      // perform blur operation on image
      cv::Mat image_blur;
      cv::blur(image, image_blur, cv::Size(2,2));
      // convert to sensor_msgs/Image
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_blur).toImageMsg();
      _feedback.image = *msg;
      _server.publishFeedback(_feedback);
    }
    if (ros::Time::now() > _end_time){
      _result.total_frames_processed = _total_processed;
      ROS_INFO("%s: Action Completed", _name.c_str());
      _server.setSucceeded(_result);
    }
  }
protected:
  ros::NodeHandle _nh;
  actionlib::SimpleActionServer<webcam_ros::BlurAction> _server;
  std::string _name;
  ros::Subscriber _sub;
  webcam_ros::BlurFeedback _feedback;
  webcam_ros::BlurResult _result;
  int _duration;
  ros::Time _start_time, _end_time;
  int _total_processed;
};

int main(int argc, char **argv){
  ros::init(argc, argv, "blur_server");
  BlurAction blur_action(ros::this_node::getName());
  ros::spin();

  return 0;
}
