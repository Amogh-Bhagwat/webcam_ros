/* Code to call blur action and display the blurred video */
#include <ros/ros.h>
#include <webcam_ros/BlurAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

void feedbackcb(const webcam_ros::BlurFeedbackConstPtr &feedback){
  // TODO: Implement Callback for feedback
  cv::Mat blur_image = cv_bridge::toCvCopy(feedback->image, sensor_msgs::image_encodings::BGR8)->image;
  // Display the image in window
  if (blur_image.data){
    cv::namedWindow("Blur Window", cv::WINDOW_AUTOSIZE);
    cv::imshow("Blur Window", blur_image);
    cv::waitKey(3);
  }
}

void donecb(const actionlib::SimpleClientGoalState &state,
            const webcam_ros::BlurResultConstPtr &result){
  // TODO: Implement Callback for result
  ROS_INFO("Total Processed Frames: %s", std::to_string(result->total_frames_processed).c_str());
}

void activecb(){
  // no operation on active
  ROS_INFO("Action Active");
}
int main (int argc, char **argv){
  ros::init(argc, argv, "blur_client");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  double duration;
  private_nh.param("duration", duration, 10.0);
  // define the simple action client
  actionlib::SimpleActionClient<webcam_ros::BlurAction> client("blur_operation");
  // wait for server to start
  client.waitForServer();
  // send goal to server
  webcam_ros::BlurGoal goal;
  goal.duration = duration;
  client.sendGoal(goal, &donecb, &activecb, &feedbackcb);
  bool finished = client.waitForResult(ros::Duration(30.0));
  if(finished){
    actionlib::SimpleClientGoalState state = client.getState();
    ROS_INFO("Action Finished: %s", state.toString().c_str());
  } else {
    ROS_INFO("Action Time Out");
  }
  return 0;
}
