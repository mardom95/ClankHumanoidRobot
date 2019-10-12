#include "ros/ros.h"
#include <iostream>
#include <mutex>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect.hpp>
#include <image_transport/image_transport.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

//control_msgs::JointTrajectoryControllerState
//bioloid/head_controller/state
//bioloid/head_controller/command

#define N_DOF 2 //
#define MAX_PAN_ANGLE 1.3 //left
#define MIN_PAN_ANGLE -1.3 //right
#define MAX_TILT_ANGLE 0.3 //Down
#define MIN_TILT_ANGLE -0.3 //Up

#define RECT_X_TRCK_THRESH 50 //Up
#define RECT_Y_TRCK_THRESH 50 //Up

namespace face_tracker {

  class faceTracker {

  public:

    faceTracker(ros::NodeHandle);
    ~faceTracker(){
      async_spinner_.stop();
      traj_msg_.header = std_msgs::Header();

      trajectory_msgs::JointTrajectoryPoint pt;
      pt.time_from_start = ros::Duration(1.0);
      pt.positions.push_back(0.0);
      pt.positions.push_back(0.0);
      traj_msg_.points.push_back(pt);
      //ROS_ERROR_STREAM("GOING TO PUBLISH "<< traj_msg_);
      head_goal_pub.publish(traj_msg_);
      ros::spinOnce();
    };

    void execute();
    void execute_kallman();

  private:

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void headControllerStateCB(const control_msgs::JointTrajectoryControllerStateConstPtr& msg);
    bool detectAndDraw(cv::Mat &frame);

    ros::NodeHandle nh_;
    image_transport::ImageTransport *it_;
    image_transport::Subscriber sub_;
    ros::Subscriber head_cnt_sub_;
    ros::Publisher head_goal_pub;
    ros::AsyncSpinner async_spinner_;

    cv::Mat in_frame_;

    trajectory_msgs::JointTrajectory traj_msg_;

    double pan_curr_pos, tilt_curr_pos;
    double err_px_, err_py_;

    double scale;

    double k_p_pan, k_p_tilt;

    bool in_tracking_;

    // PreDefined trained XML classifiers with facial features
    cv::CascadeClassifier cascade_;
    cv::Rect rect_tracking_, pred_rect_;

    std::mutex mut;

  };

}
