#include "ros/ros.h"
#include <iostream>
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
  //initialize node
  ros::init(argc, argv, "camera_publisher");
  ros::NodeHandle nh;
  
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image",1);
  
  VideoCapture cap(0);
  
  if(!cap.isOpened())
  {
	  ROS_ERROR("Cannot open the camera ");
	  return -1;
  }else{
	  
	  ros::Rate r(30);
	  
	  while(ros::ok()){
		  
		  Mat frame;
		  
		  cap >> frame;
		  
		  if(frame.empty())
			break;
			
	      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",frame).toImageMsg();
			
		  pub.publish(msg);
		  
		  r.sleep();
		  ros::spinOnce();
		  
	  }
	  
  }
  cap.release();

  return 0;
}
