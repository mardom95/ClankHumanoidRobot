#include "face_tracker.h"

namespace face_tracker {

faceTracker::faceTracker(ros::NodeHandle nh) : nh_(nh), async_spinner_(3), in_tracking_(false){

  async_spinner_.start();
  it_ = new image_transport::ImageTransport(nh_);

  std::string camera_topic, feedbck_topic, command_topic, haar_cascade_path,
      pan_joint, tilt_joint;

  nh_.param<std::string>("camera_topic",camera_topic,"camera/image");
  nh_.param<std::string>("feedback_topic",feedbck_topic,"bioloid/head_controller/state");
  nh_.param<std::string>("command_topic",command_topic,"bioloid/head_controller/command");
  nh_.param<std::string>("haar_cascade_path",haar_cascade_path,"/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_default.xml");
  nh_.param<std::string>("pan_joint",pan_joint,"head_pan_joint");
  nh_.param<std::string>("tilt_joint",tilt_joint,"head_tilt_joint");
  nh_.param<double>("k_p_pan",k_p_pan,0.8);
  nh_.param<double>("k_p_tilt",k_p_tilt,0.8);

  sub_ = it_->subscribe(camera_topic, 1, boost::bind(&faceTracker::imageCallback, this, _1));
  head_cnt_sub_ = nh_.subscribe<control_msgs::JointTrajectoryControllerState>(feedbck_topic,1,boost::bind(&faceTracker::headControllerStateCB, this,_1));
  head_goal_pub = nh_.advertise<trajectory_msgs::JointTrajectory>(command_topic,1);
  // Change path before execution
  cascade_.load(haar_cascade_path) ;

  traj_msg_.joint_names.push_back(pan_joint);
  traj_msg_.joint_names.push_back(tilt_joint);

  scale = 1;

}

void faceTracker::execute(){

  ros::Rate r(40);

  while(ros::ok()){

    if(in_frame_.empty())
      continue;

    //if detection successful
    if(detectAndDraw(in_frame_)){

      //Normalize between [-1,1]
      double x_err = err_px_/(in_frame_.cols/2);
      double y_err = err_py_/(in_frame_.rows/2);

      //Control Law
      double setp_x = pan_curr_pos + (x_err*k_p_pan);
      double setp_y = tilt_curr_pos + -(y_err*k_p_tilt);

      //Saturation x
      if(setp_x > MAX_PAN_ANGLE)
        setp_x = MAX_PAN_ANGLE;
      else if(setp_x < MIN_PAN_ANGLE)
        setp_x = MIN_PAN_ANGLE;

      //Saturation y
      if(setp_y > MAX_TILT_ANGLE)
        setp_y = MAX_TILT_ANGLE;
      else if(setp_y < MIN_TILT_ANGLE)
        setp_y = MIN_TILT_ANGLE;

      traj_msg_.header = std_msgs::Header();

      trajectory_msgs::JointTrajectoryPoint pt;
      pt.time_from_start = ros::Duration(1.0);
      pt.positions.push_back(setp_x);
      pt.positions.push_back(setp_y);
      traj_msg_.points.push_back(pt);
      //ROS_ERROR_STREAM("GOING TO PUBLISH "<< traj_msg_);
      head_goal_pub.publish(traj_msg_);
      traj_msg_.points.clear();
    }

    cv::imshow("DETECTION",in_frame_);
    cv::waitKey(10);
    ros::spinOnce();
    r.sleep();

  }

}

bool faceTracker::detectAndDraw(cv::Mat &img){

  std::vector<cv::Rect> faces, faces2;
  cv::Mat gray, smallImg;

  double img_x_center = in_frame_.cols/2,
         img_y_center = in_frame_.rows/2;

  cv::cvtColor( img, gray, cv::COLOR_BGR2GRAY ); // Convert to Gray Scale
  double fx = 1 / scale;

  // Resize the Grayscale Image
  cv::resize( gray, smallImg, cv::Size(), fx, fx, cv::INTER_LINEAR );
  cv::equalizeHist( smallImg, smallImg );

  // Detect faces of different sizes using cascade classifier

  std::vector<int> rejectionLevel;
  std::vector<double> levelWeights;

  cascade_.detectMultiScale( smallImg, faces, rejectionLevel, levelWeights,
                             1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30),cv::Size(),true);

  // TO IMPLEMENT track for center of detections
  // use in_tracking_
  if(!levelWeights.empty() ){


    if(in_tracking_)
    {
      for(size_t i=0; i < faces.size(); i++)
      {
        if(std::fabs(faces[i].x - rect_tracking_.x)< RECT_X_TRCK_THRESH &&
           std::fabs(faces[i].y - rect_tracking_.y)< RECT_Y_TRCK_THRESH)
        {

          rect_tracking_ = faces[i];
          cv::Mat smallImgROI;
          cv::Point center;
          cv::Scalar color = cv::Scalar(255, 0, 0); // Color for Drawing tool

          cv::rectangle( img, cvPoint(cvRound(rect_tracking_.x*scale), cvRound(rect_tracking_.y*scale)),
                      cvPoint(cvRound((rect_tracking_.x + rect_tracking_.width-1)*scale),
                      cvRound((rect_tracking_.y + rect_tracking_.height-1)*scale)), color, 3, 8, 0);

          //err_px_ = img_x_center - ((rect_tracking_.x + (rect_tracking_.width-1)/2)*scale);
          //err_py_ = img_y_center - ((rect_tracking_.y + (rect_tracking_.height-1)/2)*scale);

          cv::circle(img,cv::Point(rect_tracking_.x,rect_tracking_.y), 20,cv::Scalar(255,255,0),2);
          cv::line(img,cv::Point(img_x_center,img_y_center),cv::Point(img.cols/2,((rect_tracking_.y + ((rect_tracking_.height-1)/2))*scale)),cv::Scalar(0,255,0),2);
          cv::line(img,cv::Point(img_x_center,img_y_center),cv::Point((rect_tracking_.x + ((rect_tracking_.width-1)/2)*scale),img.rows/2),cv::Scalar(0,0,255),2);
          //ROS_WARN_STREAM("----------------------------------");
          return true;

        }
      }
    } else {

      int maxElementIndex = std::max_element(levelWeights.begin(),levelWeights.end()) - levelWeights.begin();

      if(levelWeights[maxElementIndex] > 6.0)
      {
        //ROS_INFO_STREAM("Rejection level " << rejectionLevel[maxElementIndex]);
        //ROS_WARN_STREAM("Level weight " << levelWeights[maxElementIndex]);

        rect_tracking_ = faces[maxElementIndex];
        cv::Mat smallImgROI;
        cv::Point center;
        cv::Scalar color = cv::Scalar(255, 0, 0); // Color for Drawing tool

        cv::rectangle( img, cvPoint(cvRound(rect_tracking_.x*scale), cvRound(rect_tracking_.y*scale)),
                    cvPoint(cvRound((rect_tracking_.x + rect_tracking_.width-1)*scale),
                    cvRound((rect_tracking_.y + rect_tracking_.height-1)*scale)), color, 3, 8, 0);

        err_px_ = img_x_center - ((rect_tracking_.x + (rect_tracking_.width-1)/2)*scale);
        err_py_ = img_y_center - ((rect_tracking_.y + (rect_tracking_.height-1)/2)*scale);

        cv::line(img,cv::Point(img_x_center,img_y_center),cv::Point(img.cols/2,((rect_tracking_.y + ((rect_tracking_.height-1)/2))*scale)),cv::Scalar(0,255,0),2);
        cv::line(img,cv::Point(img_x_center,img_y_center),cv::Point((rect_tracking_.x + ((rect_tracking_.width-1)/2)*scale),img.rows/2),cv::Scalar(0,0,255),2);
        //ROS_WARN_STREAM("----------------------------------");
        return true;
      }

    }

  }

  //ROS_WARN_STREAM("----------------------------------");
  return false;

}

void faceTracker::execute_kallman(){

  // >>>> Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;

    unsigned int type = CV_32F;
    cv::KalmanFilter kf(stateSize, measSize, contrSize, type);

    cv::Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    cv::Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    // Transition State Matrix A
    // Note: set dT at each processing step!
    // [ 1 0 dT 0  0 0 ]
    // [ 0 1 0  dT 0 0 ]
    // [ 0 0 1  0  0 0 ]
    // [ 0 0 0  1  0 0 ]
    // [ 0 0 0  0  1 0 ]
    // [ 0 0 0  0  0 1 ]
    cv::setIdentity(kf.transitionMatrix);

    // Measure Matrix H
    // [ 1 0 0 0 0 0 ]
    // [ 0 1 0 0 0 0 ]
    // [ 0 0 0 0 1 0 ]
    // [ 0 0 0 0 0 1 ]
    kf.measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    // Process Noise Covariance Matrix Q
    // [ Ex   0   0     0     0    0  ]
    // [ 0    Ey  0     0     0    0  ]
    // [ 0    0   Ev_x  0     0    0  ]
    // [ 0    0   0     Ev_y  0    0  ]
    // [ 0    0   0     0     Ew   0  ]
    // [ 0    0   0     0     0    Eh ]
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 1e-2f;
    kf.processNoiseCov.at<float>(21) = 1e-2f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    // <<<< Kalman Filter

    ROS_INFO_STREAM("\nHit 'q' to exit...\n");

    char ch = 0;

    double ticks = 0;

    int notFoundCount = 0;

    // >>>>> Main loop
    while (ch != 'q' && ch != 'Q')
    {
        ROS_WARN_STREAM(kf.gain);
        double precTick = ticks;
        ticks = (double) cv::getTickCount();

        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

        if(in_frame_.empty())
          continue;

        cv::Mat res;
        if(mut.try_lock())
        {
          in_frame_.copyTo( res );
          mut.unlock();
        } else continue;

        if (in_tracking_)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            ROS_INFO_STREAM("dT:" << dT );

            state = kf.predict();
            //ROS_INFO_STREAM("State post:" << state );

            cv::Rect predRect;
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            double img_x_center = in_frame_.cols/2,
                   img_y_center = in_frame_.rows/2;

            err_px_ = img_x_center - ((predRect.x + (predRect.width-1)/2)*scale);
            err_py_ = img_y_center - ((predRect.y + (predRect.height-1)/2)*scale);

            cv::Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            cv::circle(res, center, 2, CV_RGB(255,0,0), -1);

            cv::rectangle(res, predRect, CV_RGB(255,0,0), 2);
        }

        if(!detectAndDraw(res))
        // <<<<< Detection result
        {
          notFoundCount++;
          //ROS_INFO_STREAM("notFoundCount:" << notFoundCount );
          if( notFoundCount >= 50 )
          {
              in_tracking_ = false;
              traj_msg_.header = std_msgs::Header();

              trajectory_msgs::JointTrajectoryPoint pt;
              pt.time_from_start = ros::Duration(1.0);
              pt.positions.push_back(0);
              pt.positions.push_back(0);
              traj_msg_.points.push_back(pt);
              //ROS_ERROR_STREAM("GOING TO PUBLISH "<< traj_msg_);
              head_goal_pub.publish(traj_msg_);
              traj_msg_.points.clear();
          }
          /*else
              kf.statePost = state;*/
        }
        else
        {
            notFoundCount = 0;

            meas.at<float>(0) = rect_tracking_.x + rect_tracking_.width / 2;
            meas.at<float>(1) = rect_tracking_.y + rect_tracking_.height / 2;
            meas.at<float>(2) = (float)rect_tracking_.width;
            meas.at<float>(3) = (float)rect_tracking_.height;

            if (!in_tracking_) // First detection!
            {
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialization

                kf.statePost = state;

                in_tracking_ = true;
            }
            else {
              kf.gain = kf.gain*0.1;
              kf.correct(meas); // Kalman Correction
            }

            //Normalize between [-1,1]
            double x_err = err_px_/(in_frame_.cols/2);
            double y_err = err_py_/(in_frame_.rows/2);

            //Control Law
            double setp_x = pan_curr_pos + (x_err*k_p_pan);
            double setp_y = tilt_curr_pos + -(y_err*k_p_tilt);

            //Saturation x
            if(setp_x > MAX_PAN_ANGLE)
              setp_x = MAX_PAN_ANGLE;
            else if(setp_x < MIN_PAN_ANGLE)
              setp_x = MIN_PAN_ANGLE;

            //Saturation y
            if(setp_y > MAX_TILT_ANGLE)
              setp_y = MAX_TILT_ANGLE;
            else if(setp_y < MIN_TILT_ANGLE)
              setp_y = MIN_TILT_ANGLE;

            traj_msg_.header = std_msgs::Header();

            trajectory_msgs::JointTrajectoryPoint pt;
            pt.time_from_start = ros::Duration(1.0);
            pt.positions.push_back(setp_x);
            pt.positions.push_back(setp_y);
            traj_msg_.points.push_back(pt);
            //ROS_ERROR_STREAM("GOING TO PUBLISH "<< traj_msg_);
            head_goal_pub.publish(traj_msg_);
            traj_msg_.points.clear();


            //ROS_INFO_STREAM("Measure matrix:" << meas );
        }
        // <<<<< Kalman Update

        // Final result
        cv::imshow("Tracking", res);

        // User key
        ch = cv::waitKey(1);
    }
    // <<<<< Main loop

}

void faceTracker::imageCallback(const sensor_msgs::ImageConstPtr& msg){

  try{
      if(mut.try_lock())
      {
        in_frame_ = cv_bridge::toCvShare(msg, "bgr8")->image;
        mut.unlock();
      }


  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

}

void faceTracker::headControllerStateCB(const control_msgs::JointTrajectoryControllerStateConstPtr& msg){

  // 0 Pan joint, 1 Tilt joint
  pan_curr_pos = msg->actual.positions[0];
  tilt_curr_pos = msg->actual.positions[1];

  //ROS_INFO_STREAM("!!!!!!!! PAN RECEIVED " << pan_curr_pos);
}

}
