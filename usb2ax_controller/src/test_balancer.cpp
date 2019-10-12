//#include <curses.h>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "std_srvs/Empty.h"
#include "usb2ax_controller/SetMotorParam.h"
#include "../../bioloid_master/src/simplepid.h"
#include <deque>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/JointTrajectoryControllerState.h>

double left_ankle_pos=0.0f, right_ankle_pos =0.0f;

void AXStateCBLeft(const control_msgs::JointTrajectoryControllerStateConstPtr& msg)
{
    auto itr = std::find(msg->joint_names.begin(),msg->joint_names.end(),"left_ankle_swing_joint");

    if(itr != msg->joint_names.end())
    {
      int idx = std::distance(msg->joint_names.begin(),itr);
      left_ankle_pos = msg->actual.positions[idx];
    }

    //ROS_WARN_STREAM("left ankle pos " << left_ankle_pos);
}

int main(int argc, char **argv)
{
    //initscr();  // ncurses
    //timeout(0);

    ros::init(argc, argv, "test_balancer");

    ros::NodeHandle n;

    ros::AsyncSpinner spinners(2);
    spinners.start();
    //ros::ServiceClient getMotorCurrentPositionInRadClient =
    //        n.serviceClient<usb2ax_controller::GetMotorParam>("GetMotorCurrentPositionInRad");
    //ros::ServiceClient setMotorGoalPositionInRadClient =
    //        n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalPositionInRad");
    //ros::ServiceClient setMotorGoalSpeedInRadPerSecClient =
    //       n.serviceClient<usb2ax_controller::SetMotorParam>("SetMotorGoalSpeedInRadPerSec");
    //ros::ServiceClient homeAllMotorsClient =
    //        n.serviceClient<std_srvs::Empty>("HomeAllMotors");

    //usb2ax_controller::GetMotorParam getMotorParamSrv;
    usb2ax_controller::SetMotorParam setMotorParamSrv;
    std_srvs::Empty emptySrv;

    trajectory_msgs::JointTrajectory traj_msg_left,traj_msg_right;
    traj_msg_left.joint_names.push_back("left_ankle_lateral_joint");
    traj_msg_left.joint_names.push_back("left_ankle_swing_joint");
    traj_msg_left.joint_names.push_back("left_hip_lateral_joint");
    traj_msg_left.joint_names.push_back("left_hip_swing_joint");
    traj_msg_left.joint_names.push_back("left_hip_twist_joint");
    traj_msg_left.joint_names.push_back("left_knee_joint");


    traj_msg_right.joint_names.push_back("right_ankle_lateral_joint");
    traj_msg_right.joint_names.push_back("right_ankle_swing_joint");
    traj_msg_right.joint_names.push_back("right_hip_lateral_joint");
    traj_msg_right.joint_names.push_back("right_hip_swing_joint");
    traj_msg_right.joint_names.push_back("right_hip_twist_joint");
    traj_msg_right.joint_names.push_back("right_knee_joint");

    ros::Subscriber left_leg_sub = n.subscribe<control_msgs::JointTrajectoryControllerState>("bioloid/left_leg_controller/state",1,AXStateCBLeft);
    ros::Publisher left_leg_pub = n.advertise<trajectory_msgs::JointTrajectory>("bioloid/left_leg_controller/command",1);
    ros::Publisher right_leg_pub = n.advertise<trajectory_msgs::JointTrajectory>("bioloid/right_leg_controller/command",1);

    // Set slow speed
    //setMotorParamSrv.request.dxlID = 254;
    //setMotorParamSrv.request.value = 1.0;
    //setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
    //ros::Duration(0.5).sleep();

    // Home all motors
    //homeAllMotorsClient.call(emptySrv);
    // Temp fix for left_hip_swing_joint
    //setMotorParamSrv.request.dxlID = 12;
    //setMotorParamSrv.request.value = 0.1173;
    //setMotorGoalPositionInRadClient.call(setMotorParamSrv);
    //ros::Duration(3).sleep();

    // Set full speed
    setMotorParamSrv.request.dxlID = 254;
    setMotorParamSrv.request.value = 0.0;
    //setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
    ros::Duration(0.5).sleep();

    tf::TransformListener listener;
    tf::StampedTransform transform;

    tf::Quaternion q;
    float position;

    float Ku, Tu, Kp, Ki, Kd;
    float SP, PV, output;
    float maxSpeed = 11.8668;

    SP = 0.0;

    // Ziegler–Nichols tuning
//    Ku = 0.5;
//    Tu = 0.7;

    // PI
//    Kp = 0.45*Ku;
//    Ki = 1.2*Kp/Tu;
//    Kd = 0.0;

    // PID
//    Kp = 0.6*Ku;
//    Ki = 2*Kp/Tu;
//    Kd = Kp*Tu/8.0;

    Kp = 1.0;
    Ki = 0.0;
    Kd = 0.0;

    std::cout.precision(6);
    std::cout.setf(std::ios::fixed, std::ios::floatfield);

    std::cout << "Ku: \t" << Ku << std::endl;
    std::cout << "Tu: \t" << Tu << std::endl;
    std::cout << "Kp: \t" << Kp << std::endl;
    std::cout << "Ki: \t" << Ki << std::endl;
    std::cout << "Kd: \t" << Kd << std::endl;

    SimplePid pid(Kp, Ki, Kd, 1.4, -0.4);

    // Simple Moving Average
    float SMA = 0.0;
    int SMA_n = 5;
    std::deque<float> SMA_window;
    for (int i=0; i<SMA_n; ++i)
        SMA_window.push_back(0.0);
    float SMA_newValue, SMA_oldValue, V=0.0, Pitch_oldValue;

    double t0 = ros::Time::now().toSec();

    ros::Rate r(30);

    while (n.ok())
    {
        //int i = getch();
        //printw("Entered: %d ", i);

        try
        {
            listener.lookupTransform("odom", "imu_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
          ROS_ERROR("%s", ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // Moving average filter for pitch angle from IMU
        q = transform.getRotation();
        SMA_newValue = asin( -2.0*(q.x()*q.z() - q.y()*q.w()) / q.length() );
        SMA_oldValue = SMA_window.front();
        SMA += (SMA_newValue - SMA_oldValue)/SMA_n;
        SMA_window.pop_front();
        SMA_window.push_back(SMA_newValue);

        // PID control
        PV=SMA;
        //if(SMA > -0.05)
          //PV = SMA-0.15;

        output = pid.update(SP - PV, PV);

        /*if ( fabs(output) >= 0.005 )
        {
            // Set motor speeds - ankle swing joints
            setMotorParamSrv.request.value = output;
            setMotorParamSrv.request.dxlID = 15;
            setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);
            setMotorParamSrv.request.dxlID = 16;
            setMotorGoalSpeedInRadPerSecClient.call(setMotorParamSrv);

            // Set motor outputs - ankle swing joints
            position = -PV;
            setMotorParamSrv.request.value = position;
            setMotorParamSrv.request.dxlID = 15;
            setMotorGoalPositionInRadClient.call(setMotorParamSrv);
            setMotorParamSrv.request.dxlID = 16;
            setMotorGoalPositionInRadClient.call(setMotorParamSrv);

            ROS_WARN_STREAM("Speed : " << V);
            ROS_WARN_STREAM("Speed : " << V);
            ROS_WARN_STREAM("Position : " << position);
            ROS_WARN_STREAM("pitch angle: " << PV);
            //std::cout << "pitch angle: " << PV;
            //std::cout << "\t output speed: " << output;
            //std::cout << "\t output pos: " << position << std::endl;
        }
        else
            std::cout << "Speed too low: " << output << std::endl;
        */
        traj_msg_left.header = std_msgs::Header();
        ROS_WARN_STREAM("pitch angle: " << PV);
        std::cout << "\t output left ankle pos: " <<left_ankle_pos + output;
        trajectory_msgs::JointTrajectoryPoint pt;
        pt.time_from_start = ros::Duration(0.5);
        pt.positions.push_back(0.0);
        pt.positions.push_back(left_ankle_pos+output);//this
        pt.positions.push_back(0.0);
        pt.positions.push_back(0.0);
        pt.positions.push_back(0.0);
        pt.positions.push_back(0.0);
        traj_msg_left.points.push_back(pt);
        traj_msg_right.points.push_back(pt);
        //ROS_ERROR_STREAM("GOING TO PUBLISH "<< traj_msg_left);
        left_leg_pub.publish(traj_msg_left);
        right_leg_pub.publish(traj_msg_right);
        traj_msg_left.points.clear();
        traj_msg_right.points.clear();
        ros::spinOnce();
        r.sleep();
    }
    //endwin();  // ncurses

    return 0;
}
