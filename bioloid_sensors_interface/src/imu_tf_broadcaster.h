#ifndef IMU_TF_BROADCASTER_H
#define IMU_TF_BROADCASTER_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float32.h"
#include "tf/transform_broadcaster.h"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>


namespace bacc = boost::accumulators;

class Broadcaster
{

public:
    Broadcaster();
    virtual ~Broadcaster();
    void accelCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void magnetCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    void gyroCallback(const geometry_msgs::Vector3::ConstPtr& msg);
    //void updateTimers();
    void updateRotation();
    void correctOrientation();
    tf::TransformBroadcaster* tfBroadcaster;
    tf::Quaternion getQ() const {return q;}
    double getPrevt() const {return prevt;}
    double getDt() const {return dt;}
    float getTimeConst() const {return timeConst;}
    void setTimeConst(float value) {timeConst = value;}

private:

    /// Rolling mean accumulator and window:
    typedef bacc::accumulator_set<double, bacc::stats<bacc::tag::rolling_mean> > RollingMeanAcc;
    typedef bacc::tag::rolling_window RollingWindow;

    tf::Vector3 accel;
    tf::Vector3 magnet;
    tf::Vector3 gyro;

    tf::Vector3 angularVel;
    tf::Quaternion q;
    double prevt;
    double dt;
    float timeConst;
    float filterCoeff;

    /// Rolling mean accumulators for the linar and angular velocities:
    size_t velocity_rolling_window_size_;
    RollingMeanAcc linear_acc_x;
    RollingMeanAcc linear_acc_y;
    RollingMeanAcc linear_acc_z;
};

#endif // IMU_TF_BROADCASTER_H
