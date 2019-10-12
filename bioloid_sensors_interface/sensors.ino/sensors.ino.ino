// Hardware:
// Pololu A-Star 32U4 Mini SV
// Pololu MinIMU-9 v3 (L3GD20H and LSM303D)
// Interlink FSR 400 Short (x6)
 
// Important! Define this before #include <ros.h>
//#define USE_USBCON
 
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <LSM6.h>
#include <LIS3MDL.h>
 
// Set up the ros node and publishers
ros::NodeHandle nh;
geometry_msgs::Vector3 msg_accel;
ros::Publisher pub_accel("accel", &msg_accel);
geometry_msgs::Vector3 msg_magnet;
ros::Publisher pub_magnet("magnet", &msg_magnet);
geometry_msgs::Vector3 msg_gyro;
ros::Publisher pub_gyro("gyro", &msg_gyro);
unsigned long pubTimer = 0;


LIS3MDL mag;
LSM6 gyro;
// int yellowLEDPin = 13;  // 13
//int LEDBright = 0;
//int LEDDim = 5;
//unsigned long LEDTimer = 0;
 
void setup()
{
 
    nh.initNode();
    nh.advertise(pub_accel);
    nh.advertise(pub_magnet);
    nh.advertise(pub_gyro);
 
    // Wait until connected
    while (!nh.connected())
        nh.spinOnce();
    nh.loginfo("ROS startup complete");
 
    Wire.begin();
 
    
    if (!mag.init())
    {
        nh.logerror("Failed to autodetect compass type!");
        
    }
    mag.enableDefault();
 
    if (!gyro.init())
    {
        nh.logerror("Failed to autodetect gyro type!");
    }
    gyro.enableDefault();
 
    pubTimer = millis();
}
 
void loop()
{
    if (millis() > pubTimer)
    {
        mag.read();
        gyro.read();
 
        // Compass - accelerometer:
        // 16-bit, default range +-2 g, sensitivity 0.061 mg/digit
        // 1 g = 9.80665 m/s/s
        // e.g. value for z axis in m/s/s will be: compass.a.z * 0.061 / 1000.0 * 9.80665
        //      value for z axis in g will be: compass.a.z * 0.061 / 1000.0
        // Gravity is measured as an upward acceleration:
        // Stationary accel. shows +1 g value on axis facing directly "upwards"
        // Convert values to g
        msg_accel.x = (float)(gyro.a.x)*0.061/1000.0;
        msg_accel.y = (float)(gyro.a.y)*0.061/1000.0;
        msg_accel.z = (float)(gyro.a.z)*0.061/1000.0;
        msg_accel.z -= 1;
 
        // Compass - magnetometer:
        // 16-bit, default range +-2 gauss, sensitivity 0.080 mgauss/digit
        msg_magnet.x = mag.m.x;
        msg_magnet.x -= ((int32_t)-3441 + 2371) / 2;

        msg_magnet.y = mag.m.y;
        msg_magnet.y -= ((int32_t)-3292 + 2361) / 2;

        msg_magnet.z = mag.m.z;
        msg_magnet.z -= ((int32_t)-2594 + 2328) / 2;
        
        // Gyro:
        // 16-bit, default range +-245 dps (deg/sec), sensitivity 8.75 mdps/digit
        // Convert values to rads/sec
        msg_gyro.x = (float)(gyro.g.x)*0.00875*M_PI/180.0;
        msg_gyro.x -= 0.0488;
        msg_gyro.y = (float)(gyro.g.y)*0.00875*M_PI/180.0;
        msg_gyro.y += 0.18387;
        msg_gyro.z = (float)(gyro.g.z)*0.00875*M_PI/180.0;
        msg_gyro.z += 0.0605;
 
        pub_accel.publish(&msg_accel);
        pub_magnet.publish(&msg_magnet);
        pub_gyro.publish(&msg_gyro);
 
        pubTimer = millis() + 10;  // wait at least 10 msecs between publishing
    }
 
    nh.spinOnce();
}
