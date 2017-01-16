#ifndef _imu_broadcaster_HPP
#define _imu_broadcaster_HPP
#include "vector.hpp"
#include "MinIMU9.hpp"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "includes.ihh"
#include "sensor_msgs/Imu.h"
/**
 * @brief bridge class which uses IMU/I2C and publishes IMU sensor data
 * @version 0.1.0
 * @date 16-01-2017
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 */
class imu_broadcaster
{
public:

    /// construct
    imu_broadcaster(ros::NodeHandle & n);

    /// @brief broadcast once
    void broadcast();

    /// @brief broadcast TF of the robot
    void robot_transform();

protected: 

    // ETA in ms
    unsigned int millis();

    // calculate the quaternion using the magnetometer and angular velocity
    void consume(
                    quaternion & rotation, 
                    float dt, 
                    const vector & angular_velocity,
                    const vector & acceleration, 
                    const vector & magnetic_field
                 );

private:
    quaternion rotation__;
    MinIMU9 imu__;
    unsigned int start__ = 0;
    ros::Publisher imu_pub__; 
};
#endif
