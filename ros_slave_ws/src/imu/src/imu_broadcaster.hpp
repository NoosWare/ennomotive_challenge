#ifndef _imu_broadcaster_HPP
#define _imu_broadcaster_HPP

#include "vector.hpp"
#include "MinIMU9.hpp"
#include "includes.ihh"
#include "std_msgs/String.h"

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
    imu_broadcaster();

    /// @brief read and publish
    std::string read();

protected: 

    // ETA in ms
    unsigned int millis();

    // calculate the quaternion using the magnetometer and angular velocity
    void to_quaternion(
                        quaternion & rotation, 
                        float dt, 
                        const vector & angular_velocity,
                        const vector & acceleration, 
                        const vector & magnetic_field
                     );

    /// filter 
    vector dc_block_filter(
                           vector R,
                           vector raw_data,
                           vector previous_raw_data,
                           vector previous_filtered_data
                         );

private:
    ///Rotation
    quaternion rotation__;
    ///IMU
    MinIMU9 imu__;
    ///seconds
    unsigned int start__ = 0;
    ///delta time
    float dt__ = 0.0f;
    /// previous raw accel
    vector prev_raw_accel__;
    /// previous filtered accel
    vector prev_filt_accel__;
};
#endif
