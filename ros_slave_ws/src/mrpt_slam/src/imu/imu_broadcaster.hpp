#ifndef _imu_broadcaster_HPP
#define _imu_broadcaster_HPP
#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CObservationIMU.h>

#include "vector.hpp"
#include "MinIMU9.hpp"
#include "includes.ihh"
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

    /// @return quaternion and velocity
    void read(const std::shared_ptr<mrpt::slam::CObservationIMU> obs);

    /// @brief broadcast TF of the robot
    void make_velocity(vector acceleration);

    /// @brief get latest position stimation
    vector get_velocity();

    /// @brief convert to CPose3D
    //mrpt::poses::CPose3D convert_to_3dpose();

    ///
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
    ///Rotation
    quaternion rotation__;
    ///IMU
    MinIMU9 imu__;
    ///seconds
    unsigned int start__ = 0;
    ///Previous velocity
    vector previous_velocity__;
    ///Previous position
    vector previous_position__;
    ///delta time
    float dt__ = 0.0f;

};
#endif
