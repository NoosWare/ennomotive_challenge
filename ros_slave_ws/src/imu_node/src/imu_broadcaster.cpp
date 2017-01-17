#include "imu_broadcaster.hpp"

imu_broadcaster::imu_broadcaster(ros::NodeHandle & n)
: imu__("/dev/i2c-1"), 
  start__(millis()),
  imu_pub__(n.advertise<std_msgs::String>("imu_publisher", 1000))
{
    std::string mode = "normal";
    //std::string i2cDevice = ;
    //MinIMU9 imu(i2cDevice.c_str());
    imu__.loadCalibration();
    imu__.enable();
    imu__.measureOffsets();
    quaternion rotation__ = quaternion::Identity();
}

/// @brief broadcast once
void imu_broadcaster::broadcast()
{
    int last_start = start__;
    start__ = millis();
    // dt is seconds (or decimal) used to calculate radians per second
    float dt = (start__ - last_start) / 1000.0;
    if (dt < 0){ 
        throw std::runtime_error("Time went backwards."); 
    }
    // angular velocity => gyroscope
    vector angular_velocity = imu__.readGyro();
    // linear acceleration => accelelerometer
    vector acceleration = imu__.readAcc();
    // magnetic field in 3 axes
    vector magnetic_field = imu__.readMag();
    consume(rotation__, dt, angular_velocity, acceleration, magnetic_field);
    // construct and publish emssage
    std_msgs::String msg;
    std::stringstream ss;
    // quaternion, linear acceleration, magnetic field
    // TODO: put this in a JSON?
    ss << rotation__ << "  " << acceleration << "  " << magnetic_field << std::endl;
    msg.data = ss.str();
    imu_pub__.publish(msg);
    // TODO: publish/broadcast robot transform (TF)
}

unsigned int imu_broadcaster::millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

void imu_broadcaster::consume(
                                quaternion & rotation, 
                                float dt, 
                                const vector & angular_velocity,
                                const vector & acceleration, 
                                const vector & magnetic_field
                             )
{
    vector correction = vector(0, 0, 0);
    if (abs(acceleration.norm() - 1) <= 0.3) {
        // The magnetidude of acceleration is close to 1 g, so
        // it might be pointing up and we can do drift correction.
        const float correction_strength = 1;
        // 
        matrix rotationCompass = rotationFromCompass(acceleration, magnetic_field);
        matrix rotationMatrix = rotation.toRotationMatrix();
        correction = (
            rotationCompass.row(0).cross(rotationMatrix.row(0)) +
            rotationCompass.row(1).cross(rotationMatrix.row(1)) +
            rotationCompass.row(2).cross(rotationMatrix.row(2))
          ) * correction_strength;
    }
    rotate(rotation, angular_velocity + correction, dt);
}
