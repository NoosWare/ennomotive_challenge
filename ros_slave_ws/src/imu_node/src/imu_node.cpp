#include "vector.hpp"
#include "MinIMU9.hpp"
#include "std_msgs/String.h"
#include "ros/ros.h"
#include "includes.ihh"

// ETA in ms
inline int millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

// calculate the quaternion using the magnetometer and angular velocity
// TODO: move to another class
// 
inline void fuse_default(
                          quaternion & rotation, 
                          float dt, 
                          const vector & angular_velocity,
                          const vector & acceleration, 
                          const vector & magnetic_field
                        )
{
    vector correction = vector(0, 0, 0);
    if (abs(acceleration.norm() - 1) <= 0.3)
    {
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

// TODO: calculate velocity in the 3D space (magnitude and speed)
//       and publish that instead
//
// TODO: find min-max acceleration values for normalisation
//
// TODO: publish a JSON which is humanly readable
//
// TODO: average values in a 20ms step, for the 100ms spin of ROS
//       doign so we should get more "Smooth" and accurate readings

/**
 * @brief main ROS node reading IMU9 values from I2C and publishing them
 * @topic `imu_node`
 * @publisher `imu_publisher`
 * @version 0.1.0
 * @see https://github.com/DavidEGrayson/minimu9-ahrs
 */
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle n;
    // obtain pub handler as `imu_publisher`
    ros::Publisher imu_pub = n.advertise<std_msgs::String>("imu_publisher", 1000);
    // publish at 10Hz (e.g., every 100ms)
    ros::Rate loop_rate(10);
    
    // IMU/I2C
    std::string mode = "normal";
    std::string i2cDevice = "/dev/i2c-1";
    MinIMU9 imu(i2cDevice.c_str());
    imu.loadCalibration();
    imu.enable();
    imu.measureOffsets();
    quaternion rotation = quaternion::Identity();
    int start = millis(); // truncate 64-bit return value

    // main loop
    while (ros::ok()) {

        // FROM HERE
        int last_start = start;
        start = millis();
        // dt is seconds (or decimal) used to calculate radians per second
        float dt = (start - last_start) / 1000.0;
        if (dt < 0){ 
            throw std::runtime_error("Time went backwards."); 
        }
        // gyroscope is angular velocity
        vector angular_velocity = imu.readGyro();
        // acceleration in 3 axes
        vector acceleration = imu.readAcc();
        // magnetic field in 3 axes
        vector magnetic_field = imu.readMag();
        fuse_default(rotation, dt, angular_velocity, acceleration, magnetic_field);
        // TO HERE
        // REPEAT every 20ms, then average and report the average readings.
        
        std_msgs::String msg;
        std::stringstream ss;
        // rotation is quaternion or euler in 3D space
        // acceleration (including 1G of gravity pull)
        // magnetic field based on the compass
        ss << rotation << "  " << acceleration << "  " << magnetic_field << std::endl;
        msg.data = ss.str();

        // superflous
        ROS_INFO("%s", msg.data.c_str());

        imu_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
