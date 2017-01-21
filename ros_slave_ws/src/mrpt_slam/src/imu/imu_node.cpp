#include "ros/ros.h"
#include "includes.ihh"
#include "imu_broadcaster.hpp"

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

    ros::Rate loop_rate(10);
    imu_broadcaster imu_brd(n);

    // main loop
    while (ros::ok()) {
        // broadcast once
        imu_brd.broadcast();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
