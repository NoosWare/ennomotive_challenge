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
    ros::init(argc, argv, "imu");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("imu", 100);
    ros::Rate loop_rate(10);
    auto imu = std::make_unique<imu_broadcaster>();

    // main loop
    while (ros::ok()) {
        // read imu values into a message
        std_msgs::String msg;
        std::stringstream ss;
        ss << imu->read() << std::endl;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
