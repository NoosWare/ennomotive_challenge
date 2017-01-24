#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include "slammer.hpp"

#include <sstream>
#include <fstream>
#include <memory>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    slammer slam_obj;

    // "/scan" is for RPLIDAR messages
    // "motors" is for motor commands
    //
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &slammer::read_lazors, &slam_obj);
    ros::Subscriber sub_motors = n.subscribe<std_msgs::String>("motors", 1000, &slammer::read_motors, &slam_obj);

    slam_obj.update_map();

    ros::spin();
    loop_rate.sleep();

    return 0;
}
