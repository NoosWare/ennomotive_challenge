#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include "slammer.hpp"

#include <sstream>
#include <fstream>
#include <memory>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_slam");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    slammer slam_obj(n);

    // "/scan" is for RPLIDAR messages
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &slammer::read_lazors, &slam_obj);
    slam_obj.get_pose();

    ros::spin();
    loop_rate.sleep();

    return 0;
}
