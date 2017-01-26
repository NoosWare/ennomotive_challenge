#ifndef SLAMMER_HPP
#define SLAMMER_HPP

#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationIMU.h>
#include <mrpt/utils/CConfigFile.h>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "json.hpp"
#include <math.h>
#include <mutex>

#define RAD2DEG(x) ((x)*180./M_PI)
#define M_PIf 3.14159265358979f

class slammer
{
public:

    /// lazor data
    /// motor commands
    slammer();

    // callback for laser data
    void read_lazors(const sensor_msgs::LaserScan::ConstPtr & scan);

private:

    ///map
    mrpt::slam::CSimpleMap map__;
    mrpt::slam::CMetricMapBuilderICP icp_map__;
    mrpt::utils::CConfigFile iniFile__;
    //Timing
    double start__;
    double dt__ = 0.0f; // in seconds
    double timer__ = 0.0f;
    ///position lidar in the robot
    mrpt::poses::CPose3D sensor__;
    ///motors
    double right_speed__ = 0.0f;
    double left_speed__ = 0.0f;
    double distance__ = 0;
    ///Odometry
    double theta__ = 0;
    double wheel_base__ = 0.158; //in meters (15.8 cm)
    // Pose2D
    double x__ = 0;
    double y__ = 0;
    //Correction timer
    int count = 0;
};

#endif
