#ifndef SLAMMER_HPP
#define SLAMMER_HPP

#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationIMU.h>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "json.hpp"
#include "imu/vector.hpp"
#include "imu/imu_broadcaster.hpp"
#include <math.h>

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

    // callback for motor commands
    void read_motors(const std_msgs::String::ConstPtr& motors_data); 

    // update map
    void update_map();

    //
    mrpt::poses::CPose3D calculate_pose();

private:

    ///map
    mrpt::slam::COccupancyGridMap2D  map__;
    ///lidar observation
    std::shared_ptr<mrpt::slam::CObservation2DRangeScan> lidar_obs__;
    //Timing
    double start__;
    double time__;
    ///IMU
    imu_broadcaster imu__;
    ///position lidar in the robot
    mrpt::poses::CPose3D sensor__;
    ///motors
    float right_speed__ = 0.0f;
    float left_speed__ = 0.0f;
    ///Odometry
    float distance__ = 0.0f;

};

#endif
