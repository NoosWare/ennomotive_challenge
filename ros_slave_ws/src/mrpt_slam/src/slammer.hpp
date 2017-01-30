#ifndef SLAMMER_HPP
#define SLAMMER_HPP

#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/utils/CConfigFile.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "json.hpp"
#include <math.h>
#include <cmath>
#include "planner.hpp"
#include "navigator.hpp"
#include <functional>
#include <numeric>

#define RAD2DEG(x) ((x)*180./M_PI)
#define M_PIf 3.14159265358979f

// TODO: create a publisher which sends JSON coordinate messages(robotpose) - run every 100ms
// TODO  create a subscriber (`planner`) which receives requests for a path
// TODO: allow a constuctor which loads an existing map
class slammer : public planner, navigator
{
public:

    slammer(ros::NodeHandle & node);

    // callback for laser data
    void read_lazors(const sensor_msgs::LaserScan::ConstPtr & scan);

    void calculate_path();

    void get_pose();

    void collision_angles( std::vector<float> lidar_angles);

    void avg_distance(  std::vector<float> lidar_angles, 
                            int counter,
                            std::vector<char> valid_data
                     );

private:

    mrpt::slam::CMetricMapBuilderICP builder__;
    mrpt::utils::CConfigFile iniFile__;
    
    //Timing
    double start__;
    double dt__ = 0.0f; // in seconds
    double timer__ = 0.0f;

    mrpt::poses::CPose3D sensor__;
    mrpt::slam::COccupancyGridMap2D grid__;

    ros::Publisher poser__, collision__, free_way__;
    
};
#endif
