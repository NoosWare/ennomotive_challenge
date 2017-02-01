#ifndef SLAMMER_HPP
#define SLAMMER_HPP
#include "includes.ihh"
#include "planner.hpp"

#define RAD2DEG(x) ((x)*180./M_PI)
#define M_PIf 3.14159265358979f

class slammer 
{
public:

    slammer(ros::NodeHandle & node);

    // callback for laser data
    void read_lazors(const sensor_msgs::LaserScan::ConstPtr & scan);

    // publish pose data
    void publish_pose(mrpt::poses::CPose3D pose);

    // find collision angles
    void collision_angles(std::vector<float> lidar_angles);

    // average lazor readings
    void avg_distance(  
                        std::vector<float> lidar_angles, 
                        int counter,
                        std::vector<char> valid_data
                     );

    // save all data (gridmap, icpmap, coordinate data)
    void serialize();

private:
    mrpt::slam::CMetricMapBuilderICP builder__;
    mrpt::utils::CConfigFile iniFile__;
    
    //Timing
    double start__;
    double dt__ = 0.0f; // in seconds
    double timer__ = 0.0f;

    mrpt::poses::CPose3D sensor__;
    //mrpt::slam::COccupancyGridMap2D grid__;

    ros::Publisher poser__, collision__, free_way__;
};
#endif
