#ifndef NAVIGATOR_HPP
#define NAVIGATOR_HPP

#include <mrpt/poses/CPose3D.h>
#include <mrpt/reactivenav/CHolonomicVFF.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/reactivenav/CHolonomicLogFileRecord.h>

#include "json.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"

#define RAD2DEG(x) ((x)*180./M_PI)
#include <math.h>

class navigator 
{
public:

    navigator(ros::NodeHandle & node);

    void navigate(mrpt::poses::CPose3D robotpose,
                  std::vector<float> laser_data);

    mrpt::math::TPoint2D calculate_target(mrpt::poses::CPose3D robopose);

    void navigation_result(
                                         mrpt::poses::CPose3D robotpose, 
                                         double angle
                                     ); 
    //action_type actual_action(const std_msgs::String::ConstPtr& msg);
    double dc_block_filter(
                             double raw_data,
                             double previous_raw_data,
                             double previous_filtered_data
                           );
private:

    mrpt::utils::CConfigFile iniFile__;
    mrpt::reactivenav::CHolonomicLogFileRecordPtr log__;
    mrpt::reactivenav::CHolonomicVFF reactivenav__;

    ros::Publisher poser__;
    ros::Subscriber subs__;

    std::deque<double> angle_samples__;
    int counter__;


};
#endif
