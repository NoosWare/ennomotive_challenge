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

/// action type (C++1y enum)
enum class action_type
{ 
    forward  = 1,
    backward = 2,
    left = 3,
    right = 4,
    stop  = 0
};

/**
 * @brief interpret an action to motor commands (json)
 * @version 0.1.0
 * @date 27.01.2017
 */
struct action_interpreter
{
    std::string operator()(const action_type action)
    {
        nlohmann::json j;
        switch (action) {
            case action_type::forward: {
                j = {{"left_speed", 1}, {"right_speed", 1}};
            } break;
            case action_type::backward: {
                j = {{"left_speed", -1}, {"right_speed", -1}};
            } break;
            case action_type::left: {
                j = {{"left_speed", 1}, {"right_speed", -1}};
            } break;
            case action_type::right: {
                j = {{"left_speed", -1}, {"right_speed", 1}};
            } break;
            case action_type::stop: {
                j = {{"left_speed", 0}, {"right_speed", 0}};
            } break;
        }
        return j.dump(); 
    }
};


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
    double previous_direction__;
    double previous_filtered__;


};
#endif
