#ifndef ACTION_FACTORY_HPP
#define ACTION_FACTORY_HPP
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robot_action.hpp"
#include "robot_state.hpp"
#include "collision.hpp"
#include "sensor_fusion.hpp"
#include <random>
#include <chrono>

/// @brief
/// @note calls @class collision
class action_factory : public collision, public sensor_fusion
{
public:

    action_factory(ros::NodeHandle & node, std::string logfile)
    : collision(node), sensor_fusion(node, logfile),
      gen__(std::chrono::system_clock::now().time_since_epoch().count())
    {}

    action_type max_action_scored()
    {
        std::map<action_type, float> scores = collision::scores__;
        scores.erase(action_type::backward);
        scores[last__] += 0.3;
        auto x = std::max_element(scores.begin(), scores.end(),
                                 [](const auto & p1, const auto & p2) {
                                     return p1.second < p2.second; 
                                 });
        last__ = x->first;
        return x->first;
    }

private:

    std::mt19937 gen__;
    action_type last__ = action_type::stop;
};
#endif
