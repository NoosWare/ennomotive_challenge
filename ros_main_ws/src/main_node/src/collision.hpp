#ifndef COLLISION_HPP
#define COLLISION_HPP
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include "robot_action.hpp"

class collision
{
public:

    /// @brief construct a navigation monitor
    /// @param callback will receive updtes as they are read
    collision(ros::NodeHandle & node)
    {
        nav_sub__ = node.subscribe("collision", 
                                    1000, 
                                    &collision::read_data, 
                                    this
                                  );

        dist_sub__ = node.subscribe("distance", 
                                     1000, 
                                     &collision::read_distance, 
                                     this
                                   );

        actions__ = {{action_type::forward, false},
                     {action_type::left, false},
                     {action_type::right, false},
                     {action_type::backward, false}};
    }

    /// get detected collisions
    std::map<action_type, bool> collisions() const
    {
        return actions__;
    }

    /// get actions scores from LIDAR
    std::map<action_type, float> action_scores() const
    {
        return scores__;
    }

    /// parse JSON of imminent collisions
    void read_data(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        actions__[action_type::forward] = obj["front"];
        actions__[action_type::left]    = (obj["l_front"] || obj["l_back"]);
        actions__[action_type::right]   = (obj["r_front"] || obj["r_back"]);
        actions__[action_type::backward]= obj["back"];
    }

    void read_distance(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        scores__[action_type::forward] = obj["front"];
        scores__[action_type::left]    = obj["left"];
        scores__[action_type::right]   = obj["right"];
        scores__[action_type::backward]= obj["back"];
    }

protected:
    // available actions (not a collision)
    std::map<action_type, bool> actions__;
    std::map<action_type, float> scores__;
    ros::Subscriber nav_sub__;
    ros::Subscriber dist_sub__;
};
#endif
