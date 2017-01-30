#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include "robot_action.hpp"

class navigation
{
public:

    /// @brief construct a navigation monitor
    /// @param callback will receive updtes as they are read
    navigation(ros::NodeHandle & node)
    {
        nav_sub__ = node.subscribe("navigation", 
                                    1000, 
                                    &navigation::read_angle, 
                                    this
                                  );
    }

    /// parse JSON of {"angle":.., }
    void read_angle(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        angle__ = obj["angle"];
    }

    // @brief get last(newest)
    action_type get_latest() const
    {
        return fuzzy_angle(angle__);
    }

    // @brief get all
    std::vector<action_type> get_all() const
    {}

    // @brief get most frequent in the rolling window
    action_type get_most_frequent() const
    {}

private:

    action_type fuzzy_angle(double theta) const
    {
        if (theta > -90 && theta < 90) {
            printf("nav angle: %f %s \r\n", angle__, "forward");
            return action_type::forward;
        }
        else if (theta > 90 && theta < 170) {
            printf("nav angle: %f %s \r\n", angle__, "right");
            return action_type::right;
        }
        else if (theta < -90 && theta > -170) {
            printf("nav angle: %f %s \r\n", angle__, "left");
            return action_type::left;
        }
        else if (theta < -170 || theta > 170) {
            printf("nav angle: %f %s \r\n", angle__, "back");
            return action_type::backward;
        }
        else {
            throw std::runtime_error("unknown angle or condition");
        }
    }

protected:
    
    double angle__ = 0;
    //std::deque <double> angles__;
    ros::Subscriber nav_sub__;
};
#endif
