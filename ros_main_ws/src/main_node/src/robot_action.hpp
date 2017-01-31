#ifndef ROBOT_ACTION_HPP
#define ROBOT_ACTION_HPP
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "json.hpp"
#include "relearn.hpp"

/// action type (C++1y enum)
enum class action_type
{ 
    forward  = 1,
    backward = 2,
    left = 3,
    right = 4,
    stop  = 0
};

struct action_string
{
    std::string operator()(const action_type action)
    {
        switch (action) {
            case action_type::forward: {
                return "forward";        
            } break;
            case action_type::backward: {
                return "backward";        
            } break;
            case action_type::left: {
                return "left";
            } break;
            case action_type::right: {
                return "right";
            } break;
            case action_type::stop: {
                return "stop";
            } break;
        }
    }
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
                j = {{"left_speed", -1}, {"right_speed", 1}};
            } break;
            case action_type::right: {
                j = {{"left_speed", 1}, {"right_speed", -1}};
            } break;
            case action_type::stop: {
                j = {{"left_speed", 0}, {"right_speed", 0}};
            } break;
        }
        return j.dump(); 
    }
};

/**
 * @class robot_action
 **/
class robot_action
{
public:

    /// public member `move`
    action_type move;

    robot_action() = delete;

    robot_action(const robot_action &) = delete;

    robot_action(const ros::Publisher & publisher, action_type arg)
    : pub__(publisher), move(arg)
    {}

    bool operator==(const robot_action & arg) const
    {
        return (this->move == arg.move);
    }

    /// @brief run this action (e.g., command motors)
    void run()
    {
        std::string json = action_interpreter()(move);
        std_msgs::String msg;
        msg.data = json.c_str();
        pub__.publish(msg); 
    }

private:
    // publisher to ROS/motors
    const ros::Publisher & pub__;
};

namespace std 
{
template<> struct hash<action_type>
{
    size_t operator()(const action_type & arg) const 
    {
        return std::hash<std::size_t>()(static_cast<std::size_t>(arg));
    }
};
template <> struct hash<robot_action>
{
    std::size_t operator()(robot_action const& arg) const
    {
        std::size_t seed = 0;
        relearn::hash_combine<action_type>(seed, arg.move);
        return seed;
    }
};
}

#endif
