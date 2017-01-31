#include "ros/ros.h"
#include "robot_action.hpp"
#include "robot_state.hpp"
#include "action_factory.hpp"
#include "navigation.hpp"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle node;
    auto pub = node.advertise<std_msgs::String>("motors", 1000);
    
    auto log = logger("training.data");
    auto a_factory = action_factory(node);
    auto nav = navigation(node);

    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) 
    {
        count++;
        if (count > 300) {
            break;
        }

        // create an action
        auto atype = a_factory.max_action_scored();
        std::cout << "action: " << action_string()(atype) << std::endl;
        robot_action a_t(pub, atype);
        a_t.run();

        //factory.get_state().print();
        ros::spinOnce();
        loop_rate.sleep();
    }

    // stop the robot
    robot_action a_t(pub, action_type::stop);
    a_t.run();

    return 0;
}
