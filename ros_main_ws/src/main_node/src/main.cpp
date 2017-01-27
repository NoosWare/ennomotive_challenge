#include "ros/ros.h"
#include "robot_action.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle node;
    auto pub = node.advertise<std_msgs::String>("motors", 1000);

    // TEST: run forward for 3 seconds
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()){

        robot_action a_t(pub, action_type::forward);
        a_t.run();

        ros::spinOnce();
        loop_rate.sleep();
        count++;
        if (count > 30) {
            break;
        }
    }
    robot_action a_t(pub, action_type::stop);
    a_t.run();

    return 0;
}
