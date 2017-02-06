#include "ros/ros.h"
#include "robot_action.hpp"
#include "robot_state.hpp"
#include "action_factory.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle node;
    auto pub = node.advertise<std_msgs::String>("motors", 1000);
    
    auto at_fact = action_factory(node, "samples");
    //auto st_fact = state_factory(node);

    ros::Rate loop_rate(10);
    int count = 0;

    while (ros::ok()) 
    {
        count++;
        if (count > 800) {
            break;
        }

        auto atype = at_fact.max_action_scored();
        std::cout << "action: " << action_string()(atype) << std::endl;

        // training sample for ANN        
        //at_fact.add_sample();

        //auto st = st_fact.get_state();
        //st.print();

        robot_action a_t(pub, atype);
        a_t.run();

        ros::spinOnce();
        loop_rate.sleep();
    }

    // stop the robot
    robot_action a_t(pub, action_type::stop);
    a_t.run();
    
    // train the ANN
    //at_fact.train_ann();
    std::cout << "main node exiting" << std::endl;

    return 0;
}
