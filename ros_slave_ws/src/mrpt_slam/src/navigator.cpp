#include "navigator.hpp"

navigator::navigator(ros::NodeHandle & node)
: reactivenav__(), iniFile__("navigator.ini"), log__() 
{
    poser__ = node.advertise<std_msgs::String>("navigation", 50);
    //subs__ = node.subscribe("motors", 30, actual_action);
    reactivenav__.initialize(iniFile__);
    previous_direction__ = 0;
    previous_filtered__ = 0;
}

void navigator::navigate(mrpt::poses::CPose3D robotpose,
                         std::vector<float> laser_data)
{
    
    double direction_result, speed_result;
    //std::vector<double> laser_data_double(laser_data.begin(), laser_data.end());
    auto target = calculate_target(robotpose); 
    reactivenav__.navigate(
                            target,
                            laser_data,
                            (double)0.2, //maxspeed pseudometer2= meter2 + (rad A· r)2
                            direction_result, //out
                            speed_result, //out
                            log__  //log
                          );
    //navigation_result(robotpose, direction_result);

    //double angle_desire = dc_block_filter(direction_result, previous_direction__, previous_filtered__);
    previous_direction__ = direction_result;
    previous_filtered__ = angle_desire;

    std::cout << " " << robotpose.x() << " "<< robotpose.y() << " " << target.x << " " << target.y << " " << RAD2DEG(robotpose.yaw()) << " " << RAD2DEG(direction_result) << std::endl;
}

mrpt::math::TPoint2D navigator::calculate_target(mrpt::poses::CPose3D robopose)
{
    double new_x = robopose.x() + (0.01488 * 5) * cos(robopose.yaw()); //target for distance in 1 sec 
    double new_y = robopose.y() + (0.01488 * 5) * sin(robopose.yaw());
    return mrpt::math::TPoint2D(new_x, new_y);
}

void navigator::navigation_result(
                                     mrpt::poses::CPose3D robotpose, 
                                     double angle
                                  ) 
{
    std_msgs::String msg;
    nlohmann::json json = { {"origin" ,{{"x", robotpose.x()},
                                       {"y", robotpose.y()},
                                       {"theta", robotpose.yaw()}
                                       }},
                            {"result", angle}
                          };
    msg.data = json.dump();
    poser__.publish(msg);
 
    std::cout << msg.data << std::endl;
}

/*
action_type navigator::actual_action(const std_msgs::String::ConstPtr& msg)
{
    
    std::string message = msg.data.c_str();
    nlohmann::json json = nlohmann::json::parse(message);
    int right_speed = json["right_speed"];
    int left_speed = json["left_speed"];

}*/

double navigator::dc_block_filter(
                                         double raw_data,
                                         double previous_raw_data,
                                         double previous_filtered_data
                                       )
{
    return raw_data - previous_raw_data + 0.995 * previous_filtered_data;
}
