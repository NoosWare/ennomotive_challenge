#include "navigator.hpp"

navigator::navigator(ros::NodeHandle & node)
: reactivenav__(), iniFile__("navigator.ini"), log__() 
{
    poser__ = node.advertise<std_msgs::String>("navigation", 50);
    //subs__ = node.subscribe("motors", 30, actual_action);
    reactivenav__.initialize(iniFile__);
    angle_samples__ = {0, 0, 0, 0, 0};
    counter__ = 0;
}

void navigator::navigate(mrpt::poses::CPose3D robotpose,
                         std::vector<float> laser_data)
{
    
    double direction_result, speed_result;
    auto target = calculate_target(robotpose); 
    reactivenav__.navigate(
                            target,
                            laser_data,
                            (double)0.1488, //maxspeed pseudometer2= meter2 + (rad A· r)2
                            direction_result, //out
                            speed_result, //out
                            log__  //log
                          );

    angle_samples__.erase(angle_samples__.begin());
    angle_samples__.push_back(direction_result);
    counter__++;

    if (counter__ == 4) {
        double average = 0;
        for (auto sample : angle_samples__) {
            average += sample; 
        }
        average /= 5;
        navigation_result(robotpose, average);
        counter__ = 0;
    }
    //navigation_result(robotpose, direction_result);
    std::cout << target.x << " " << target.y  << " " << RAD2DEG(direction_result) << std::endl;
}

mrpt::math::TPoint2D navigator::calculate_target(mrpt::poses::CPose3D robopose)
{
    double new_x = robopose.x() + (0.02) * cos(robopose.yaw()); //target for distance in 1 sec 
    double new_y = robopose.y() + (0.02) * sin(robopose.yaw());
    return mrpt::math::TPoint2D(new_x, new_y);
}

void navigator::navigation_result(
                                     mrpt::poses::CPose3D robotpose, 
                                     double angle
                                  ) 
{
    std_msgs::String msg;
    nlohmann::json json = {
                            {"angle", RAD2DEG(angle)}
                          };
    msg.data = json.dump();
    poser__.publish(msg);
 
    //std::cout << msg.data << std::endl;
}

/*
action_type navigator::actual_action(const std_msgs::String::ConstPtr& msg)
{
    
    std::string message = msg.data.c_str();
    nlohmann::json json = nlohmann::json::parse(message);
    int right_speed = json["right_speed"];
    int left_speed = json["left_speed"];

}*/
