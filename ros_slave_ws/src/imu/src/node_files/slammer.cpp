#include "slammer.hpp"

slammer::slammer()
: map__(),
  imu__()
{
    //position__ = vector(0, 0, 0);
    start__ = ros::Time::now().toSec();
    timer__ = start__;
    sensor__ = mrpt::poses::CPose3D(0.1, 0, 0.1, 
                                    mrpt::utils::DEG2RAD(7), // yaw
                                    0, 
                                    mrpt::utils::DEG2RAD(180)); // roll

    lidar_obs__.rightToLeft = false;
    lidar_obs__.aperture = 2 * M_PIf;
    lidar_obs__.maxRange = 6.0;
    lidar_obs__.stdError = 0.010f;
    lidar_obs__.setSensorPose(sensor__);
}    

void slammer::read_lazors(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    int count = scan->scan_time / scan->time_increment;
    lidar_obs__.timestamp = mrpt::system::now();
    lidar_obs__.scan.resize(count);
    lidar_obs__.validRange.resize(count);
    for(int i = 0; i < count; i++) {
        lidar_obs__.scan[i] = (float)scan->ranges[i];
        lidar_obs__.validRange[i] = (char)(scan->intensities[i]);
    }
    //update_map();
}

void slammer::read_motors(const std_msgs::String::ConstPtr& motors_data)
{
    dt__ = ros::Time::now().toSec() - timer__ ; 
    std::string data = motors_data->data.c_str();
    nlohmann::json json_data;
    json_data = nlohmann::json::parse(data);
    right_speed__ = json_data["right_speed"];
    left_speed__ = json_data["left_speed"];
    right_speed__ = (right_speed__ / 3) * (0.3333 * dt__);
    left_speed__ = (left_speed__ / 3) * (0.3333 * dt__);
    radius__ = (right_speed__ + left_speed__) / 2;
    distance__ += radius__;
    timer__ = ros::Time::now().toSec();

    //Update map with new lidar observation
    update_map();
}

void slammer::update_map()
{
    auto pose3d = std::make_shared<mrpt::poses::CPose3D>(calculate_pose2d()); 
    auto lazor_obs = std::make_shared<mrpt::slam::CObservation2DRangeScan>(lidar_obs__);
    map__.insertObservation(lazor_obs.get(), pose3d.get());
    if ((ros::Time::now().toSec() - start__) > 10) {
        std::cout << "saving map" << std::endl;
        map__.saveMetricMapRepresentationToFile("image_map");
        start__ = ros::Time::now().toSec();
    }
}

mrpt::poses::CPose3D slammer::calculate_pose2d()
{
    float TWO_PI = 2 * M_PI;
    euler_angles euler = imu__.read();
    /*
    auto theta = euler.yaw - theta__;
    theta__ = euler.yaw;
     
    theta -= (float)((int)(theta / TWO_PI)) * TWO_PI;
    if (theta < -M_PI) { 
        theta += TWO_PI; 
    }
    else { 
        if (theta > M_PI) theta -= TWO_PI; 
    }
    std::cout << "Theta: " << theta << std::endl;

    x__ += radius__  * sin(theta);
    y__ += radius__  * cos(theta);
    */
    /*
    std::cout// << " r: " << radius__
              << " theta: " << euler.yaw
              << " x: " << x__
              << " y: " << y__ 
              << " dist: " << distance__
              << std::endl;
    */
    return mrpt::poses::CPose3D(x__,
                                y__,
                                0,
                                0,
                                0, 
                                0); 
                                /*
                                euler.yaw, 
                                euler.pitch, 
                                euler.roll); 
                                */
}

/*
mrpt::poses::CPose3D slammer::calculate_pose()
{
    euler_angles euler = imu__.read();

    auto x = radius__ * sin(euler.pitch) * cos(euler.yaw);
    auto y = radius__ * sin(euler.pitch) * sin(euler.yaw);
    auto z = radius__ * cos(euler.yaw);
    position__[0] += x;
    position__[1] += y;
    position__[2] += z;

    std::cout// << " r: " << radius__
              << " x: " << position__[0] 
              << " y: " << position__[1] 
              << " z: " << position__[2] 
              << std::endl;

    return mrpt::poses::CPose3D(position__[0],
                                position__[1],
                                position__[2], 
                                euler.pitch, 
                                euler.yaw, 
                                euler.roll); 
}
*/
