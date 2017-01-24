#include "slammer.hpp"

slammer::slammer()
: map__()
{
    start__ = ros::Time::now().toSec();
    time__ = start__;
    sensor__ = mrpt::poses::CPose3D(0.1, 0, 0.1, 
                                    mrpt::utils::DEG2RAD(7), // yaw
                                    0, 
                                    mrpt::utils::DEG2RAD(180)); // roll

    lidar_obs__->rightToLeft = false;
    lidar_obs__->aperture = 2 * M_PIf;
    lidar_obs__->maxRange = 6.0;
    lidar_obs__->stdError = 0.010f;

}    

void slammer::read_lazors(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    int count = scan->scan_time / scan->time_increment;
    lidar_obs__->setSensorPose(sensor__);
    lidar_obs__->timestamp = mrpt::system::now();
    lidar_obs__->scan.resize(count);
    lidar_obs__->validRange.resize(count);

    for(int i = 0; i < count; i++) {
        lidar_obs__->scan[i] = (float)scan->ranges[i];
        lidar_obs__->validRange[i] = (char)(scan->intensities[i]);
    }
}

void slammer::read_motors(const std_msgs::String::ConstPtr& motors_data)
{
    std::string data = motors_data->data.c_str();
    nlohmann::json json_data;
    json_data = nlohmann::json::parse(data);
    right_speed__ = json_data["right_speed"];
    left_speed__ = json_data["left_speed"];

    distance__ = (right_speed__ + left_speed__)/2;

}

void slammer::update_map()
{
    auto pose3d = std::make_shared<mrpt::poses::CPose3D>(calculate_pose()); 
    map__.insertObservation(lidar_obs__.get(), pose3d.get());
    if ((ros::Time::now().toSec() - start__) > 10) {
        std::cout << "saving map" << std::endl;
        map__.saveMetricMapRepresentationToFile("image_map");
        start__ = ros::Time::now().toSec();
    }

}

mrpt::poses::CPose3D slammer::calculate_pose()
{
    vector euler = imu__.read();
    auto x = distance__ * sin(mrpt::utils::DEG2RAD(euler(1))) * cos(mrpt::utils::DEG2RAD(euler(2)));
    auto y = distance__ * sin(mrpt::utils::DEG2RAD(euler(1))) * sin(mrpt::utils::DEG2RAD(euler(2)));
    auto z = distance__ * cos(mrpt::utils::DEG2RAD(euler(2)));

    return mrpt::poses::CPose3D(x, y, z, euler(2), euler(1), euler(0)); 
}
