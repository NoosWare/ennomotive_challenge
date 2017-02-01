#include "slammer.hpp"

slammer::slammer(ros::NodeHandle & node)
: iniFile__("icp_config.ini")
{
    poser__ = node.advertise<std_msgs::String>("coordinates", 100);
    collision__ = node.advertise<std_msgs::String>("collision", 100);
    free_way__ = node.advertise<std_msgs::String>("distance", 100);

    start__ = ros::Time::now().toSec();
    timer__ = start__;
    sensor__ = mrpt::poses::CPose3D(0.1, 0, 0.1, 
                                    mrpt::utils::DEG2RAD(7), // yaw
                                    0, 
                                    mrpt::utils::DEG2RAD(180)); // roll

    //if (grid__.loadFromBitmapFile("grid_map.png", 0.04)) {
    //    std::cout << "loaded grid map" << std::endl;
    //}
    builder__.options.verbose = false;
    builder__.ICP_options.loadFromConfigFile(iniFile__, "MappingApplication");
    builder__.ICP_params.loadFromConfigFile(iniFile__, "ICP");
    builder__.initialize();
    //builder__.enableMapUpdating();
}    

void slammer::read_lazors(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    auto obs = std::make_shared<mrpt::slam::CObservation2DRangeScan>();
    int count = scan->scan_time / scan->time_increment;
    obs->timestamp = mrpt::system::now();
    obs->scan.resize(count);
    obs->validRange.resize(count);
    obs->rightToLeft = false;
    obs->aperture = 2 * M_PIf;
    obs->maxRange = 6.0;
    obs->stdError = 0.010f;
    obs->setSensorPose(sensor__);

    std::vector<float> angles;

    for (int i = 0; i < count; i++) {
        obs->scan[i] = scan->ranges[i];
        obs->validRange[i] = (char)(scan->intensities[i]);
        if (obs->scan[i] < 0.20) {
            angles.push_back(i);
        }        
    }

    collision_angles(angles);
    avg_distance(obs->scan, count, obs->validRange);

    auto lazor = mrpt::slam::CObservation2DRangeScanPtr();
    lazor.setFromPointerDoNotFreeAtDtor(obs.get());
    builder__.processObservation(lazor);

    mrpt::poses::CPose3D robotpose;
    builder__.getCurrentPoseEstimation()->getMean(robotpose);
    auto pose = std::make_shared<mrpt::poses::CPose3D>(robotpose);

    grid__.insertObservation(obs.get(), pose.get());
    publish_pose(*pose.get());

    if ((ros::Time::now().toSec() - start__) > 10) {
        std::cout << "saving maps" << std::endl;
        serialize();
    }
}

void slammer::publish_pose(mrpt::poses::CPose3D pose)
{
    std_msgs::String msg;
    nlohmann::json json = {{"x", pose.x()},
                           {"y", pose.y()},
                           {"theta", pose.yaw()}
                          };
    msg.data = json.dump();
    poser__.publish(msg);
}

void slammer::collision_angles( std::vector<float> lidar_angles) 
{
    bool front, l_front, r_front, l_back, r_back, back;
    front = l_front = r_front = l_back = r_back = back = false;

    for (const auto & angle : lidar_angles) {
        if (angle <= 30 || angle >= 330) {
            front = true;
        }     
        else if (angle > 30 && angle <= 90) {
            l_front = true;
        }
        else if (angle > 90 && angle <= 150) {
            l_back = true;
        }
        else if (angle > 150 && angle <= 210) {
            back = true;
        }
        else if (angle > 210 && angle <= 270) {
            r_back = true;
        }
        else if (angle > 270 && angle < 330) {
            r_front = true;
        }
    }
    std_msgs::String msg;
    nlohmann::json json = {{"front", front},
                           {"r_front", r_front},
                           {"l_front", l_front},
                           {"l_back", l_back},
                           {"r_back", r_back},
                           {"back", back}
                          };
    msg.data = json.dump();
    collision__.publish(msg);

}

void slammer::avg_distance( 
                            std::vector<float> lidar_angles, 
                            int counter,
                            std::vector<char> valid_data
                          )
{
    std::vector<float> lidar(lidar_angles);
    for (unsigned int i = 0; i < lidar_angles.size(); i++) {
        if (std::isinf(lidar_angles[i])) {
            lidar[i] = 0.0f;
        }
        else if (valid_data[i] == 0) {
            lidar[i] = 0.0f;
        }
        else {
            lidar[i] = lidar_angles[i];
        }
    }

    float avg_front, avg_right, avg_left, avg_back;
    avg_front = avg_right = avg_left = avg_back = 0.0f;
    std::vector<float> front, right, left, back;
    //front
    std::copy(lidar.begin(), lidar.begin() + 44,
              std::back_inserter(front));
    std::copy(lidar.begin() + 315, lidar.end(),
              std::back_inserter(front));
    //left
    std::copy(lidar.begin() + 45, lidar.begin() + 134,
              std::back_inserter(left));
    //back
    std::copy(lidar.begin() + 135, lidar.begin() + 224,
              std::back_inserter(back));
    //right
    std::copy(lidar.begin() + 225, lidar.begin() + 314,
              std::back_inserter(right));

    ///Average
    std::vector<float> avg = { 0, 0, 0, 0};
    std::vector<std::vector<float>> data = { front, left, right, back};
    int count = 0;
    
    // score for forward
    float front_score  = std::accumulate(front.begin(), front.end(), 0.0f) / front.size();
    float back_score   = std::accumulate(back.begin(), back.end(), 0.0f) / back.size();
    float left_score   = std::accumulate(left.begin(), left.end(), 0.0f) / left.size();
    float right_score  = std::accumulate(right.begin(), right.end(), 0.0f) / right.size();

    std_msgs::String msg;
    nlohmann::json json = {{"front", front_score},
                           {"right", right_score},
                           {"left",  left_score},
                           {"back",  back_score}
                          };
    msg.data = json.dump();
    free_way__.publish(msg);
    //std::cout << front_score << " " << right_score << " " << back_score << " " << left_score  << std::endl;
}

void slammer::serialize()
{
    builder__.saveCurrentEstimationToImage("icp_map", true);
    grid__.saveAsBitmapFile("grid_map.png");
    start__ = ros::Time::now().toSec();

    /*
    std::ofstream coords("coordinates");
    boost::archive::text_oarchive oa(coords);
    oa & visited__;
    */
}
