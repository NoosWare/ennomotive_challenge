#include "slammer.hpp"

slammer::slammer(ros::NodeHandle & node)
: planner(), navigator(node), iniFile__("icp_config.ini")
{
    poser__ = node.advertise<std_msgs::String>("coordinates", 30);

    start__ = ros::Time::now().toSec();
    timer__ = start__;
    sensor__ = mrpt::poses::CPose3D(0.1, 0, 0.1, 
                                    mrpt::utils::DEG2RAD(7), // yaw
                                    0, 
                                    mrpt::utils::DEG2RAD(180)); // roll

    builder__.options.verbose = false;
    builder__.ICP_options.loadFromConfigFile(iniFile__, "MappingApplication");
    builder__.ICP_params.loadFromConfigFile(iniFile__, "ICP");
    builder__.initialize();
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

    for (int i = 0; i < count; i++) {
        obs->scan[i] = scan->ranges[i];
        obs->validRange[i] = (char)(scan->intensities[i]);
    }

    auto lazor = mrpt::slam::CObservation2DRangeScanPtr();
    lazor.setFromPointerDoNotFreeAtDtor(obs.get());
    builder__.processObservation(lazor);

    mrpt::poses::CPose3D robotpose;
    builder__.getCurrentPoseEstimation()->getMean(robotpose);
    auto pose = std::make_shared<mrpt::poses::CPose3D>(robotpose);
    grid__.insertObservation(obs.get(), pose.get());

    // TODO: move loop out of here, this callbacle by a topic `planner`
    //calculate_path();
    if ((ros::Time::now().toSec() - timer__) > 1) {
        navigator::navigate(robotpose, obs->scan);
        timer__ = ros::Time::now().toSec();
    }

    if ((ros::Time::now().toSec() - start__) > 10) {
        std::cout << "saving map" << std::endl;
        builder__.saveCurrentEstimationToImage("icp_map", true);
        grid__.saveAsBitmapFile("grid_map.png");
        start__ = ros::Time::now().toSec();
    }
}

void slammer::calculate_path()
{
    mrpt::poses::CPose3D robotpose;
    planner::path_planner(grid__, robotpose);
}

void slammer::get_pose()
{
    std_msgs::String msg;
    mrpt::poses::CPose3D robotpose;
    builder__.getCurrentPoseEstimation()->getMean(robotpose);
    nlohmann::json json = {{"x", robotpose.x()},
                           {"y", robotpose.y()},
                           {"theta", robotpose.yaw()}
                          };
    msg.data = json.dump();
    poser__.publish(msg);
}
