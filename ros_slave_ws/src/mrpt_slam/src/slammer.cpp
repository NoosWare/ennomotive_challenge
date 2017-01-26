#include "slammer.hpp"

std::once_flag flag1;

slammer::slammer()
: map__(), iniFile__("icp_slam.ini"), icp_map__()
{
    start__ = ros::Time::now().toSec();
    timer__ = start__;
    sensor__ = mrpt::poses::CPose3D(0.1, 0, 0.1, 
                                    mrpt::utils::DEG2RAD(7), // yaw
                                    0, 
                                    mrpt::utils::DEG2RAD(180)); // roll

    icp_map__.ICP_options.loadFromConfigFile(iniFile__, "MappingApplication");
    icp_map__.ICP_params.loadFromConfigFile(iniFile__, "ICP");
    //icp_map__.ICP_params.dumpToConsole();
    //icp_map__.ICP_options.dumpToConsole();
    icp_map__.initialize();
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
        obs->scan[i] = (float)scan->ranges[i];
        obs->validRange[i] = (char)(scan->intensities[i]);
    }

    auto lazor = mrpt::slam::CObservation2DRangeScanPtr();
    lazor.setFromPointerDoNotFreeAtDtor(obs.get());
    mrpt::poses::CPose3D robotPose;
    icp_map__.processObservation(lazor);
    icp_map__.getCurrentPoseEstimation()->getMean(robotPose);

    if ((ros::Time::now().toSec() - start__) > 10) {
        std::cout << "saving map" << std::endl;
        //map__.saveMetricMapRepresentationToFile("image_map");
        icp_map__.saveCurrentEstimationToImage("icp_map", true);
        start__ = ros::Time::now().toSec();
    }
}
