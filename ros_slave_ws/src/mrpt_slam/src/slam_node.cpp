#include <mrpt/poses/CPose3D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservationIMU.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include "imu/imu_broadcaster.hpp"

#include <sstream>
#include <fstream>
#include <memory>

#define RAD2DEG(x) ((x)*180./M_PI)
#define M_PIf 3.14159265358979f

class slam_sub
{
public:

    slam_sub()
    : map__()
    {
        start__ = ros::Time::now().toSec();
    } 

    /// callback from laser data 
    void run(const sensor_msgs::LaserScan::ConstPtr & scan)
    {
        int count = scan->scan_time / scan->time_increment;
        auto obs = std::make_shared<mrpt::slam::CObservation2DRangeScan>();
        obs->timestamp = mrpt::system::now();
        obs->scan.resize(count);
        obs->validRange.resize(count);

        for(int i = 0; i < count; i++) {
            //float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
            //ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
            obs->scan[i] = (float)scan->ranges[i];
            obs->validRange[i] = (char)(scan->intensities[i]);
            //std::cout<< obs->scan[i] << " " << scan->intensities[i] << std::endl;
        }
        obs->rightToLeft = false;
        obs->aperture = 2 * M_PIf;
        obs->maxRange = 6.0;
        obs->stdError = 0.010f;
        map__.insertObservation(obs.get(), NULL);

        auto obs_imu = std::make_shared<mrpt::slam::CObservationIMU>();
        obs_imu->timestamp = mrpt::system::now();
        imu__.read(obs_imu);
        map__.insertObservation(obs_imu.get(), NULL);

        // save the map every 10 seconds
        if ((ros::Time::now().toSec() - start__) > 10) {
            std::cout << "saving map" << std::endl;
            //map__.save2D_to_text_file("mrpt_2D_slam_map.txt");
            map__.saveMetricMapRepresentationToFile("image_map.png");
            //map__.saveToPlyFile("ply_map.ply");
            start__ = ros::Time::now().toSec();
        }
    }

private: 
    mrpt::slam::COccupancyGridMap2D  map__;
    double start__;
    imu_broadcaster imu__;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle n;

    slam_sub slammer;    
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &slam_sub::run, &slammer);
    ros::spin();

    //
    //std::ofstream data("acceleration.dat", std::ios::app);
    // TODO: run for one minute, and find how much the drift is in each acceleration axis
    //       and how much drift there is in each gyroscope axis
        /*
        // loop 10 times in 100ms
        std::pair<quaternion, vector> tmp;
        for (unsigned int i = 0; i < 10; i++) {
            auto pair = broadcaster.read();
            tmp.second += pair.second;
            tmp.first = pair.first;
            usleep(10000);
        }
        tmp.second /= 10.f;
        double secs_after = ros::Time::now().toSec() - secs;
        data << secs_after << " " << tmp.second.x() << " " << tmp.second.y() << " " << tmp.second.z() << "\r\n";
        pose = (mrpt::math::CQuaternionDouble(tmp.first.w(),
                                             tmp.first.x(),
                                             tmp.first.y(),
                                             tmp.first.z()),
                                tmp.second(0),
                                tmp.second(1),
                                tmp.second(2));
        */
    //data.close();

    return 0;
}
