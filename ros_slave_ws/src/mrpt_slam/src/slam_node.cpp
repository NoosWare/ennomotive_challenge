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
    : map__(),
      data__("data.dat", std::ios::app)
    {
        start__ = ros::Time::now().toSec();
        time__ = start__;
        sensor__ = mrpt::poses::CPose3D(0.1, 0, 0.1, 
                                        mrpt::utils::DEG2RAD(7), // yaw
                                        0, 
                                        mrpt::utils::DEG2RAD(180)); // roll
    } 

    ~slam_sub()
    {
        data__.close();
    }

    /// callback from laser data 
    void run(const sensor_msgs::LaserScan::ConstPtr & scan)
    {
        //Lidar
        int count = scan->scan_time / scan->time_increment;
        auto obs = std::make_shared<mrpt::slam::CObservation2DRangeScan>();
        obs->setSensorPose(sensor__);
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

        //IMU
        //std::tuple<vector, vector, vector, quaternion> imu_data = imu__.read();
        //vector acc = std::get<0>(imu_data);
        //vector fil = std::get<1>(imu_data);
        //vector pos = std::get<2>(imu_data);
        //quaternion quat = std::get<3>(imu_data);
        //pos__ += vector( 0.033, 0, 0);
        x__ += 0.33;
        auto pose = std::make_shared<mrpt::slam::CPose3D>(x__, 0, 0, 0, 0, 0);
        
        /*
        auto secs_after = ros::Time::now().toSec() - time__;
        data__ << secs_after << " " << acc.x() << " " 
               << acc.y() << " " << acc.z() << "  " 
               << fil.x() << " " << fil.y() << " " << fil.z() << "  " 
               <<  pose->x() << " " << pose->y() << " " << pose->z() << std::endl;
        */

        ///MAP
        map__.insertObservation(obs.get(), pose.get()); 
        
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
    double time__;
    imu_broadcaster imu__;
    std::ofstream data__;
    //vector pos__;
    int x__ = 0;
    mrpt::poses::CPose3D sensor__;
};



int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);
    slam_sub slammer;

    // "/scan" is for RPLIDAR messages
    // "motors" is for motor commands
    //
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &slam_sub::run, &slammer);


    ros::spin();
    loop_rate.sleep();

    return 0;
}
