#include "ros/ros.h"
#include "imu/imu_broadcaster.hpp"

#include <sstream>
#include <fstream>
#include <memory>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle n;


    //
    std::ofstream data("accel_and_3dpose.dat", std::ios::app);
    //std::ofstream data1("angular_valocity.dat", std::ios::app);
    //std::ofstream data2("magnetic_field.dat", std::ios::app);
    // TODO: run for one minute, and find how much the drift is in each acceleration axis
    //       and how much drift there is in each gyroscope axis
    imu_broadcaster imu;
    auto secs = ros::Time::now().toSec();

    while(ros::ok()) {
        
        std::tuple<vector, vector, vector, quaternion> imu_data = imu.read();
        vector acc = std::get<0>(imu_data);
        vector fil = std::get<1>(imu_data);
        vector pos = std::get<2>(imu_data);
        quaternion quat = std::get<3>(imu_data);
        mrpt::poses::CPose3D pose = imu.convert_to_3dpose(pos, quat);
        
        auto secs_after = ros::Time::now().toSec() - secs;
        data << secs_after << " " << acc.x() << " " << acc.y() << " " << acc.z() << "  " << fil.x() << " " << fil.y() << " " << fil.z() << "  " <<  pose.x() << " " << pose.y() << " " << pose.z() << std::endl;
        /*
        data1 << secs_after << " " << ang.x() << " " << ang.y() << " " << ang.z() << "\r\n";
        data2 << secs_after << " " << mag.x() << " " << mag.y() << " " << mag.z() << "\r\n";
        */

    }
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
        data << secs_after << " " << tmp.second.x() << " " << tmp.second.y() << " " << tmp.second.z() << "\r\n";
        pose = (mrpt::math::CQuaternionDouble(tmp.first.w(),
                                             tmp.first.x(),
                                             tmp.first.y(),
                                             tmp.first.z()),
                                tmp.second(0),
                                tmp.second(1),
                                tmp.second(2));
        */

    ros::spin();
    data.close();
    //data1.close();
    //data2.close();

    return 0;
}
