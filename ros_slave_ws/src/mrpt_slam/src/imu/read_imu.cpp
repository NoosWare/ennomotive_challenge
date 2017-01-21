#include "read_imu.hpp"

read_imu::read_imu(imu_broadcaster & broadcaster)
: imu__(broadcaster)
{}

mrpt::poses::CPose3D read_imu::read_pos_imu() 
{
    std::pair<quaternion, vector> pose = imu__.read();
    return mrpt::poses::CPose3D(mrpt::math::CQuaternionDouble((double)pose.first.w(),
                                                             (double)pose.first.x(),
                                                             (double)pose.first.y(),
                                                             (double)pose.first.z()),
                                pose.second(0),
                                pose.second(1),
                                pose.second(2));
}
