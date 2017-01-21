#ifndef READ_IMU_HPP
#define READ_IMU_HPP

#include <mrpt/poses/CPose3D.h>
#include <mrpt/math/CQuaternion.h>
#include "vector.hpp"
#include "imu_broadcaster.hpp"
#include "includes.ihh"

/**
 * @brief class to read IMU values and transform to MRPT CPose3D
 * @version 0.1.0
 * @date 18-01-2017
 * @author Maria Ramos <m.ramos@ortelio.co.uk>
 */

class read_imu
{
public:

    /// @brief constructor
    /// @param imu_broadcaster  
    read_imu(imu_broadcaster & broadcaster);

    /// @return CPose3D of the position and orientation of the robot
    mrpt::poses::CPose3D read_pos_imu();

public:
    imu_broadcaster & imu__;
};

#endif
