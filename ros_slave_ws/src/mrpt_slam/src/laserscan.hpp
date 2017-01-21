#ifndef LASERSCAN_HPP
#define LASERSCAN_HPP
#include <mrpt/hwdrivers/CRoboPeakLidar.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/utils/CTicTac.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/threads.h> // sleep
#include <mrpt/system/os.h>
#include <iostream>

/**
 * @brief class to scan enviroment with ARPLidar
 * @version 0.1.0
 * @date 18-01-2017
 * @author Maria Ramos
 */
class laserscan 
{
public: 

    /// @brief Constructor
    laserscan();

    ///@brief Do an observation
    mrpt::slam::CObservation2DRangeScan scan_observation();

private:

    ///Laser object
    mrpt::hwdrivers::CRoboPeakLidar  lidar;
    ///Path device
    std::string com_device = "/dev/ttyUSB0";
};

#endif  
