#ifndef LIDAR_HPP
#define LIDAR_HPP

#include <mrpt/utils/circular_buffer.h>
#include <rplidar/rplidar.h>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdexcept>
#include <cassert>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

class lidar
{
public:
    
    /// @brief constructor
    lidar();

    /// @brief destructor
    ~lidar();

    ///@brief turn on the motor of the lidar
    void start(); 

    /// @brief turn off the motor of the lidar
    void stop();
    
    void scan();

protected:
    /// @brief get status of the lidar
    void getDeviceHealth() const;

private:
    ///driver 
    RPlidarDriver * drv__;
    ///port
    std::string port__ = "/dev/ttyUSB0";
};

#endif
