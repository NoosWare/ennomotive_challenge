#include "laserscan.hpp"

laserscan::laserscan() {

    lidar.setSerialPort(com_device);
    if (!lidar.turnOn()) {
        throw std::runtime_error("Laser not connected");
    }
}

mrpt::slam::CObservation2DRangeScan laserscan::scan_observation() 
{
    mrpt::slam::CObservation2DRangeScan obs;
    bool t_observation, hard_error;
    lidar.doProcessSimple(t_observation, obs, hard_error);

    if (hard_error)
        throw std::runtime_error("Hardware laser error");

    if (!t_observation)
    {
        throw std::runtime_error("No observation from laser available");
    }

    return obs;

}
