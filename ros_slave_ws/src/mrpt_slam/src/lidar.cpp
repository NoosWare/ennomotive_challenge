#include "lidar.hpp"

lidar::lidar() 
{
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
    if (!drv){
        throw std::runtime_error("failed to acquire driver ptr");
    }
    drv__ = drv;
    // make connection...
    if (IS_FAIL(drv__->connect(port__.c_str(), 115200))) {
        throw std::runtime_error("Can't connect with ttyUSB0");
    }
    rplidar_response_device_info_t devinfo;
    u_result op_result = drv__->getDeviceInfo(devinfo);

    // check health:
    getDeviceHealth();
}

lidar::~lidar()
{
    assert(drv__);
    RPlidarDriver::DisposeDriver(drv__); 
}

void lidar::start()
{
    assert(drv__);
    drv__->startMotor();
    drv__->startScan();
}

void lidar::stop()
{
    assert(drv__);
    drv__->stop();
    drv__->stopMotor();
}

void lidar::getDeviceHealth() const
{
    assert(drv__);
    u_result     op_result;
    rplidar_response_device_health_t healthinfo;
    op_result = drv__->getHealth(healthinfo);
    if (IS_OK(op_result)) {
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            throw std::runtime_error("RPLIDAR_STATUS_ERROR");
        }
        else {
            return;
        }
    }
    else {
        throw std::runtime_error("failed to acquire RPLIDAR health");
    }
}


void lidar::scan()
{
    assert(drv__);
    rplidar_response_measurement_node_t nodes[360 * 2];
    size_t count = _countof(nodes);
    u_result op_result = drv__->grabScanData(nodes, count);

    if (IS_OK(op_result)) {
        drv__->ascendScanData(nodes, count);
        for (int pos = 0; pos < (int)count; ++pos) {
            printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                nodes[pos].distance_q2/4.0f,
                nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
        }
    }
}
