#ifndef _MINIMU9_HPP
#define _MINIMU9_HPP

#include "IMU.hpp"
#include "LSM303.hpp"
#include "L3G.hpp"
#include "exceptions.hpp"
#include "vector.hpp"
#include "includes.ihh"

class MinIMU9 : public IMU
{
public:

    MinIMU9(const char * i2cDeviceName);

    LSM303 compass;
    L3G gyro;

    virtual vector readAcc();
    virtual vector readMag();
    virtual vector readGyro();

    virtual void enable();
    virtual void loadCalibration();
    virtual void measureOffsets();
};

#endif
