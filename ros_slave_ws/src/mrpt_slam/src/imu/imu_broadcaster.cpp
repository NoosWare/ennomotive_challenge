#include "imu_broadcaster.hpp"

imu_broadcaster::imu_broadcaster()
: imu__("/dev/i2c-1"), 
  start__(millis())
{
    std::string mode = "normal";
    //std::string i2cDevice = ;
    //MinIMU9 imu(i2cDevice.c_str());
    imu__.loadCalibration();
    imu__.enable();
    imu__.measureOffsets();
    quaternion rotation__ = quaternion::Identity();
    previous_velocity__ = vector(0.0, 0.0, 0.0);
    previous_position__ = vector(0.0, 0.0, 0.0);
}

/// @brief broadcast once
//std::pair<quaternion, vector> imu_broadcaster::read()
void imu_broadcaster::read(const std::shared_ptr<mrpt::slam::CObservationIMU> obs)
{
    int last_start = start__;
    start__ = millis();
    // dt is seconds (or decimal) used to calculate radians per second
    dt__ = (start__ - last_start) / 1000.0;
    if (dt__ < 0){ 
        throw std::runtime_error("Time went backwards."); 
    }
    // angular velocity => gyroscope
    vector angular_velocity = imu__.readGyro();
    // linear acceleration => accelelerometer
    vector acceleration = imu__.readAcc();
    //acceleration -= vector(0.0030f, 0.0051f, 0.250f);
    // magnetic field in 3 axes
    vector magnetic_field = imu__.readMag();
    consume(rotation__, dt__, angular_velocity, acceleration, magnetic_field);

    obs->rawMeasurements[mrpt::slam::IMU_X_ACC] = acceleration[0];
    obs->rawMeasurements[mrpt::slam::IMU_Y_ACC] = acceleration[1];
    obs->rawMeasurements[mrpt::slam::IMU_Z_ACC] = acceleration[2];
    obs->dataIsPresent[mrpt::slam::IMU_X_ACC] = true;
    obs->dataIsPresent[mrpt::slam::IMU_Y_ACC] = true;
    obs->dataIsPresent[mrpt::slam::IMU_Z_ACC] = true;

    obs->rawMeasurements[mrpt::slam::IMU_YAW_VEL]   = angular_velocity[2];
    obs->rawMeasurements[mrpt::slam::IMU_PITCH_VEL] = angular_velocity[1];
    obs->rawMeasurements[mrpt::slam::IMU_ROLL_VEL]  = angular_velocity[0];
    obs->dataIsPresent[mrpt::slam::IMU_YAW_VEL] = true;
    obs->dataIsPresent[mrpt::slam::IMU_PITCH_VEL] = true;
    obs->dataIsPresent[mrpt::slam::IMU_ROLL_VEL] = true;

    obs->rawMeasurements[mrpt::slam::IMU_MAG_X] = magnetic_field[0];
    obs->rawMeasurements[mrpt::slam::IMU_MAG_Y] = magnetic_field[1];
    obs->rawMeasurements[mrpt::slam::IMU_MAG_Z] = magnetic_field[2];
    obs->dataIsPresent[mrpt::slam::IMU_MAG_X] = true;
    obs->dataIsPresent[mrpt::slam::IMU_MAG_Y] = true;
    obs->dataIsPresent[mrpt::slam::IMU_MAG_Z] = true;
}

unsigned int imu_broadcaster::millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

void imu_broadcaster::consume(
                                quaternion & rotation, 
                                float dt, 
                                const vector & angular_velocity,
                                const vector & acceleration, 
                                const vector & magnetic_field
                             )
{
    vector correction = vector(0, 0, 0);
    if (abs(acceleration.norm() - 1) <= 0.3) {
        // The magnetidude of acceleration is close to 1 g, so
        // it might be pointing up and we can do drift correction.
        const float correction_strength = 1;
        // 
        matrix rotationCompass = rotationFromCompass(acceleration, magnetic_field);
        matrix rotationMatrix = rotation.toRotationMatrix();
        correction = (
            rotationCompass.row(0).cross(rotationMatrix.row(0)) +
            rotationCompass.row(1).cross(rotationMatrix.row(1)) +
            rotationCompass.row(2).cross(rotationMatrix.row(2))
          ) * correction_strength;
    }
    rotate(rotation, angular_velocity + correction, dt);
    ///
    make_velocity(acceleration);
}

void imu_broadcaster::make_velocity(vector acceleration)
{
    previous_position__ = previous_position__ 
                        + (previous_velocity__ * dt__) 
                        + (acceleration * (dt__ * dt__) / 2.f );

    previous_velocity__ = (previous_velocity__ + (acceleration * dt__));
}

//mrpt::poses::CPose3D imu_broadcaster::convert_to_3dpose() 
//{
//    std::pair<quaternion, vector> pose = read();
//    return mrpt::poses::CPose3D(mrpt::math::CQuaternionDouble((double)pose.first.w(),
//                                                             (double)pose.first.x(),
//                                                             (double)pose.first.y(),
//                                                             (double)pose.first.z()),
//                                pose.second(0),
//                                pose.second(1),
//                                pose.second(2));
//}

vector imu_broadcaster::get_velocity()
{
    return previous_velocity__;
}

