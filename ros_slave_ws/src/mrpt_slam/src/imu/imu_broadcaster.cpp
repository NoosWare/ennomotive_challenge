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
    velocity__ = vector(0.0, 0.0, 0.0);
    position__ = vector(0.0, 0.0, 0.0);
    prev_raw_accel__ = vector(0.0, 0.0, 0.0);// vector(-0.0122, -0.03538, 1.05359);
    prev_filt_accel__ = vector(0.0, 0.0, 0.0);// vector(-0.0040875, 0.00171853, 0.00699055);
}

std::tuple<vector, vector, vector, quaternion> imu_broadcaster::read()
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
    // magnetic field in 3 axes
    vector magnetic_field = imu__.readMag();

    vector acceleration = imu__.readAcc();
    acceleration[2] -= 0.98f;
    //The position of the sensor is inverted (gravity has to be negative)
    vector filtered = dc_block_filter(vector(0.999, 0.999, 0.999), 
                                      acceleration,
                                      prev_raw_accel__,
                                      prev_filt_accel__);
    prev_filt_accel__ = filtered;
    prev_raw_accel__ = acceleration;

    to_quaternion(rotation__, dt__, angular_velocity, filtered, magnetic_field);
    auto position = calculate_position(filtered);

    return std::make_tuple(acceleration, filtered, position, rotation__);
}

unsigned int imu_broadcaster::millis()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

void imu_broadcaster::to_quaternion(
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
}

vector imu_broadcaster::calculate_position(vector acceleration)
{
    velocity__ = velocity__ + (acceleration * dt__);
    position__ = position__ + (velocity__ * dt__) + ((acceleration * (dt__ * dt__)) / 2.0f);
    //position__ = position__ + ((acceleration * (dt__ * dt__)) / 2.0f);
    return position__;
}

vector imu_broadcaster::dc_block_filter(
                                         vector R,
                                         vector raw_data,
                                         vector previous_raw_data,
                                         vector previous_filtered_data
                                       )
{
    //vector filtered_data = raw_data - previous_raw_data + R.cross(previous_filtered_data)
    return vector(raw_data[0] - previous_raw_data[0] + R[0] * previous_filtered_data[0],
                  raw_data[1] - previous_raw_data[1] + R[1] * previous_filtered_data[1],
                  raw_data[2] - previous_raw_data[2] + R[2] * previous_filtered_data[2]);
}

mrpt::poses::CPose3D imu_broadcaster::convert_to_3dpose(   
                                                         vector position,
                                                         quaternion rotation 
                                                        ) 
{
    return mrpt::poses::CPose3D(mrpt::math::CQuaternionDouble((double)rotation.w(),
                                                             (double)rotation.x(),
                                                             (double)rotation.y(),
                                                             (double)rotation.z()),
                                position(0),
                                position(1),
                                position(2));
}
