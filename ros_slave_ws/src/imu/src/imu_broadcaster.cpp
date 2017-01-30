#include "imu_broadcaster.hpp"
#include "json.hpp"

imu_broadcaster::imu_broadcaster()
: imu__("/dev/i2c-1"), 
  start__(millis())
{
    std::string mode = "normal";
    imu__.loadCalibration();
    imu__.enable();
    imu__.measureOffsets();
    rotation__ = quaternion::Identity();
    prev_raw_accel__ = vector(0.0, 0.0, 0.0);// vector(-0.0122, -0.03538, 1.05359);
    prev_filt_accel__ = vector(0.0, 0.0, 0.0);// vector(-0.0040875, 0.00171853, 0.00699055);
}

std::string imu_broadcaster::read()
{
    int last_start = start__;
    start__ = millis();
    // dt is seconds (or decimal) used to calculate radians per second
    dt__ = (start__ - last_start) / 1000.0;
    if (dt__ < 0){ 
        throw std::runtime_error("Time went backwards."); 
    }
    // angular velocity => gyroscope
    vector angular = imu__.readGyro();
    vector magnetic = imu__.readMag();
    vector linear = imu__.readAcc();
    vector filtered = dc_block_filter(vector(0.999, 0.999, 0.999), 
                                      linear,
                                      prev_raw_accel__,
                                      prev_filt_accel__);
    prev_filt_accel__ = filtered;
    prev_raw_accel__  = linear;
    to_quaternion(rotation__, dt__, angular, linear, magnetic);
    // 2 = z, 1 = y, 0 = x (Quaternion to Euler)
    vector euler = (rotation__.toRotationMatrix().eulerAngles(2, 1, 0));

    // to JSON, to std::string
    nlohmann::json json = {
        {"linear", {{"x", filtered[0]}, 
                    {"y", filtered[1]},
                    {"z", filtered[2]}}},
        {"angular",{{"x", angular[0]}, 
                    {"y", angular[1]},
                    {"z", angular[2]}}},
        {"euler",  {{"x", euler[0]},
                    {"y", euler[1]},
                    {"z", euler[2]}}},
        {"magnetic",{{"x", magnetic[0]},
                     {"y", magnetic[1]},
                     {"z", magnetic[2]}}}};
    return json.dump();
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
        const float correction_strength = 1;
        matrix rotationCompass = rotationFromCompass(acceleration, magnetic_field);
        matrix rotationMatrix = rotation.toRotationMatrix();
        correction = (
            rotationCompass.row(0).cross(rotationMatrix.row(0)) +
            rotationCompass.row(1).cross(rotationMatrix.row(1)) +
            rotationCompass.row(2).cross(rotationMatrix.row(2))
          ) * correction_strength;
    }
    rotate(rotation, angular_velocity + correction, dt);
}

vector imu_broadcaster::dc_block_filter(
                                         vector R,
                                         vector raw_data,
                                         vector previous_raw_data,
                                         vector previous_filtered_data
                                       )
{
    return vector(raw_data[0] - previous_raw_data[0] + R[0] * previous_filtered_data[0],
                  raw_data[1] - previous_raw_data[1] + R[1] * previous_filtered_data[1],
                  raw_data[2] - previous_raw_data[2] + R[2] * previous_filtered_data[2]);
}

