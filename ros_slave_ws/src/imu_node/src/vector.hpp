#ifndef _vector_HPP
#define _vector_HPP

#include "includes.ihh"

// An Euler angle could take 8 chars: -234.678, but usually we only need 6.
static float field_width = 6;
#define FLOAT_FORMAT std::fixed << std::setprecision(3) << std::setw(field_width)

typedef Eigen::Vector3f vector;
typedef Eigen::Vector3i int_vector;
typedef Eigen::Matrix3f matrix;
typedef Eigen::Quaternionf quaternion;

// operators which stream the objects
static inline std::ostream & operator << (std::ostream & os, const vector & vector)
{
    return os << FLOAT_FORMAT << vector(0) << ' '
              << FLOAT_FORMAT << vector(1) << ' '
              << FLOAT_FORMAT << vector(2);
}

static inline std::ostream & operator << (std::ostream & os, const matrix & matrix)
{
    return os << (vector)matrix.row(0) << ' '
              << (vector)matrix.row(1) << ' '
              << (vector)matrix.row(2);
}

static inline std::ostream & operator << (std::ostream & os, const quaternion & quat)
{
    return os << FLOAT_FORMAT << quat.w() << ' '
              << FLOAT_FORMAT << quat.x() << ' '
              << FLOAT_FORMAT << quat.y() << ' '
              << FLOAT_FORMAT << quat.z();
}

static inline vector vector_from_ints(int (*ints)[3])
{
    return vector((float)(*ints)[0], (float)(*ints)[1], (float)(*ints)[2]);
}

static inline int_vector int_vector_from_ints(int (*ints)[3])
{
    return int_vector((*ints)[0], (*ints)[1], (*ints)[2]);
}

static inline void output_quaternion(quaternion & rotation)
{
    std::cout << rotation;
}

static inline void output_euler(quaternion & rotation)
{
    std::cout << (vector)(rotation.toRotationMatrix().eulerAngles(2, 1, 0) * (180 / M_PI));
}

//! Uses the acceleration and magnetic field readings from the compass
// to get a noisy estimate of the current rotation matrix.
// This function is where we define the coordinate system we are using
// for the ground coords:  North, East, Down.
//
static inline matrix rotationFromCompass(
                                          const vector& acceleration, 
                                          const vector& magnetic_field
                                        )
{
    vector down = -acceleration;     // usually true
    vector east = down.cross(magnetic_field); // actually it's magnetic east
    vector north = east.cross(down);
    east.normalize();
    north.normalize();
    down.normalize();
    matrix r;
    r.row(0) = north;
    r.row(1) = east;
    r.row(2) = down;
    return r;
}

// Uses the given angular velocity and time interval to calculate
// a rotation and applies that rotation to the given quaternion.
// w is angular velocity in radians per second.
// dt is the time.
//
static inline void rotate(
                            quaternion & rotation, 
                            const vector& w, 
                            float dt
                         )
{
    // Multiply by first order approximation of the
    // quaternion representing this rotation.
    rotation *= quaternion(1, w(0)*dt/2, w(1)*dt/2, w(2)*dt/2);
    rotation.normalize();
}

#endif
