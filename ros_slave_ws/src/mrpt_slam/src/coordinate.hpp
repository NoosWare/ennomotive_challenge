#ifndef COORDINATE_HPP
#define COORDINATE_HPP
#include "includes.ihh"

///
inline float round(float f,float prec)
{
    return (float) (floor(f*(1.0f/prec) + 0.5)/(1.0f/prec));
}

struct coordinate
{
    float x;
    float y;
    float yaw;

    coordinate(mrpt::poses::CPose3D pose)
    : x(round(pose.x(), 0.01)), 
      y(round(pose.y(), 0.01)), 
      yaw(round(pose.yaw(), 0.01))
    {}

    void print() const
    {
        printf("%f %f %f\r\n", x, y, yaw);
    }

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & x;
        ar & y;
        ar & yaw;
    }
};
#endif
