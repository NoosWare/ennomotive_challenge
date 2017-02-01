#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/slam/COccupancyGridMap2D.h>
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/slam/CPathPlanningCircularRobot.h>
#include <mrpt/math/lightweight_geom_data.h>

#include "std_msgs/String.h"
#include <math.h>

#define RAD2DEG(x) ((x)*180./M_PI)
#define M_PIf 3.14159265358979f

class planner 
{
public:

    planner();

    // callback for laser data
    std::deque<mrpt::math::TPoint2D> path_planner(
                                                  const mrpt::slam::COccupancyGridMap2D & map,
                                                  mrpt::poses::CPose3D start,
                                                  mrpt::poses::CPose3D goal
                                                 ); 

private:

    ///calculate the path
    mrpt::slam::CPathPlanningCircularRobot path_planner__;

};

#endif
