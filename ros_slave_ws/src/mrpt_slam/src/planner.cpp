#include "planner.hpp"

planner::planner()
: path_planner__()
{
    path_planner__.robotRadius = 0.2f;
}

std::deque<mrpt::math::TPoint2D> planner::path_planner(
                                                        const mrpt::slam::COccupancyGridMap2D & map,
                                                        mrpt::poses::CPose3D    robotpose
                                                      )
{
    std::deque<mrpt::math::TPoint2D> path;
    bool path_found = false;

    std::cout << "computePath" << std::endl;
    path_planner__.computePath(map,
                               mrpt::poses::CPose2D(robotpose.x(), robotpose.y(), 0),
                               mrpt::poses::CPose2D(-1.51, 0.035, 0),
                               path,
                               path_found, 
                               5); //default
     
        std::cout << "Path found " << path_found << " steps: "<< path.size() << " steps" << std::endl;    
    return path;
}

