#include "planner.hpp"

planner::planner()
: path_planner__()
{
    path_planner__.robotRadius = 0.25f;
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
                               mrpt::poses::CPose2D(-0.59, 0.53, 0),
                               path,
                               path_found, 
                               -1); //default
     
    if (!path_found){
        std::cout << " No path found " << std::endl;
    }
    else {
        std::cout << "Path found with: " << path.size() << " steps" << std::endl;    
    }
    return path;
}

