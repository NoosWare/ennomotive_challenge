#include "planner.hpp"

planner::planner()
: path_planner__()
{
    path_planner__.robotRadius = 0.2f;
}

std::deque<mrpt::math::TPoint2D> planner::path_planner(
                                                        const mrpt::slam::COccupancyGridMap2D & map,
                                                        mrpt::poses::CPose3D start,
                                                        mrpt::poses::CPose3D goal                                                                                       )
{
    std::deque<mrpt::math::TPoint2D> path;
    bool not_found = false;

    std::cout << "computePath" << std::endl;
    path_planner__.computePath(map,
                               mrpt::poses::CPose2D(start.x(), start.y(), start.yaw()),
                               mrpt::poses::CPose2D(goal.x(), goal.y(), goal.yaw()),
                               path,
                               not_found, 
                               5); //default
     
    std::cout << "Path found " << not_found << " steps: "<< path.size() << " steps" << std::endl;    
    return path;
}

