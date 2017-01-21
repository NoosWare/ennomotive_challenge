#ifndef MAP_HPP
#define MAP_HPP
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/maps/CSimplePointsMap.h>

using namespace mrpt::utils;

class map 
{
public: 

    ///Constructor
    map();

    ///Insert an observation from laser
    void laser_observation(
                             mprt::obs::CObservation2DRangeScan & obs,
                             mrpt::poses::CPose3D new_robotpose
                          );

    /// @brief Save data into a file 
    void save_to_file(std::string filename);

    ///Destroy map
    void destroy_map();

private:
    ///Object map
    mprt::maps::CSimplePointsMapPtr  map1_;
};

#endif  
