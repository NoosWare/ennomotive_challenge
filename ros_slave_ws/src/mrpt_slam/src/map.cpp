#include "map.hpp"

map::map() {

    map1_ = mrpt::maps::CSimplePointsMap::Create();
}

void map::laser_observation(
                             mrpt::obs::CObservation2DRangeScan obs,
                             mrpt::poses::CPose3D new_robotpose
                           ) 
{
    map1_->insertObservation(obs, new_robotpose);
}

void map::destroy_map() {

    map1_->clear();
}

void map::save_to_file(std::string filename)
{
    map1_->save2D_to_text_file(filename);   
}
