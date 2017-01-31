#include "index.hpp"

std::string index_lines(const cv::Mat & image)
{
    //auto time = ros::Time::now().toSec();
    double time = ros::Time::now().toSec();
    std::string time_str = boost::lexical_cast<std::string>(time);
    return time_str;
}
