#include "index.hpp"

std::string index_lines(const cv::Mat & image)
{
    //auto time = ros::Time::now().toSec();
    ros::Time::now().toSec();
    std::string time_str = boost::lexical_cast<std::string>(time);
    time_str = "lines/" + time_str = ".jpg";
    cv::imwrite(time_str, image);
    return time_str;
}
