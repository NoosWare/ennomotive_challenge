#ifndef _object_detect_HPP
#define _object_detect_HPP
#include "includes.ihh"

/**
 * @brief detect object contours in the foreground
 * ...
 */
namespace cv_detect
{
    typedef std::tuple<int,int,float,float> hough_line;

    cv::Mat region_of_interest(const cv::Mat & image);

    /// @brief obtain a matrix of contour points
    int contour_pixels(const cv::Mat & image);

    /// @brief obtain a list of lines
    /// @return is a pair of (x, y, θ, μ)
    /// where x,y is origin θ is angle and μ is size
    std::vector<hough_line> find_lines(const cv::Mat & image);
}
#endif
