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

    /// @brief find ORB features
    void orb_features(const cv::Mat & image, const cv::Mat & model);

/// @brief calss handler for ORB
class orb
{
public:

    /// construct with a model
    orb(const cv::Mat & model);

    /// find features in frame
    cv::Mat find_features(const cv::Mat & image);

private:

    cv::Ptr<cv::FeatureDetector> orb__;
    std::vector<cv::KeyPoint> model_keys__;
    cv::Mat model_desc__;
    cv::Mat model__;
};

}
#endif
