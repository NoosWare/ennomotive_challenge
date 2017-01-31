#ifndef _object_detect_HPP
#define _object_detect_HPP
#include "includes.ihh"

/**
 * @brief detect object contours in the foreground
 * ...
 */
namespace cv_detect
{

struct line
{
    int x;
    int y;
    float yaw;
    float size;

    bool operator()(const line a, const line b) {
        return (a.size > b.size);
    }
}


/// @brief do a min-max calculation (unsigned, e.g. 0~1)
float min_max_unsigned(float value, float min, float max);

/// @brief signed min-max calculation (e.g, -1 ~ +1)
float min_max_signed(float value, float min, float max);


/// @brief mask ROI and return a cv::Mat of the trapezoid
cv::Mat region_of_interest(const cv::Mat & image);

/// @brief obtain a min-max value of contour pixels in trapezoid ROI
std::string contour_pixels(const cv::Mat & gray);

/// @brief obtain a list of lines inside the frame
/// @return json string
/// @note: each line is made of (x, y, θ, μ)
///        where x,y is origin θ is angle and μ is size
/// @warning: all values are min-max normalised
///
std::string find_lines(const cv::Mat & image);

/// @brief search for a red circle (traffic light)?
cv::Mat find_red_circle(
                        const cv::Mat & image,
                        const cv::Mat & gray
                       );

/// @brief class handler for ORB
/// @note  use for traffic light region detection
///        and possibly for detecting ramps?
class orb
{
public:

    /// construct with a model
    orb(const cv::Mat & model);

    /// find features in frame
    cv::Mat match(
                    const cv::Mat & image,
                    const cv::Mat & gray
                 );

private:

    cv::Ptr<cv::FeatureDetector> orb__;
    std::vector<cv::KeyPoint> model_keys__;
    cv::Mat model_desc__;
    cv::Mat model__;
};


struct qr
{
    struct point
    {
        unsigned int x;
        unsigned int y;
    };

    point top_left;
    point top_right;
    point bot_left;
    point bot_right;
    std::string data;
};

/// @brief class handler for QR scanner
class qr_scan
{
public:
    qr_scan(); 

    std::vector<qr> scan(const cv::Mat & gray);
    
private:
    zbar::ImageScanner scanner__;
};

}
#endif
