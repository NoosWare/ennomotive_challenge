#ifndef _object_detect_HPP
#define _object_detect_HPP
#include "includes.ihh"

/**
 * @brief detect object contours in the foreground
 * ...
 */
class object_detect
{
public:

    cv::Mat grub_cut(const cv::Mat & image) const;

    cv::Mat convex_roi(const cv::Mat & image) const;

};
#endif
