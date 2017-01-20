#include "object_detect.hpp"

cv::Mat cv_detect::region_of_interest(const cv::Mat & image)
{
    // extract the ROI (polygon/trapezoid)
    // create a mask to use when copying the result
    // see: http://www.pieter-jan.com/node/5
    cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8UC1);
    // adjust ROI
    cv::Point ptss[4] = {
        cv::Point(180, 280),
        cv::Point(0,   470),
        cv::Point(640, 470),
        cv::Point(460, 280)
    };
    cv::fillConvexPoly(mask, ptss, 4, cv::Scalar(255, 8, 0));
    cv::Mat result(image.rows, image.cols, CV_8UC1);
    image.copyTo(result, mask);
    cvtColor(result, result,CV_RGB2GRAY);
    return result;
}

int cv_detect::contour_pixels(const cv::Mat & image)
{
    cv::RNG rng(12345);
    // Convert image to gray and blur it
    cv::Mat img_gray;
    cv::cvtColor(image, img_gray, CV_BGR2GRAY);
    cv::blur(img_gray, img_gray, cv::Size(3,3));
    cv::Mat markers(img_gray.size(), CV_32S);
    markers = cv::Scalar::all(0);
    cv::Mat threshold_output;

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    /// detect edges using Threshold
    cv::threshold(img_gray, threshold_output, 50, 255, cv::THRESH_BINARY);
    /// find contours
    cv::findContours(threshold_output, 
                    contours, 
                    hierarchy,
                    cv:: RETR_CCOMP, 
                    cv::CHAIN_APPROX_TC89_KCOS, 
                    cv::Point(0, 0));
    cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        cv::drawContours(drawing, contours, i, color, 3, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }
    return cv::countNonZero(region_of_interest(drawing));
}

std::vector<cv_detect::hough_line> cv_detect::find_lines(const cv::Mat & image)
{
    cv::Mat img;
    cv::Mat found;
    cv::Canny(image, img, 50, 200, 3);
    cv::cvtColor(img, found, CV_GRAY2BGR);

    std::vector<cv::Vec4i> lines;
    // 1 px = 1point, 
    // theta = PI/180
    // min # of intersections = 50
    // min # of points forming a line = 100
    // max # of gap points inside a line = 5
    cv::HoughLinesP(img, lines, 1, CV_PI/180, 50, 80, 5);

    // return a pairof (theta, size)
    std::vector<hough_line> result;

    for (std::size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
        // if either Y is too high up => ignore
        if (l[1] <= 100 || l[3] <= 100) {
            continue;
        }
        // draw lines on `found` if needed
        //cv::line(found, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
        // calculate for each line, its magnitude and angle
        float dx = l[2] - l[0];
        float dy = l[3] - l[1];
        float theta = atan(dy/dx) * 180.f / M_PI;
        // calculate size of line
        float length = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if (theta != 0 && length != 0) {
            result.push_back(std::make_tuple(l[0], l[1], theta, length));
        }
    }
    return result;
}
