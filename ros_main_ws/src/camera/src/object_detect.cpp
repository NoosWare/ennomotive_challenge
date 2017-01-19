#include "object_detect.hpp"

cv::Mat object_detect::grub_cut(const cv::Mat & image) const
{
    cv::Mat im32fc3;
    cv::Mat hist;
    cv::Mat backpr32f;
    cv::Mat backpr8u;
    cv::Mat backprBw;

    image.convertTo(im32fc3, CV_32FC3);
    static const float   rgbRange[]  = {0, 256};
    static const float * ranges[]    = {rgbRange, rgbRange, rgbRange};
    static const int     channels[]  = {0, 1, 2};
    static const int     hist_size[] = {32, 32, 32};
    cv::calcHist(&im32fc3, 1, channels, cv::Mat(), hist, 3, hist_size, ranges, true, false);
    cv::calcBackProject(&im32fc3, 1, channels, hist, backpr32f, ranges);

    double minval, maxval;
    cv::minMaxIdx(backpr32f, &minval, &maxval);
    cv::threshold(backpr32f, backpr32f, (maxval/32), 255, cv::THRESH_TOZERO);
    backpr32f.convertTo(backpr8u, CV_8U, (255.0/maxval));
    threshold(backpr8u, backprBw, 10, 255, cv::THRESH_BINARY);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::dilate(backprBw, backprBw, kernel);
    cv::morphologyEx(backprBw, backprBw, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

    backprBw = 255 - backprBw;

    cv::morphologyEx(backprBw, backprBw, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);
    erode(backprBw, backprBw, kernel);

    cv::Mat mask(backpr8u.rows, backpr8u.cols, CV_8U);
    mask.setTo(cv::GC_PR_BGD);
    mask.setTo(cv::GC_PR_FGD, backprBw);

    cv::Mat bgdModel, fgdModel;
    cv::grabCut(image, mask, cv::Rect(), bgdModel, fgdModel, cv::GC_INIT_WITH_MASK);

    cv::Mat fg = mask == cv::GC_PR_FGD;
    return fg;
}

cv::Mat object_detect::convex_roi(const cv::Mat & image) const
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
    /// Detect edges using Threshold
    cv::threshold(img_gray, threshold_output, 50, 255, cv::THRESH_BINARY);
    /// Find contours
    cv::findContours(threshold_output, 
                    contours, 
                    hierarchy,
                    cv:: RETR_CCOMP, 
                    cv::CHAIN_APPROX_TC89_KCOS, 
                    cv::Point(0, 0));
    /// Find the convex hull object for each contour
    std::vector<std::vector<cv::Point>> detected(contours.size());
    for (int i = 0; i < contours.size(); i++){
        cv::convexHull(cv::Mat(contours[i]), detected[i], false); 
        //cv::approxPolyDP(cv::Mat(contours[i]), detected[i], 3, true);
    }
    /// Draw contours + hull results
    cv::Mat drawing = cv::Mat::zeros(threshold_output.size(), CV_8UC3);
    for (int i = 0; i < contours.size(); i++) {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
        cv::drawContours(drawing, contours, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
        cv::drawContours(drawing, detected, i, color, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
    }
    // run watershed segmentation using the markers
    cv::watershed(drawing, markers);
    // extract the ROI (polygon/trapezoid)
    // create a mask to use when copying the result
    // see: http://www.pieter-jan.com/node/5
    cv::Mat mask = cv::Mat::zeros(drawing.rows, drawing.cols, CV_8UC1);
    cv::Point ptss[4] = {
        cv::Point(180, 280),
        cv::Point(0,   480),
        cv::Point(640, 480),
        cv::Point(460, 280)
    };
    cv::fillConvexPoly(mask, ptss, 4, cv::Scalar(255, 8, 0));
    cv::Mat result(drawing.rows, drawing.cols, CV_8UC1);
    drawing.copyTo(result, mask);
    cvtColor(result, result,CV_RGB2GRAY);
    return result;
}
