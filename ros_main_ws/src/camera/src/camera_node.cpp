/**
 * @brief main entry point for camera node
 * @version 0.1.0
 * @date 18.01.2017
 * @author Alex Giokas
 */
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/program_options.hpp>
#include <sstream>
#include <thread>
#include <raspicam/raspicam_cv.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "object_detect.hpp"
#include "vision_processing.hpp"

void publish(
                 ros::Publisher & pub,
                 std::string json
            )
{
    std_msgs::String msg;
    std::stringstream ss;
    ss << json;
    msg.data = ss.str();
    pub.publish(msg);
}

namespace po = boost::program_options;
/**
 * @brief we use varargs to control :
 *         1. blue (whitebalance)
 *         2. brightness
 *         3. saturation
 *         4. contrast
 *
 * TODO: consider creating a video stream, and saving it to file
 *       or streaming it to a client (to view live)
 */
int main(int argc, char **argv)
{
    raspicam::RaspiCam_Cv camera;
    po::options_description desc("Options"); 
    desc.add_options()("blue,bl", po::value<int>(), "blue white balance, between [0,100] or -1 for auto") 
                      ("saturation,st", po::value<int>(), "saturation, between [0,100]") 
                      ("brightness,br", po::value<int>(), "brightness")
                      ("contrast,ct", po::value<int>(), "contrast");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("blue")) {
        camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, vm["blue"].as<int>());
    }
    else {
        camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 0);
        camera.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 100);
    }
    if (vm.count("saturation")) {
        camera.set(CV_CAP_PROP_SATURATION, vm["saturation"].as<int>());
    }
    else {
        camera.set(CV_CAP_PROP_SATURATION, 40);
    }
    if (vm.count("brightness")) {
        camera.set(CV_CAP_PROP_BRIGHTNESS, vm["brightness"].as<int>());
    }
    else {
        camera.set(CV_CAP_PROP_BRIGHTNESS, 60);
    }
    if (vm.count("contrast")) {
        camera.set(CV_CAP_PROP_CONTRAST, vm["contrast"].as<int>());
    }
    else {
        camera.set(CV_CAP_PROP_CONTRAST, 70);
    }
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    camera.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    camera.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 0);

    if (!camera.open()) {
        std::cerr << "Error opening camera" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "cvision");
    ros::NodeHandle n;

    auto qr_pub = n.advertise<std_msgs::String>("qr", 10);
    auto px_pub = n.advertise<std_msgs::String>("contours", 10);
    auto hg_pub = n.advertise<std_msgs::String>("lines", 10);
    auto tf_pub = n.advertise<std_msgs::String>("circles", 10);

    ros::Rate loop_rate(5);
    cv::Mat image;
    auto qrscan = cv_detect::qr_scan();

    while (ros::ok()) {
        camera.grab();
        camera.retrieve(image);
        // convert to grayscale now
        cv::Mat gray = cv::Mat(image.size(), CV_8UC1);
        cv::cvtColor(image, gray, CV_BGR2GRAY);

        double t = (double)cv::getTickCount(); 

        std::thread thread_contours([&]() {
            std::string pixels = cv_detect::contour_pixels(gray);
            publish(px_pub, pixels);
        });
        std::thread thread_lines([&]() {
            std::string lines = cv_detect::find_lines(image);
            if (!lines.empty()) {
                publish(hg_pub, lines);
            }
        });
        std::thread thread_qr([&]() {
            std::vector<cv_detect::qr> qrs = qrscan.scan(gray);
            if (!qrs.empty()) {
                auto qrfinder = qr_hunter(qrs);
                publish(qr_pub, qrfinder.relative_position());
            }
        });
        std::thread thread_circle([&]() {
            auto json = cv_detect::find_red_circle(image, gray);
            if (!json.empty()){
                publish(tf_pub, json);
            }
        });

        t = (double)cv::getTickCount() - t;
        //printf("eta = %gms\n", t*1000./cv::getTickFrequency());

        thread_contours.join();
        thread_lines.join();
        thread_qr.join();
        thread_circle.join();

        ros::spinOnce();
        loop_rate.sleep();
    }
    camera.release();
    return 0;
}
