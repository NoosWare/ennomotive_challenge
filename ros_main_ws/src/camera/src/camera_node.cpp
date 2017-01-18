/**
 * @brief main entry point for camera node
 * @version 0.1.0
 * @date 18.01.2017
 * @author Alex Giokas
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <raspicam/raspicam_cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/program_options.hpp>

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
        camera.set(CV_CAP_PROP_SATURATION, 20);
    }
    if (vm.count("brightness")) {
        camera.set(CV_CAP_PROP_BRIGHTNESS, vm["brightness"].as<int>());
    }
    else {
        camera.set(CV_CAP_PROP_BRIGHTNESS, 70);
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
    auto pub = n.advertise<std_msgs::String>("cvision", 1000);
    ros::Rate loop_rate(10);
    cv::Mat image;

    // on a loooooop
    //while (ros::ok()) {
        camera.grab();
        camera.retrieve(image);
        // TODO: run all the cv algorithms here
    //}

    std::ostringstream os;
    os << "image_test.jpg";
    cv::imwrite(os.str(), image);

    camera.release();
    return 0;
}
