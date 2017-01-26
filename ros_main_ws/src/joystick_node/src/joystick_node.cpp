// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright Drew Noakes 2013-2016

// see: https://github.com/drewnoakes/joystick

#include "joystick.hpp"
#include <unistd.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <math.h>

/// normalise and scale Axis values to velocity values
std_msgs::String normalise_scale(
                                  int left_value,
                                  int right_value
                                )
{
    // normalize between 0 and 1
    float left = 0.f;
    float right = 0.f;
    
    // invert if needed and scale back to (-1 ~ 0) or (0 ~ 1)
    if (left_value > 0) {
        left = -(left_value / 32767.f) * 1.f;
    }
    else if (left_value < 0) {
        left = -(left_value / -32767.f) * -1.f;
    }

    // right value
    if (right_value > 0) {
        right = -(right_value / 32767.f) * 1.f;
    }
    else if (right_value < 0){
        right = -(right_value / -32767.f) * -1.f;
    }

    std_msgs::String msg;
    std::stringstream ss;

    if ((right >= -1 && right <= 1) &&
        (left >= -1 && left <= 1))
    {
       ss << "{\"left_speed\":" << left
          << ", \"right_speed\":" << right
          << "}";
        msg.data = ss.str();
        return msg;
    }
    else {
        ss << "{\"left_speed\":" << 0.f
          << ", \"right_speed\":" << 0.f
          << "}";
        msg.data = ss.str();
        return msg;
    }
}

// sample multiple events and filter the ones we care about
// then forward to `normalise_scale`
std_msgs::String sample(
                        Joystick & joystick,
                        int & left_value,
                        int & right_value
                       )
{
    bool left, right = false;
    // take 100 samples within 100ms
    for (unsigned int i = 0; i < 100; i++) {
        JoystickEvent event;
        // no event when values are steady
        if (joystick.sample(&event)) {
             if (event.isAxis()) {
                if (event.number == 1) {
                    left_value = event.value;
                    left = true;
                }
                if (event.number == 3) {
                    right_value = event.value;
                    right = true;
                }
            }
        }
        // we need at least two samples of left & right axes
        if (!left || !right) {
            usleep(1000);
        }
        else {
            break;
        } 
    }
    return normalise_scale(left_value, right_value);
}

// main loop
int main(int argc, char** argv)
{
    // Create an instance of Joystick
    Joystick joystick("/dev/input/js0");
    // Ensure that it was found and that we can use it
    if (!joystick.isFound()) {
        printf("open failed.\n");
        return 1;
    }
    // setup ROS node
    ros::init(argc, argv, "joystick_node");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<std_msgs::String>("motors", 1000);

    // 10Hz = 100ms
    ros::Rate loop_rate(10);
    int left_value = 0;
    int right_value = 0;

    // sping indefinitely
    while (ros::ok()) {
        // sample and publish
        std_msgs::String msg = sample(joystick, left_value, right_value);
        //ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
