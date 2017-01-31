#ifndef SENSOR_FUSION_HPP
#define SENSOR_FUSION_HPP
#include "logger.hpp"
#include "robot_action.hpp"

class sensor_fusion 
{
public:
    
    /**
     * WARNING: neural vector has: [QR],[COLLISION],[CONTOUR PIXELS][TRAFFIC][LINES]
     *                      count:   4    4           1               1        20
     */

    sensor_fusion(ros::NodeHandle & node, std::string logfile)
    : index(0), vector(30), labels(5),
    {
        vector.setZero();
        labels.setZero();
        qr_sub = node.subscribe("qr", 
                                100, 
                                &sensor_fusion::qr_cb, 
                                this);
        pixel_sub = node.subscribe("contours", 
                                   100, 
                                   &sensor_fusion::pixels_cb, 
                                   this);
        traffic_sub = node.subscribe("circles", 
                                     100, 
                                     &sensor_fusion::traffic_cb, 
                                     this);
        collis_sub =  node.subscribe("collision", 
                                     100, 
                                     &sensor_fusion::collision_cb, 
                                     this);
        lines_sub =  node.subscribe("lines", 
                                     100, 
                                     &sensor_fusion::lines_cb,
                                     this);
        motors_sub = node.subscribe("motors", 
                                     100, 
                                     &sensor_fusion::motors_cb,
                                     this);
    }

    /// add one sample
    void add_sample()
    {
        // append to inputs and outputs
        inputs(index) = vector;
        outputs(index) = labels;
        // set all to zero
        vector.setZero();
        labels.setZero();
        index++;
    }

    void qr_cb(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        vector[0] = (float)obj["left"];
        vector[1] = (float)obj["centre"];
        vector[2] = (float)obj["right"];
        vector[3] = (float)obj["area"];
    }

    void collision_cb(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        vector[4] = (float)(obj["front"] ? 1.f : 0.f);
        vector[5] = (float)((obj["l_front"] || obj["l_back"]) ? 1.f : 0.f);
        vector[6] = (float)((obj["r_front"] || obj["r_back"]) ? 1.f : 0.f);
        vector[7] = (float)(obj["back"] ? 1.f : 0.f);
    }

    void pixels_cb(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        vector[8] = (float)obj["contours"];
    }

    void traffic_cb(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        vector[9] = (float)(obj["traffic"] ? 1.f : 0.f);
    }

    void lines_cb(const std_msgs::String:::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        // TODO: find a member "lines" and iterate it
        //
    }

    void motors_cb(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        // same for both motors
        if (obj["left_speed"] == obj["right_speed"]) {
            // larger than zero? => forward
            if (obj["left_speed"] > 0 && obj["right_speed"] > 0){
                labels[0] = 1.f;
            }
            // else if smaller than zero => reverse
            else if (obj["left_speed"] < 0 && obj["right_speed"] < 0) {
                labels[1] = 0.f;   
            }
        }
        else {
            // left
            labels[2] = (obj["left_speed"] <  obj["right_speed"] ? 1.f : 0.f);
            // right
            labels[3] = (obj["left_speed"] >  obj["right_speed"] ? 1.f : 0.f);
            // stopped (both are equal to zero)
            labels[4] = (obj["left_speed"] == 0 && 
                         obj["right_speed"]  == 0 ? 1.f : 0.f);
        }
    }

private:

    ros::Subscriber qr_sub;
    ros::Subscriber pixel_sub;
    ros::Subscriber traffic_sub;
    ros::Subscriber collis_sub;
    ros::Subscriber lines_sub;
    ros::Subscriber motors_sub;

    unsigned int index;
    Eigen::RowVectorXf vector;
    Eigen::RowVectorXf labels;
    Eigen::MatrixXf    inputs;
    Eigen::MatrixXf    outputs;
};
