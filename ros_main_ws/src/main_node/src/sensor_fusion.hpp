#ifndef SENSOR_FUSION_HPP
#define SENSOR_FUSION_HPP
#include "logger.hpp"
#include "robot_action.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <array>
#include <OpenANN/OpenANN>
#include <OpenANN/io/Logger.h>
#include <OpenANN/io/DirectStorageDataSet.h>
#include <OpenANN/Evaluation.h>

class sensor_fusion 
{
public:
    
    /**
     * WARNING: neural vector has: [QR],[COLLISION],[CONTOUR][TRAFFIC][LINES][SUGGESTED ACTION]
     *                      count:   4    4           1       1         20       4
     */
    sensor_fusion(ros::NodeHandle & node, std::string logfile)
    : vector(35), label(5)
    {
        vector.setZero();
        label.setZero();
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
        dist_sub = node.subscribe("distance", 
                                  100, 
                                  &sensor_fusion::read_distance, 
                                  this);
    }

    /// add one sample
    void add_sample()
    {
        if (vector.sum() != 0) {
            // append to inputs and outputs
            samples.push_back(vector);
            labels.push_back(label);
            // zero them
            vector.setZero();
            label.setZero();
        }
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
        vector[9] = (float)(obj["traffic"]);
    }

    void lines_cb(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        auto line_it = obj.find("lines");
        int index = 10;
        for (auto i = line_it->begin(); i < line_it->end(); i++) {
            vector[index] = i->find("x")->get<int>();
            vector[index + 1] = i->find("y")->get<int>();
            vector[index + 2] = i->find("yaw")->get<float>();
            vector[index + 3] = i->find("size")->get<float>();
            index += 4;
        }
    }

    void read_distance(const std_msgs::String::ConstPtr & msg)
    {
        auto obj = nlohmann::json::parse(msg->data);
        std::map<action_type, float> scores;
        scores[action_type::forward] = obj["front"];
        scores[action_type::left]    = obj["left"];
        scores[action_type::right]   = obj["right"];
        scores[action_type::backward]= obj["back"];
        auto x = std::max_element(scores.begin(), scores.end(),
                                 [](const auto & p1, const auto & p2) {
                                     return p1.second < p2.second; 
                                 });
        vector[30] = (x->first == action_type::forward ? 1.f : 0.f);
        vector[31] = (x->first == action_type::left    ? 1.f : 0.f);
        vector[32] = (x->first == action_type::right   ? 1.f : 0.f);
        vector[33] = (x->first == action_type::backward? 1.f : 0.f);
    }
    
    void motors_cb(const std_msgs::String::ConstPtr & msg)
    {
        label.setZero();
        auto obj = nlohmann::json::parse(msg->data);
        // same for both motors
        if (obj["left_speed"] == obj["right_speed"]) {
            // larger than zero? => forward
            if (obj["left_speed"] > 0 && obj["right_speed"] > 0){
                label[0] = 1.f;
            }
            // else if smaller than zero => reverse
            else if (obj["left_speed"] < 0 && obj["right_speed"] < 0) {
                label[1] = 0.f;
            }
        }
        else {
            // left
            label[2] = (obj["left_speed"] <  obj["right_speed"] ? 1.f : 0.f);
            // right
            label[3] = (obj["left_speed"] >  obj["right_speed"] ? 1.f : 0.f);

            // stopped (both are equal to zero)
            label[4] = (obj["left_speed"] == 0 && 
                        obj["right_speed"]  == 0 ? 1.f : 0.f);
        }
        //std::cout << label.transpose() << std::endl;
    }

    void train_ann()
    {
        std::cout << "collected input samples: " << samples.size() << std::endl;
        std::cout << "collected output samples: " << labels.size() << std::endl;

        Eigen::MatrixXd samples_mtx(samples.size(), 35);
        Eigen::MatrixXd outputs_mtx(labels.size(), 5);

        for (unsigned int i = 0; i < samples.size(); i++) {
            samples_mtx.row(i) = samples[i];
            outputs_mtx.row(i) = labels[i];
        }

        std::ofstream filea("input.mtx");
        filea << samples_mtx;
        std::ofstream fileb("output.mtx");
        fileb << outputs_mtx;

        OpenANN::DirectStorageDataSet dataset(&samples_mtx, &outputs_mtx);
        OpenANN::useAllCores();
        auto net = std::make_unique<OpenANN::Net>();
        OpenANN::makeMLNN(*net,
                          OpenANN::RECTIFIER,
                          OpenANN::SOFTMAX,
                          35,
                          5,
                          // hidden layers: hidden nodes per hidden layer
                          1, 100);

        // set regularization: L1, L2 and max weight bounds
        net->setRegularization(0.005, 0.001, 0.0);
        net->useDropout(false);
        net->trainingSet(dataset);
        std::cout << "\t net created, training..." << std::endl;

        // stop criteria: epochs and CE stop error
        OpenANN::StoppingCriteria stop;
        stop.maximalIterations = 10000;                       // epochs
        stop.minimalValueDifferences = 0.01;               // stop error
        OpenANN::train(*net, "MBSGD", OpenANN::CE, stop);   // Mini-Batch Stochastic Gradient Descent

        double acc = OpenANN::accuracy(*net, dataset);
        double ce  = OpenANN::ce(*net, dataset);
        double hit = OpenANN::classificationHits(*net, dataset);
        double all = dataset.samples();
        std::cout << "\t - concept/relation network trained, accuracy: " << acc
                  << " Cross Entropy: " << ce
                  << " Classification Hits: " << hit << "/" << all
                  << std::endl << std::endl;
        net->save("fusion.net");
    }

private:

    ros::Subscriber qr_sub;
    ros::Subscriber pixel_sub;
    ros::Subscriber traffic_sub;
    ros::Subscriber collis_sub;
    ros::Subscriber lines_sub;
    ros::Subscriber motors_sub;
    ros::Subscriber dist_sub;

    std::vector<Eigen::VectorXd> samples;
    std::vector<Eigen::VectorXd> labels;

    Eigen::VectorXd vector;
    Eigen::VectorXd label;
};
#endif
