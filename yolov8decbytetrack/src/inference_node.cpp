// Ultralytics 🚀 AGPL-3.0 License - https://ultralytics.com/license
// Simplified inference_node: only uses TensorRT inference for "car" detection and tracking.
// Armor-related code removed.

#include <iostream>
#include <opencv4/opencv2/core/types.hpp>
#include <vector>
#include <getopt.h>
#include <tuple>
#include <functional>
#include <thread>
#include <algorithm>
#include <unordered_map>
#include <nlohmann/json.hpp>
#include <fstream>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

#include "inference.h"    // TensorRT-based inference wrapper (your class)
#include "BYTETracker.h"
#include "HikCamera.h"
#include "config.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include "geometry_msgs/msg/point.hpp"
#include "tutorial_interfaces/msg/detection.hpp"
#include "tutorial_interfaces/msg/target.hpp"

using namespace std;
using namespace cv;
using namespace std::chrono;
using namespace byte_track;
using json = nlohmann::json;

// create STrack from Detection (for BYTETracker)
STrack createStrackFromDet(const Detection& detection)
{
    byte_track::Rect<float> rect
    (
        detection.box.x,
        detection.box.y,
        detection.box.width,
        detection.box.height
    );
    return STrack(rect, detection.confidence);
}

struct detection_info
{
    std::string class_name;
    cv::Point2f point;
    float confidence;
    std::string toString() const
    {
        return class_name + "(" + std::to_string(point.x) + "," + std::to_string(point.y) + ")";
    };
};

void remove_same_obj(std::vector<Detection>& output_armor)
{
    // 先按 className 分组，再按 confidence 降序排列
    std::sort(output_armor.begin(), output_armor.end(),
              [](const Detection& a, const Detection& b)
              {
                  if (a.className == b.className)
                      return a.confidence > b.confidence; // confidence 从大到小
                  return a.className < b.className; // className 排序
              });

    // unique + erase 去掉同名的，只保留第一个（confidence 最大的）
    output_armor.erase(
        std::unique(output_armor.begin(), output_armor.end(),
                    [](const Detection& a, const Detection& b)
                    {
                        return a.className == b.className;
                    }),
        output_armor.end());
}

// Run TensorRT inference for a frame using provided Inference_trt instance
static std::vector<Detection> inferencethrow(Inference_trt& inf_car, cv::Mat& frame)
{
    return inf_car.runInference_TensorRT(frame);
}

// Convert detections -> byte_track::Object vector
static std::vector<byte_track::Object> detectionsToObjects(const std::vector<Detection>& dets)
{
    std::vector<byte_track::Object> objs;
    objs.reserve(dets.size());
    for (const auto& d : dets)
    {
        byte_track::Rect<float> rect(d.box.x, d.box.y, d.box.width, d.box.height);
        // class_id not used much here; set to 0 (car) by default
        byte_track::Object obj(rect, 0, d.confidence);
        objs.push_back(obj);
    }
    return objs;
}

class inference_node : public rclcpp::Node
{
public:
    inference_node():Node("inference_node"),runOnGPU_(true),
                      tracker_(30 /*frame_rate*/, 90 /*track_buffer*/, 0.3f /*track_thresh*/, 0.6f /*high_thresh*/, 0.7f /*match_thresh*/)
    {
        Config cfg;
        hik_config.sn = cfg.hik_cfg.sn;
        hik_config.exposure = cfg.hik_cfg.exposure;
        hik_config.gain = cfg.hik_cfg.gain;
        hik_config.frame_rate = cfg.hik_cfg.frame_rate;
        hik_config.rotate_180 = cfg.hik_cfg.rotate_180;
        hik_config.log_level = cfg.hik_cfg.log_level;

        // classes for car model
        std::vector<std::string> classes_all{"car","armor","ignore","watcher","base"};
        std::vector<std::string> classes_armor{"B1", "B2", "B3", "B4", "B5", "B7","R1", "R2", "R3", "R4", "R5", "R7"};
        std::vector<std::string> classes_red{"R1", "R2", "R3", "R4", "R5", "R7"};
        std::vector<std::string> classes_blue{"B1", "B2", "B3", "B4", "B5", "B7"};

        // TensorRT engine paths (adjust to your environment)
        std::string car_engine = "/home/zqz/ros2_ws/model2/best8.engine";
        //
        Inference inf_armor("/home/zqz/ros2_ws/model2/armor.onnx", cv::Size(640, 640), classes_armor, runOnGPU_);
        inf_armor_ = std::make_unique<Inference>(inf_armor);
        // create TensorRT inference instance for car detection
        inf_car_ = std::make_unique<Inference_trt>(car_engine, cv::Size(1024,1024), classes_all, runOnGPU_);
        inf_car_->setModelConfidenceThreshold(0.25f);
        inf_car_->setLetterBoxForSquare(true);


        publisher_detection = this->create_publisher<tutorial_interfaces::msg::Detection>("detection_topic", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&inference_node::timerCallback, this));
        cv::namedWindow("Detection", cv::WINDOW_NORMAL);

        RCLCPP_INFO(this->get_logger(), "inference_node initialized");
    }

private:
    // Process tracked results: draw boxes + labels on frame and fill msg
    std::tuple<std::vector<Detection>,std::vector<cv::Point2f>> processAndPublishTracks(const std::vector<std::shared_ptr<STrack>>& tracks, cv::Mat& frame, Inference& inf_armor)
    {
        auto msg = tutorial_interfaces::msg::Detection();
        std::vector<cv::Point2f> points;
        //std::vector<std::string> classes_armor_{};
        std::vector<Detection> detections_armor;
        for (const auto& track_ptr : tracks)
        {
            if (!track_ptr) continue;
            const STrack& track = *track_ptr;
            auto rect = track.getRect();
            cv::Rect box(static_cast<int>(rect.x()), static_cast<int>(rect.y()),
                         static_cast<int>(rect.width()), static_cast<int>(rect.height()));
            //std::cout << rect.x() << "/" << rect.y() << "/" << rect.width() << "/" << rect.height() << "/" << std::endl;

            // draw box
            cv::Scalar Box_color(0, 255, 0);
            cv::rectangle(frame, box, Box_color, 2);

            // draw ID and score
            std::string trackInfo = "ID: " + std::to_string(track_ptr->getTrackId());
            std::string cof_info = "cof:" + std::to_string(track_ptr->getScore());

            cv::putText(frame, trackInfo, cv::Point(box.x + 5, std::max(box.y - 5, 10)),
                        cv::FONT_HERSHEY_DUPLEX, 0.7, cv::Scalar(255,255,255), 1, 0);
            cv::putText(frame, cof_info, cv::Point(box.x + 5, std::max(box.y - 25, 10)),
                        cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1, 0);

            // 装甲板检测（在车辆ROI内）
             if (box.x >=0 && box.y >=0 && box.x + box.width <= frame.cols && box.y + box.height <= frame.rows && box.width > 0 && box.height > 0)
            {

                try
                {
                //std::cout << "box.x:" << box.x << " y:" << box.y << " w:" << box.width << " h:" << box.height << std::endl;
                cv::Mat roi = frame(box);
                std::vector<Detection> output_armor = inf_armor.runInference(roi);
                remove_same_obj(output_armor);
                    for (const auto& detection_armor : output_armor)
                    {
                        detections_armor.push_back(detection_armor);
                        cv::Rect box_armor = detection_armor.box;
                        cv::Scalar armor_color(255, 0, 0); // 蓝色表示装甲板

                        if (box_armor.x >= 0 && box_armor.y >= 0 &&
                            box_armor.x + box_armor.width <= roi.cols &&
                            box_armor.y + box_armor.height <= roi.rows )
                        {
                        
                        cv::rectangle(roi, box_armor, armor_color, 2);

                        std::string classString_armor = detection_armor.className + ' ' + 
                                                    std::to_string(detection_armor.confidence).substr(0, 4);
                        
                        cv::Size textSize_armor = cv::getTextSize(classString_armor, cv::FONT_HERSHEY_DUPLEX, 0.5, 1, 0);
                        cv::Rect textBox_armor(box_armor.x, box_armor.y - 20, 
                                            textSize_armor.width + 5, textSize_armor.height + 5);

                        cv::rectangle(roi, textBox_armor, (0,0,0), cv::FILLED);
                        cv::putText(roi, classString_armor, cv::Point(box_armor.x + 3, box_armor.y - 5), 
                                cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(255,255,255), 1, 0);
                        //output_json_arrmor = {{"class_name:", detection_armor.className}, {"confidence:",detection_armor.confidence}};
                        //file2 << output_json_arrmor.dump(4);
                        
                        // 绘制中心点
                        cv::circle(roi, cv::Point(box_armor.x + box_armor.width/2, 
                                                box_armor.y + box_armor.height), 
                                3, cv::Scalar(0, 0, 255), -1, 8, 0);
                        points.push_back(cv::Point2f(box.x + box_armor.x + box_armor.width/2, box.y + box_armor.y + box_armor.height));
                        //classes_armor_.push_back(detection_armor.className);
                        //std::cout << "armor.x:" << box_armor.x << " y:" << box_armor.y << " w:" << box_armor.width << " h:" << box_armor.height << std::endl;
                        }
                    } 
                }
                catch (const cv::Exception& e)
                {
                    std::cerr << "Error: " << e.what() << std::endl;
                }
            }   
            else
            {
                std::cout << "invalid roi:x" << box.x << "y" << box.y << "w" << box.width << "h" << box.height << std::endl;
            }
        }

        return std::make_tuple(detections_armor, points);
    }

    void timerCallback()
    {
        std::cout << "模式选择(test/hik):" << std::endl;
        std::string mode;
        std::cin >> mode;

        if (mode == "hik")
        {
            HikCamera hik_camera(hik_config); // PixelType_Gvsp_RGB8_Packed format
            std::cout << "开始采集 (HikCamera)" << std::endl;
            bool running = true;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            while (running)
            {
                cv::Mat frame_rgb = hik_camera.getLatestFrame();
                cv::cvtColor(frame_rgb, frame, cv::COLOR_RGB2BGR);

                if (frame.empty())
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to get frame");
                    return;
                }

                // --- 推理: 车辆 (TensorRT) ---
                std::vector<Detection> detections;
                try 
                {
                    detections = inferencethrow(*inf_car_, frame);
                } 
                catch (const std::exception& e) 
                {
                    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                    return;
                }

                // Convert detections to byte_track::Object and update tracker
                std::vector<byte_track::Object> objects = detectionsToObjects(detections);
                std::vector<std::shared_ptr<STrack>> tracked = tracker_.update(objects);


                // draw and publish
                std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> result = processAndPublishTracks(tracked, frame, *inf_armor_);
                std::vector<Detection> result_detections_armor = std::get<0>(result);
                std::vector<cv::Point2f> result_points = std::get<1>(result);
                std::vector<std::string> classes_name;
                auto msg = tutorial_interfaces::msg::Detection();
                for (int i = 0; i < result_detections_armor.size() && i < result_points.size(); i++)
                {
                    tutorial_interfaces::msg::Target target_msg;
                        //msg.class_number = result_detections_armor.size();
                    target_msg.confidence = result_detections_armor[i].confidence;
                    target_msg.class_name = result_detections_armor[i].className;
                    target_msg.x = result_points[i].x;
                    target_msg.y = result_points[i].y;
                    RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", target_msg.class_name.c_str(), target_msg.x, target_msg.y, target_msg.confidence);
                    msg.targets.push_back(target_msg);

                        
                        //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);
                        
                }
                publisher_detection->publish(msg);

                cv::imshow("Detection", frame);
                if (cv::waitKey(1) == 27)
                {
                    cv::destroyAllWindows();
                    running = false;
                }
            }
        }
        else if (mode == "test")
        {
            cap_.open("/home/zqz/ros2_ws/image/raw.mp4");
            if (!cap_.isOpened())
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to open video");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Video opened");
            bool running = true;
            while (running)
            {
                cap_ >> frame;
                //cv::Mat frame = cv::imread("/home/zqz/fjut_radar/image/test_image.jpg");
                if (frame.empty())
                {
                    RCLCPP_INFO(this->get_logger(), "Video finished");
                    return;
                }

                // --- 推理: 车辆 (TensorRT) ---
                std::vector<Detection> detections;
                try {
                    detections = inferencethrow(*inf_car_, frame);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Inference error: %s", e.what());
                    return;
                }

                // tracking
                std::vector<byte_track::Object> objects = detectionsToObjects(detections);
                std::vector<std::shared_ptr<STrack>> tracked = tracker_.update(objects);


                // draw and publish
                std::tuple<std::vector<Detection>, std::vector<cv::Point2f>> result = processAndPublishTracks(tracked, frame, *inf_armor_);
                std::vector<Detection> result_detections_armor = std::get<0>(result);
                std::vector<cv::Point2f> result_points = std::get<1>(result);
                std::vector<std::string> classes_name;
                auto msg = tutorial_interfaces::msg::Detection();
                for (int i = 0; i < result_detections_armor.size() && i < result_points.size(); i++)
                {
                    tutorial_interfaces::msg::Target target_msg;
                        //msg.class_number = result_detections_armor.size();
                    target_msg.confidence = result_detections_armor[i].confidence;
                    target_msg.class_name = result_detections_armor[i].className;
                    target_msg.x = result_points[i].x;
                    target_msg.y = result_points[i].y;
                    RCLCPP_INFO(this->get_logger(), "armor_result: %s, %f, %f , %f,", target_msg.class_name.c_str(), target_msg.x, target_msg.y, target_msg.confidence);
                    msg.targets.push_back(target_msg);

                        
                        //RCLCPP_INFO(this->get_logger(), "一共有: %i 个目标", msg.class_number);
                        
                }
                publisher_detection->publish(msg);
                cv::imshow("Detection", frame);
                if (cv::waitKey(1) == 27)
                {
                    cv::destroyAllWindows();
                    running = false;
                }
            }
        }
    }


    // members
    bool runOnGPU_;
    std::string colcor_ = "blue";
    std::vector<std::string> classes_car_;
    std::vector<std::string> classes_armor_;
    std::unique_ptr<Inference_trt> inf_car_;
    std::unique_ptr<Inference> inf_armor_;
    BYTETracker tracker_;
    cv::VideoCapture cap_;
    rclcpp::Publisher<tutorial_interfaces::msg::Detection>::SharedPtr publisher_detection;
    rclcpp::TimerBase::SharedPtr timer_;
    Config::HikConfig hik_config;
    cv::Mat frame;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<inference_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}