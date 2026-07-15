#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "color_detector/msg/detected_target_array.hpp"
#include "color_detector/msg/detected_target.hpp"
#include "color_detector/ColorClassiFication.h"
#include "color_detector/target.h"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

/* Global definitions from original main.cpp */
int cnt[7] = {0};
float target::minArea = 5000;
float target::maxArea = 12000;

std::unordered_set<COLORS> ColorClassiFication::COLORS_SET{
    PURPLE, GREEN, BLUE, ORANGE, YELLOW, RED, BROWN, WHITE};

std::unordered_map<int, COLORS> target::color_status = {
    {39, YELLOW}, {57, YELLOW}, {60, PURPLE}, {15, PURPLE},
    {30, BLUE},   {51, GREEN},  {58, BROWN},  {23, BROWN}};

std::unordered_map<int, bool> target::status = {
    {39, 1}, {57, 0}, {60, 0}, {15, 1},
    {51, 0}, {30, 0}, {58, 0}, {23, 1}};

std::map<COLORS, cv::Vec3b> standard_colors = {
    {PURPLE, cv::Vec3b(120, 105, 0)}, {GREEN, cv::Vec3b(47, 210, 0)},
    {YELLOW, cv::Vec3b(24, 255, 0)},  {BLUE, cv::Vec3b(99, 250, 0)},
    {ORANGE, cv::Vec3b(17, 250, 0)},  {RED, cv::Vec3b(0, 220, 0)},
    {BROWN, cv::Vec3b(19, 170, 0)},   {WHITE, cv::Vec3b(120, 10, 0)}};

std::map<COLORS, std::pair<cv::Scalar, cv::Scalar>> HSVThresholds = {
    {PURPLE, {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {GREEN,  {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {YELLOW, {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {BLUE,   {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {ORANGE, {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {RED,    {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {BROWN,  {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}},
    {WHITE,  {cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 0)}}};

std::map<COLORS, std::string> colorToString = {
    {PURPLE, "PURPLE"}, {GREEN, "GREEN"},   {BLUE, "BLUE"},
    {ORANGE, "ORANGE"}, {YELLOW, "YELLOW"}, {RED, "RED"},
    {BROWN, "BROWN"},   {WHITE, "WHITE"}};

class DetectionNode : public rclcpp::Node {
public:
    DetectionNode() : Node("detection_node") {
        target_pub_ = this->create_publisher<color_detector::msg::DetectedTargetArray>(
            "/detected_targets", 1);

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 1,
            std::bind(&DetectionNode::imageCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Detection node started, waiting for camera images...");
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msgImg) {
        RCLCPP_INFO(this->get_logger(), "Image received, processing...");

        /* Convert ROS image to OpenCV */
        cv_bridge::CvImagePtr cvImgPtr;
        try {
            cvImgPtr = cv_bridge::toCvCopy(msgImg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat src = cvImgPtr->image;
        cv::Mat Src = src.clone();

        /* Save image for Python processing */
        std::string data_dir = DATA_DIR;
        std::string work_dir = "/tmp/color_detector";
        system(("mkdir -p " + work_dir).c_str());
        cv::imwrite(work_dir + "/src.png", src);

        /* Call Python k_means */
        std::string cmd = "python3 " + data_dir + "/script/k_means.py " + work_dir;
        RCLCPP_INFO(this->get_logger(), "Running: %s", cmd.c_str());
        int ret = system(cmd.c_str());
        if (ret != 0) {
            RCLCPP_WARN(this->get_logger(), "k_means.py returned %d", ret);
        }

        /* Read processed image */
        cv::Mat processed = cv::imread(work_dir + "/dst.png");
        if (processed.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read dst.png");
            return;
        }

        /* Grayscale + threshold + morphology */
        cv::cvtColor(processed, processed, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(processed, processed, cv::Size(5, 5), 0);
        cv::threshold(processed, processed, 0, 255, cv::THRESH_OTSU);
        processed = ~processed;

        cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
        cv::morphologyEx(processed, processed, cv::MORPH_ERODE, kernel3);

        /* Detect targets */
        std::map<COLORS, std::vector<target>> targetSet;
        target::findTarget(processed, targetSet);

        /* Build and publish message */
        auto msg = std::make_unique<color_detector::msg::DetectedTargetArray>();

        for (auto &[k, v] : targetSet) {
            RCLCPP_INFO(this->get_logger(), "%s has %zu targets",
                        colorToString[k].c_str(), v.size());
            cnt[(int)k] = v.size();

            for (auto &t : v) {
                color_detector::msg::DetectedTarget dt;
                dt.color_id = (int32_t)k;
                dt.x = t.position.x;
                dt.y = t.position.y;
                dt.yaw = t.Yaw;
                dt.sta = t.sta;
                dt.updown = t.updown;
                msg->targets.push_back(dt);
            }
        }

        for (int i = 0; i < 7; i++) {
            msg->color_counts[i] = cnt[i];
        }

        target_pub_->publish(std::move(msg));
        RCLCPP_INFO(this->get_logger(), "Published %zu targets", targetSet.size());

        /* Show preview (disabled to avoid lag) */
        // for (auto &[k, v] : targetSet) {
        //     for (auto &t : v) {
        //         cv::circle(Src, t.position, 3, cv::Scalar(255, 255, 255), -1);
        //         target::DrawRect(Src, t.getRect(), cv::Scalar(255, 255, 255));
        //     }
        // }
        // cv::imshow("Detection Result", Src);
        // cv::waitKey(1);
    }

    rclcpp::Publisher<color_detector::msg::DetectedTargetArray>::SharedPtr target_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
